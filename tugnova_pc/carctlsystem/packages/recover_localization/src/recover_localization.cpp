#include <recover_localization.h>

namespace RecoverLocalization {

    Node::Node(int _buffer_size, int _window_size, double _threshold, std::string _mode) {
        buffer_size = _buffer_size;
        window_size = _window_size;
        threshold = _threshold;
        mode = _mode;

        is_locked = false;
        is_other_initialpose = false;
        timer = ros::Time::now();

        odometry_list.clear();
        ndt_pose_list.clear();
        ndt_stat_list.clear();
    }

    /**
     * /vehicle/odomの監視.
     */
    void Node::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (is_locked) {
            return;
        }

        int odometry_buffer_size = buffer_size * 10;
        std::size_t size = odometry_list.size();

        if (size >= odometry_buffer_size) {
            int range = size - odometry_buffer_size + 1;
            odometry_list.erase(odometry_list.begin(), (odometry_list.begin() + range));
        }
        odometry_list.push_back(msg);
    }

    /**
     * /ndt_poseの監視.
     */
    void Node::ndtPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (is_locked) {
            return;
        }

        std::size_t size = ndt_pose_list.size();

        if (size >= buffer_size) {
            int range = size - buffer_size + 1;
            ndt_pose_list.erase(ndt_pose_list.begin(), (ndt_pose_list.begin() + range));
        }
        ndt_pose_list.push_back(msg);
    }

    /**
     * /ndt_statの監視.
     */
    void Node::ndtStatCallback(const autoware_msgs::NDTStat::ConstPtr& msg) {
        if (is_locked) {
            return;
        }

        std::size_t size = ndt_stat_list.size();

        if (size >= buffer_size) {
            int range = size - buffer_size + 1;
            ndt_stat_list.erase(ndt_stat_list.begin(), (ndt_stat_list.begin() + range));
        }
        RecoverLocalization::Data::Ptr data(new RecoverLocalization::Data(msg));
        ndt_stat_list.push_back(data);
    }

    /**
     *　/ndt_monitor/ndt_statusの監視.
     */
    void Node::monitorStatCallback(const std_msgs::String msg) {
        // recover_localization以外が行ったinitialpose後の動きに反応しないようにする。
        // 理由は、recover_localizationの場合のみ自動的に走行開始させたいから。他のケースでは止まったままとする。
        if (is_other_initialpose && msg.data == NDT_OK) {
            is_other_initialpose = false;
        }

        if (!is_locked) {
            return;
        }

        ros::Time now = ros::Time::now();
        if (now < timer + ros::Duration(3.0/*sec*/)) {
            return;
        }

        if (msg.data == NDT_OK || msg.data == NDT_WARNING) {
            ROS_INFO("Recover SUCCESS");
            initialize();
            publishInitialpose2();
            system("/home/nvidia/Autoware/ros/nodeshell/respawn_node.py back_blocker");
        } else if (msg.data == NDT_ERROR || msg.data == NDT_FATAL) {
            ROS_INFO("Recover FAILED");
        } else { /* NDT_NOT_INITALIZED */
            ROS_INFO("Recover Wait...");
            timer = ros::Time::now(); // 延長
        }
    }

    /**
     * 状態の初期化.
     */
    void Node::initialize() {
        if (mode == NDT_POSE) {
            ndt_pose_list.clear();
        } else {
            odometry_list.clear();
        }
        ndt_stat_list.clear();
        is_locked = false;
    }

    /**
     * /monitor_statusの監視.
     */
    void Node::healthCallback(const carctl_msgs::monitor_status& msg) {
        if (msg.service_name != LOST_LOCALIZATION // lost_localizationからのエラー通知以外は無視
        || msg.status != ERROR
        || msg.error_msg != MAIGO
        || is_locked                              // 本ノードがpublishしたinitialposeの実行中のため連投を避ける
        || is_other_initialpose                   // RVIZ等で手動による自己位置合わせに反応しない
        || ndt_stat_list.size() <= window_size) { // 記録がなければ実行しない
            return;
        }

        if (mode == NDT_POSE) {
            if (ndt_pose_list.size() <= window_size) { return; }
        } else {
            if (odometry_list.size() <= window_size) { return; }
        }

        is_locked = true;
        system("/home/nvidia/Autoware/ros/nodeshell/kill_back_blocker.sh");
        recover();
    }

    /**
     * 自己位置自動復帰の処理.
     */
    void Node::recover() {
        medianFilter();
        autoware_msgs::NDTStat::ConstPtr ndt_stat = search();
        geometry_msgs::Pose pose = getPose(ndt_stat);
        ros::Duration(10.0).sleep(); // 早くinitialposeを送信してしまうと、ndt_matching_monitorが追従できず、NDT_FATALを解除できない.
        publishInitialpose(pose);
    }

    /**
     * 自己位置合わせの受信.
     * 主にRVIZ等で手動による自己位置合わせを行った時に自己位置自動復帰の機能が反応しないようにする.
     */
    void Node::initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        is_other_initialpose = true;
    }

    /**
     * ノイズ除去処理.
     */
    void Node::medianFilter() {
        RecoverLocalization::Statistics statistics;
        int count = 0;

        while (count < ndt_stat_list.size()) {
            count += 1;
            if (count < window_size) { continue; }

            int start = count - window_size;
            std::for_each(ndt_stat_list.begin() + start, ndt_stat_list.begin() + count, [&statistics](RecoverLocalization::Data::Ptr& data) {
                autoware_msgs::NDTStat::ConstPtr ndt_stat = data->getNdtStat();
                statistics.accumulate(ndt_stat->score);
            });

            ndt_stat_list[count - 1]->setMedianResult(statistics.median());
            statistics.clear();
        }
    }

    /**
     * 変化点割り出し.
     */
    autoware_msgs::NDTStat::ConstPtr Node::search() {
        RecoverLocalization::Statistics statistics;
        int count = 0;

        while (count < ndt_stat_list.size()) {
            count += 1;
            if (count <= window_size) { continue; }

            int start = window_size - 1;
            std::for_each(ndt_stat_list.begin() + start, ndt_stat_list.begin() + count, [&statistics](RecoverLocalization::Data::Ptr& data) {
                statistics.accumulate(data->getMedianResult());
            });

            if (statistics.stdev() >= threshold) {
                break;
            }
            statistics.clear();
        }

        return ndt_stat_list[count - 1]->getNdtStat();
    }

    /**
     * initialposeに使用するPose情報を取得.
     */
    geometry_msgs::Pose Node::getPose(autoware_msgs::NDTStat::ConstPtr& ndt_stat) {
        if (mode == NDT_POSE) {
            geometry_msgs::PoseStamped::ConstPtr ndt_pose = getNdtPose(ndt_stat);
            return ndt_pose->pose;
        } else {
            nav_msgs::Odometry::ConstPtr odometry = getOdometry(ndt_stat);
            return odometry->pose.pose;
        }
    }

    /**
     * 近似のオドメトリを取得.
     */
    nav_msgs::Odometry::ConstPtr Node::getOdometry(autoware_msgs::NDTStat::ConstPtr& ndt_stat) {
        std::deque<nav_msgs::Odometry::ConstPtr>::iterator iter =
        std::find_if(odometry_list.begin(), odometry_list.end(), [&ndt_stat](nav_msgs::Odometry::ConstPtr& odometry) {
            return odometry->header.stamp >= ndt_stat->header.stamp;
        });

        nav_msgs::Odometry::ConstPtr result;
        if (iter == odometry_list.end()) {
            result = *(iter - 1);
        } else {
            result = *iter;
        }

        return result;
    }

    /**
     * 近似のndt_poseを取得.
     */
    geometry_msgs::PoseStamped::ConstPtr Node::getNdtPose(autoware_msgs::NDTStat::ConstPtr& ndt_stat){
        std::deque<geometry_msgs::PoseStamped::ConstPtr>::iterator iter =
        std::find_if(ndt_pose_list.begin(), ndt_pose_list.end(), [&ndt_stat](geometry_msgs::PoseStamped::ConstPtr& ndt_pose) {
            return ndt_pose->header.stamp >= ndt_stat->header.stamp;
        });

        geometry_msgs::PoseStamped::ConstPtr result;
        if (iter == ndt_pose_list.end()) {
            result = *(iter - 1);
        } else {
            result = *iter;
        }

        return result;
    }

    /**
     * initialposeの実行.
     */
    void Node::publishInitialpose(geometry_msgs::Pose& pose) {
        geometry_msgs::PoseWithCovarianceStamped msg;
        std_msgs::Header header;
        geometry_msgs::PoseWithCovariance pose_with_covariance;

        header.frame_id = "map";
        pose_with_covariance.pose = pose;
        pose_with_covariance.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};

        msg.header = header;
        msg.pose = pose_with_covariance;

        timer = ros::Time::now();

        ROS_INFO("Recover: position(x: %f, y: %f, z: %f) orientation(x: %f, y: %f, z: %f, z: %f)", 
            pose.position.x, pose.position.y, pose.position.z, 
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        initialpose_pub.publish(msg);
    }

    /**
     * initialpose2の実行
     * lost_localizationの脱線状態を解除するために行う。自己位置が合った状態で通知しないとすぐ脱線判定になるので注意。
     * メッセージ内容は確認しないのでダミーデータでよい。
     */
    void Node::publishInitialpose2() {
        geometry_msgs::PoseWithCovarianceStamped msg;
        std_msgs::Header header;
        geometry_msgs::PoseWithCovariance pose_with_covariance;

        initialpose2_pub.publish(msg);
    }

    /**
     * メイン関数.
     */
    void Node::run() {
        if (mode == NDT_POSE) {
            ndt_pose_sub = nh.subscribe("/ndt_pose", 100, &RecoverLocalization::Node::ndtPoseCallback, this);
        } else {
            odom_sub = nh.subscribe("/vehicle/odom", 100, &RecoverLocalization::Node::odometryCallback, this);
        }
        ndt_stat_sub = nh.subscribe("/ndt_stat", 10, &RecoverLocalization::Node::ndtStatCallback, this);
        monitor_stat_sub = nh.subscribe("/ndt_monitor/ndt_status", 10, &RecoverLocalization::Node::monitorStatCallback, this);
        health_sub = nh.subscribe("/monitor_status", 10, &RecoverLocalization::Node::healthCallback, this);
        initialpose_sub = nh.subscribe("/initialpose", 10, &RecoverLocalization::Node::initialposeCallback, this);

        initialpose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
        initialpose2_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose2", 1000);

        ros::spin();
    }
}
