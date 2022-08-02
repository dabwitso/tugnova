#include <waypoint_editor.h>

namespace WaypointEditor {
    /**
     * 注意
     * waypoint_saverとwaypoint_editorはノード起動から終了までの間、記録し続け出力する動きのため、
     * 経路を作る時は必ず新規でノードを立ち上げ直すこと。
     * waypoint_editorはwaypoint_saverとwaypointの情報同期を行っているので、
     * waypoint_editorだけ立ち上げたまま、waypoint_saverを再起動させるといった行為も行ってはいけない。
     */
    Node::Node(
        const WaypointEditor::ParamGeneral::ConstPtr _general,
        const WaypointEditor::ParamChangeRoute::ConstPtr _change_route,
        const WaypointEditor::ParamCurve::ConstPtr _curve
    ) {
        general = _general;
        change_route = _change_route;
        curve = _curve;

        waypoints.clear();
        isRunning = false;
        sensor2d = 1; // 2Dセンサーは1～15の範囲のため、デフォルトは1とする
        start_position_id = 0;
    }

    /**
     * waypoint_saver_markerトピック受信.
     */
    void Node::markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        // MarkerArrayは必ずsaved_waypoint_arrowとsaved_waypoint_velocityの情報が含まれるためsizeは2倍になる
        std::size_t wp_size_double = waypoints.size() * 2;
        int waypoint_id = 1;

        std::for_each((msg->markers.begin() + wp_size_double), msg->markers.end(), [&](const visualization_msgs::Marker& marker) {
            if (marker.ns == "saved_waypoint_arrow") { return; }

            waypoint_id = marker.id + 1;
            double x = marker.pose.position.x;
            double y = marker.pose.position.y;
            double z = marker.pose.position.z;
            double yaw = tf::getYaw(marker.pose.orientation);
            WaypointEditor::Data::Ptr data(new WaypointEditor::Data(waypoint_id, x, y, z, yaw, sensor2d));
            waypoints.push_back(data);
            std::cout << "Insert WaypointID: " << waypoint_id << std::endl;
        });

        if (start_position_id == 0) {
            start_position_id = waypoint_id;
            start_position_pose = current_pose;
        }
    }

    /**
     * waypoint_editor_beginトピック受信.
     */
    void Node::beginCallback(const carctl_msgs::wp_edit_begin_msg::ConstPtr& msg) {
        isRunning = true;
        sensor2d = msg->mode_2d;
        sendReturnMessage(INFO, BEGIN_SUCCESS);
    }

    /**
     * waypoint_editor_finishトピック受信.
     */
    void Node::finishCallback(const carctl_msgs::wp_edit_finish_msg::ConstPtr& msg) {
        if (!isRunning) {
            sendReturnMessage(ERROR, BEFORE_BEGIN_ERROR);
            return;
        }

        if (start_position_id == 0) {
            sendReturnMessage(ERROR, SAVE_FAILURE);
            return;
        }
        setHotspot(start_position_id, HOTSPOT_STATUS_RESERVE);

        isRunning = false;

        WaypointEditor::CurveDetection curve_detection;
        curve_detection.run(curve, waypoints);

        WaypointEditor::OutputCSV output_csv;
        if (!output_csv.run(general, change_route, curve, waypoints)) {
            sendReturnMessage(ERROR, SAVE_FAILURE);
            return;
        }

        sendReturnMessage(INFO, FINISH_SUCCESS);
    }

    /**
     * waypoint_editor_stopトピック受信.
     */
    void Node::stopCallback(const carctl_msgs::wp_edit_stop_msg::ConstPtr& msg) {
        if (!isRunning) {
            sendReturnMessage(ERROR, BEFORE_BEGIN_ERROR);
            return;
        }

        std::deque<WaypointEditor::Data::Ptr>::iterator iter = findData(msg->waypoint);
        if (iter == waypoints.end()) {
            sendReturnMessage(ERROR, NOT_FOUND_WAYPOINT_ID_ERROR);
            return;
        }

        WaypointEditor::Data::Ptr target = *iter;
        target->setStop(msg->status);
        target->setInputTargets(msg->input_targets);

        sendReturnMessage(INFO, STOP_SUCCESS);
    }

    /**
     * waypoint_editor_splitトピック受信.
     */
    void Node::splitCallback(const carctl_msgs::wp_edit_split_msg::ConstPtr& msg) {
        if (!isRunning) {
            sendReturnMessage(ERROR, BEFORE_BEGIN_ERROR);
            return;
        }

        std::deque<WaypointEditor::Data::Ptr>::iterator iter = findData(msg->waypoint);
        if (iter == waypoints.end()) {
            sendReturnMessage(ERROR, NOT_FOUND_WAYPOINT_ID_ERROR);
            return;
        }

        WaypointEditor::Data::Ptr target = *iter;
        target->setSplit(msg->status);

        sendReturnMessage(INFO, SPLIT_SUCCESS);
    }

    /**
     * waypoint_editor_pauseトピック受信.
     */
    void Node::pauseCallback(const carctl_msgs::wp_edit_pause_msg::ConstPtr& msg) {
        if (!isRunning) {
            sendReturnMessage(ERROR, BEFORE_BEGIN_ERROR);
            return;
        }

        std::deque<WaypointEditor::Data::Ptr>::iterator iter = findData(msg->waypoint);
        if (iter == waypoints.end()) {
            sendReturnMessage(ERROR, NOT_FOUND_WAYPOINT_ID_ERROR);
            return;
        }

        WaypointEditor::Data::Ptr target = *iter;
        target->setPause(msg->status);

        sendReturnMessage(INFO, PAUSE_SUCCESS);
    }

    /**
     * waypoint_editor_outputトピック受信.
     */
    void Node::outputCallback(const carctl_msgs::wp_edit_output_msg::ConstPtr& msg) {
        if (!isRunning) {
            sendReturnMessage(ERROR, BEFORE_BEGIN_ERROR);
            return;
        }

        std::deque<WaypointEditor::Data::Ptr>::iterator iter = findData(msg->waypoint);
        if (iter == waypoints.end()) {
            sendReturnMessage(ERROR, NOT_FOUND_WAYPOINT_ID_ERROR);
            return;
        }

        WaypointEditor::Data::Ptr target = *iter;
        target->setOutputTargets(msg->output_targets);

        sendReturnMessage(INFO, OUTPUT_SUCCESS);
    }

    /**
     * waypoint_editor_2dトピック受信.
     */
    void Node::sensor2dCallback(const carctl_msgs::wp_edit_2d_msg::ConstPtr& msg) {
        if (!isRunning) {
            sendReturnMessage(ERROR, BEFORE_BEGIN_ERROR);
            return;
        }

        std::deque<WaypointEditor::Data::Ptr>::iterator iter = findData(msg->waypoint);
        if (iter == waypoints.end()) {
            sendReturnMessage(ERROR, NOT_FOUND_WAYPOINT_ID_ERROR);
            return;
        }

        sensor2d = msg->mode; // 通知が来たwaypoint以降は全て同じセンサーモードで登録を行う
        std::for_each(iter, waypoints.end(), [&msg](WaypointEditor::Data::Ptr data){
            data->setSensor2D(msg->mode);
        });

        sendReturnMessage(INFO, SENSOR2D_SUCCESS);
    }

    /**
     * waypoint_editor_hotspotトピック受信.
     */
    void Node::hotspotCallback(const carctl_msgs::wp_edit_hotspot_msg::ConstPtr& msg) {
        if (!isRunning) {
            sendReturnMessage(ERROR, BEFORE_BEGIN_ERROR);
            return;
        }

        setHotspot(msg->waypoint, msg->status);
    }

    void Node::setHotspot(const int waypoint_id, const int status) {
        std::deque<WaypointEditor::Data::Ptr>::iterator iter = findData(waypoint_id);
        if (iter == waypoints.end()) {
            sendReturnMessage(ERROR, NOT_FOUND_WAYPOINT_ID_ERROR);
            return;
        }

        geometry_msgs::PoseStamped pose;
        if (waypoint_id == start_position_id) {
            pose = start_position_pose;
        } else {
            pose = current_pose;
        }

        WaypointEditor::Data::Ptr target = *iter;
        if (status == HOTSPOT_STATUS_RESERVE) {
            target->setHotspot(
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            );
        } else {
            target->clearHotspot();
        }

        sendReturnMessage(INFO, HOTSPOT_SUCCESS);
    }

    /**
     * current_poseトピックの受信.
     */
    void Node::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose = *msg;
    }

    /**
     * 指定Waypointの情報を取得.
     */
    std::deque<WaypointEditor::Data::Ptr>::iterator Node::findData(const int waypoint) {
        return std::find_if(waypoints.begin(), waypoints.end(), [&waypoint](WaypointEditor::Data::Ptr data) {
            return data->getWaypointId() == waypoint;
        });
    }

    /**
     * 返信メッセージを送信.
     */
    void Node::sendReturnMessage(const int status, const std::string messageId) {
        if (status == ERROR) {
            std::cout << "[ERROR] " << messageId << std::endl;
        } else {
            std::cout << "Receiving Command: " << messageId << std::endl;
        }

        carctl_msgs::monitor_status return_msg;
        return_msg.service_name = "waypoint_editor";
        return_msg.status = status;
        return_msg.error_msg = messageId;
        health_pub.publish(return_msg);
    }

    /**
     * メイン関数.
     */
    void Node::run() {
        health_pub = nh.advertise<carctl_msgs::monitor_status>("/monitor_status", 10);

        marker_sub = nh.subscribe("/waypoint_saver_marker", 10, &WaypointEditor::Node::markerCallback, this);
        begin_sub = nh.subscribe("/waypoint_editor_begin", 10, &WaypointEditor::Node::beginCallback, this);
        finish_sub = nh.subscribe("/waypoint_editor_finish", 10, &WaypointEditor::Node::finishCallback, this);
        stop_sub = nh.subscribe("/waypoint_editor_stop", 10, &WaypointEditor::Node::stopCallback, this);
        split_sub = nh.subscribe("/waypoint_editor_split", 10, &WaypointEditor::Node::splitCallback, this);
        pause_sub = nh.subscribe("/waypoint_editor_pause", 10, &WaypointEditor::Node::pauseCallback, this);
        output_sub = nh.subscribe("/waypoint_editor_output", 10, &WaypointEditor::Node::outputCallback, this);
        sensor_2d_sub = nh.subscribe("/waypoint_editor_2d", 10, &WaypointEditor::Node::sensor2dCallback, this);
        hotspot_sub = nh.subscribe("/waypoint_editor_hotspot", 10, &WaypointEditor::Node::hotspotCallback, this);

        current_pose_sub = nh.subscribe("/current_pose", 10, &WaypointEditor::Node::currentPoseCallback, this);

        ros::spin();
    }
}