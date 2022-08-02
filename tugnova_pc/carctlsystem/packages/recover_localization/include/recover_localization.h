#ifndef RECOVER_LOCALIZATION_H
#define RECOVER_LOCALIZATION_H

#include <ros/ros.h>
#include <cmath>
#include <memory>
#include <queue>

#include <nav_msgs/Odometry.h>
#include <autoware_msgs/NDTStat.h>
#include <std_msgs/String.h>
#include <carctl_msgs/monitor_status.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace RecoverLocalization {
    class Data {
        public:
            Data(const autoware_msgs::NDTStat::ConstPtr& msg);
            autoware_msgs::NDTStat::ConstPtr getNdtStat() const;
            double getMedianResult();
            void setMedianResult(double result);

            typedef std::shared_ptr<RecoverLocalization::Data> Ptr;
        private:
            autoware_msgs::NDTStat::ConstPtr ndt_stat;
            double median_result;
    };

    class Statistics {
        public:
            void accumulate(double number);
            std::size_t size();
            void clear();
            double median();
            double stdev();
        private:
            std::deque<double> list;

            double average();
            double variance(double ave);
    };

    class Node {
        public:
            Node(int _buffer_size, int _window_size, double _threshold, std::string _mode);
            void run();
        private:
            ros::NodeHandle nh;

            ros::Subscriber odom_sub;
            ros::Subscriber ndt_pose_sub;
            ros::Subscriber ndt_stat_sub;
            ros::Subscriber monitor_stat_sub;
            ros::Subscriber health_sub;
            ros::Subscriber initialpose_sub;

            ros::Publisher initialpose_pub;
            ros::Publisher initialpose2_pub;

            int buffer_size;
            int window_size;
            double threshold;
            std::string mode;

            bool is_locked;
            bool is_other_initialpose;
            ros::Time timer;

            std::deque<nav_msgs::Odometry::ConstPtr> odometry_list;
            std::deque<geometry_msgs::PoseStamped::ConstPtr> ndt_pose_list;
            std::deque<RecoverLocalization::Data::Ptr> ndt_stat_list;

            void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
            void ndtPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
            void ndtStatCallback(const autoware_msgs::NDTStat::ConstPtr& msg);
            void monitorStatCallback(const std_msgs::String msg);
            void healthCallback(const carctl_msgs::monitor_status& msg);
            void initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

            void initialize();
            void recover();
            void medianFilter();
            autoware_msgs::NDTStat::ConstPtr search();
            geometry_msgs::Pose getPose(autoware_msgs::NDTStat::ConstPtr& ndt_stat);
            nav_msgs::Odometry::ConstPtr getOdometry(autoware_msgs::NDTStat::ConstPtr& ndt_stat);
            geometry_msgs::PoseStamped::ConstPtr getNdtPose(autoware_msgs::NDTStat::ConstPtr& ndt_stat);
            void publishInitialpose(geometry_msgs::Pose& pose);
            void publishInitialpose2();
    };

    static const std::string NDT_POSE = "ndt_pose";

    static const std::string NDT_OK = "NDT_OK";
    static const std::string NDT_WARNING = "NDT_WARNING";
    static const std::string NDT_ERROR = "NDT_ERROR";
    static const std::string NDT_FATAL = "NDT_FATAL";

    static const std::string LOST_LOCALIZATION = "lost_localization";
    static const int16_t ERROR = 1;
    static const std::string MAIGO = "lost_localization_1_0";
}

#endif /* RECOVER_LOCALIZATION_H */
