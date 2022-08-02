#ifndef WAYPOINT_EDITOR_H
#define WAYPOINT_EDITOR_H

#include <waypoint_editor_base.h>
#include <waypoint_editor_param.h>
#include <waypoint_editor_hotspot.h>
#include <waypoint_editor_data.h>
#include <waypoint_editor_curve_detection.h>
#include <waypoint_editor_output_csv.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <carctl_msgs/wp_edit_begin_msg.h>
#include <carctl_msgs/wp_edit_finish_msg.h>
#include <carctl_msgs/wp_edit_stop_msg.h>
#include <carctl_msgs/wp_edit_split_msg.h>
#include <carctl_msgs/wp_edit_pause_msg.h>
#include <carctl_msgs/wp_edit_output_msg.h>
#include <carctl_msgs/wp_edit_2d_msg.h>
#include <carctl_msgs/wp_edit_hotspot_msg.h>
#include <carctl_msgs/monitor_status.h>

namespace WaypointEditor {
    class Node {
        public:
            Node(
                const WaypointEditor::ParamGeneral::ConstPtr _general,
                const WaypointEditor::ParamChangeRoute::ConstPtr _change_route,
                const WaypointEditor::ParamCurve::ConstPtr _curve
            );

            void run();
        private:
            ros::NodeHandle nh;

            WaypointEditor::ParamGeneral::ConstPtr general;
            WaypointEditor::ParamChangeRoute::ConstPtr change_route;
            WaypointEditor::ParamCurve::ConstPtr curve;

            std::deque<WaypointEditor::Data::Ptr> waypoints;

            ros::Publisher health_pub;

            ros::Subscriber marker_sub;
            ros::Subscriber begin_sub;
            ros::Subscriber finish_sub;
            ros::Subscriber stop_sub;
            ros::Subscriber split_sub;
            ros::Subscriber pause_sub;
            ros::Subscriber output_sub;
            ros::Subscriber sensor_2d_sub;
            ros::Subscriber hotspot_sub;

            ros::Subscriber current_pose_sub;

            bool isRunning;
            int sensor2d;

            void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
            void beginCallback(const carctl_msgs::wp_edit_begin_msg::ConstPtr& msg);
            void finishCallback(const carctl_msgs::wp_edit_finish_msg::ConstPtr& msg);
            void stopCallback(const carctl_msgs::wp_edit_stop_msg::ConstPtr& msg);
            void splitCallback(const carctl_msgs::wp_edit_split_msg::ConstPtr& msg);
            void pauseCallback(const carctl_msgs::wp_edit_pause_msg::ConstPtr& msg);
            void outputCallback(const carctl_msgs::wp_edit_output_msg::ConstPtr& msg);
            void sensor2dCallback(const carctl_msgs::wp_edit_2d_msg::ConstPtr& msg);
            void hotspotCallback(const carctl_msgs::wp_edit_hotspot_msg::ConstPtr& msg);
            void setHotspot(const int waypoint_id, const int status);

            geometry_msgs::PoseStamped current_pose;
            geometry_msgs::PoseStamped start_position_pose;
            int start_position_id;
            void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

            std::deque<WaypointEditor::Data::Ptr>::iterator findData(const int waypoint);
            void sendReturnMessage(const int status, const std::string messageId);
    };

    static const int16_t INFO = 0;
    static const int16_t ERROR = 1;

    static const int HOTSPOT_STATUS_RESERVE = 1;

    static const std::string BEGIN_SUCCESS = "waypoint_editor_0_0";
    static const std::string FINISH_SUCCESS = "waypoint_editor_0_1";
    static const std::string STOP_SUCCESS = "waypoint_editor_0_2";
    static const std::string SPLIT_SUCCESS = "waypoint_editor_0_3";
    static const std::string PAUSE_SUCCESS = "waypoint_editor_0_4";
    static const std::string OUTPUT_SUCCESS = "waypoint_editor_0_5";
    static const std::string SENSOR2D_SUCCESS = "waypoint_editor_0_6";
    static const std::string HOTSPOT_SUCCESS = "waypoint_editor_0_7";

    static const std::string BEFORE_BEGIN_ERROR = "waypoint_editor_1_0";
    static const std::string NOT_FOUND_WAYPOINT_ID_ERROR = "waypoint_editor_1_1";
    static const std::string SAVE_FAILURE = "waypoint_editor_1_2";
}

#endif /* WAYPOINT_EDITOR_H */