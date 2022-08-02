#ifndef WAYPOINT_EDITOR_OUTPUT_CSV_H
#define WAYPOINT_EDITOR_OUTPUT_CSV_H

#include <waypoint_editor_base.h>

namespace WaypointEditor {
    class OutputCSV {
        public:
            OutputCSV() {}
            bool run(
                const WaypointEditor::ParamGeneral::ConstPtr& general,
                const WaypointEditor::ParamChangeRoute::ConstPtr& change_route,
                const WaypointEditor::ParamCurve::ConstPtr& curve,
                std::deque<WaypointEditor::Data::Ptr>& waypoints
            );

        private:
            std::deque<std::string> route_csv_header = {
                "waypoint_id",
                "x",
                "y",
                "z",
                "yaw",
                "velocity",
                "change_flag",
                "shutter_point",
                "shutter_stop",
                "shutter_passed",
                "steering_flag",
                "accel_flag",
                "stop_flag",
                "event_flag",
                "traffic_light_detection",
                "target_traffic_light_camera",
                "start",
                "goal",
                "stop_point",
                "detection_range_front_l",
                "detection_range_front_r",
                "detection_range_rear_l",
                "detection_range_rear_r",
                "fatal_area_left",
                "fatal_area_right",
                "cargo_left",
                "cargo_right",
                "obstacle_detection_distance_change",
                "winker_point",
                "obstacle_sensor_range",
                "mode_change",
                "attention_broadcast",
                "inter_vehicular_distance",
                "minimum_target_point_distance",
                "target_point_ratio",
                "voxel_leaf_size",
                "local_max_slope",
                "preliminary_output",
                "route_change",
                "pause_point",
                "pause_time",
                "cross_point",
                "cross_point_detection_area_left",
                "cross_point_detection_area_right",
                "cross_point_detection_area_front",
                "cross_point_detection_area_back"
            };
            std::deque<std::string> config_csv_header = {
                "waypoint_id",
                "location_name",
                "input_target_equip1",
                "input_target_equip2",
                "input_target_equip3",
                "output_target_equip1",
                "output_value1",
                "output_target_equip2",
                "output_value2",
                "output_target_equip3",
                "output_value3"
            };

            int getSplitCount(std::deque<WaypointEditor::Data::Ptr>& waypoints);
            int getExtensionWaypointCount(const double velocity, const WaypointEditor::ParamChangeRoute::ConstPtr& change_route);
            void outputRouteCSVHeader(std::ofstream& route_fs, WaypointEditor::Data::Ptr& data);
            void outputConfigCSVHeader(std::ofstream& config_fs);
            void outputRouteData(
                std::ofstream& route_fs,
                const WaypointEditor::ParamGeneral::ConstPtr& general,
                const WaypointEditor::ParamChangeRoute::ConstPtr& change_route,
                WaypointEditor::Data::Ptr& data
            );
            void outputConfigData(
                std::ofstream& config_fs,
                WaypointEditor::Data::Ptr& data
            );
            std::string outputHotspot(const std::string path, const int hotspots, WaypointEditor::Hotspot& hotspot);
            void outputIndex(const std::string route_path, const std::string name, std::string index_text);
            std::string getInputTarget(std::vector<std::string> input_targets, int offset);
            carctl_msgs::output_target getOutputTarget(std::vector<carctl_msgs::output_target> output_targets, int offset);
    };

    static const std::string LOCALTION_NAME = "位置名なし";
    static const std::string NULL_STR = "NULL";
    static const int SPLIT_STATUS_TAIL = 2;
}

#endif /* WAYPOINT_EDITOR_OUTPUT_CSV_H */
