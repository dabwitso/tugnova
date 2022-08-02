#include <waypoint_editor.h>

/**
 * 一般パラメータ(waypoint_editor.*)の取得.
 */
WaypointEditor::ParamGeneral::ConstPtr createParamGeneral() {
    ros::NodeHandle private_nh("~");

    std::string output;
    std::string name;
    double default_obstacle_detection_distance_change;
    double default_minimum_target_point_distance;
    double default_pause_time;
    double default_cargo_range;

    private_nh.param<std::string>("output", output, "/home/nvidia/Autoware/work");
    private_nh.param<std::string>("name", name, "ROUTENAME");
    private_nh.param<double>("default_obstacle_detection_distance_change", default_obstacle_detection_distance_change, 5.0);
    private_nh.param<double>("default_minimum_target_point_distance", default_minimum_target_point_distance, 1.5);
    private_nh.param<double>("default_pause_time", default_pause_time, 10.0);
    private_nh.param<double>("default_cargo_range", default_cargo_range, 1.0);

    WaypointEditor::ParamGeneral::ConstPtr param(new WaypointEditor::ParamGeneral(
        output,
        name,
        default_obstacle_detection_distance_change,
        default_minimum_target_point_distance,
        default_pause_time,
        default_cargo_range
    ));

    return param;
}

/**
 * 経路分割系パラメータ(waypoint_editor_change_route.*)の取得.
 */
WaypointEditor::ParamChangeRoute::ConstPtr createParamChangeRoute() {
    ros::NodeHandle private_nh("~");

    double waypoint_distance;
    double lookahead_ratio;

    private_nh.param<double>("waypoint_distance", waypoint_distance, 0.5);
    private_nh.param<double>("lookahead_ratio", lookahead_ratio, 2.0);

    WaypointEditor::ParamChangeRoute::ConstPtr param(new WaypointEditor::ParamChangeRoute(
        waypoint_distance,
        lookahead_ratio
    ));

    return param;
}

/**
 * カーブ判定系パラメータ(waypoint_editor_curve.*)の取得.
 */
WaypointEditor::ParamCurve::ConstPtr createParamCurve() {
    ros::NodeHandle private_nh("~");

    double default_velocity;
    double default_detection_range_front;
    double default_detection_range_rear;
    int curve_lookahead;
    double minimum_velocity;
    double minumum_detection_range;
    double enter_curve_threshold_rad;
    double exit_curve_threshold_rad;
    int window_size;
    int steps;

    private_nh.param<double>("default_velocity", default_velocity, 4.5);
    private_nh.param<double>("default_detection_range_front", default_detection_range_front, 1.2);
    private_nh.param<double>("default_detection_range_rear", default_detection_range_rear, 1.2);
    private_nh.param<int>("curve_lookahead", curve_lookahead, 10);
    private_nh.param<double>("minimum_velocity", minimum_velocity, 2.7);
    private_nh.param<double>("minumum_detection_range", minumum_detection_range, 0.36);
    private_nh.param<double>("enter_curve_threshold_rad", enter_curve_threshold_rad, 0.35);
    private_nh.param<double>("exit_curve_threshold_rad", exit_curve_threshold_rad, 0.05);
    private_nh.param<int>("window_size", window_size, 5);
    private_nh.param<int>("steps", steps, 3);

    WaypointEditor::ParamCurve::ConstPtr param(new WaypointEditor::ParamCurve(
        default_velocity,
        default_detection_range_front,
        default_detection_range_rear,
        curve_lookahead,
        minimum_velocity,
        minumum_detection_range,
        enter_curve_threshold_rad,
        exit_curve_threshold_rad,
        window_size,
        steps
    ));

    return param;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "waypoint_editor");

    WaypointEditor::Node waypoint_editor(
        createParamGeneral(),
        createParamChangeRoute(),
        createParamCurve()
    );
    waypoint_editor.run();

    return 0;
}