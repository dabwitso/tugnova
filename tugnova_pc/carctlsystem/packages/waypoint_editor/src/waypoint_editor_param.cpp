#include <waypoint_editor.h>

namespace WaypointEditor {
    /**
     * 以降、一般パラメータ(waypoint_editor.*)関連
     */
    ParamGeneral::ParamGeneral(
        const std::string _output,
        const std::string _name,
        const double _default_obstacle_detection_distance_change,
        const double _default_minimum_target_point_distance,
        const double _default_pause_time,
        const double _default_cargo_range
    ) {
        output = _output;
        name = _name;
        default_obstacle_detection_distance_change = _default_obstacle_detection_distance_change;
        default_minimum_target_point_distance = _default_minimum_target_point_distance;
        default_pause_time = _default_pause_time;
        default_cargo_range = _default_cargo_range;
    }

    std::string ParamGeneral::getOutput() const { return output; }
    std::string ParamGeneral::getName() const { return name; }
    double ParamGeneral::getDefaultObstacleDetectionDistanceChange() const { return default_obstacle_detection_distance_change; }
    double ParamGeneral::getDefaultMinimumTargetPointDistance() const { return default_minimum_target_point_distance; }
    double ParamGeneral::getDefaultPauseTime() const { return default_pause_time; }
    double ParamGeneral::getDefaultCargoRange() const { return default_cargo_range; }

    /**
     * 以降、カーブ判定系パラメータ(waypoint_editor_curve.*)関連
     */
    ParamChangeRoute::ParamChangeRoute(
        const double _waypoint_distance,
        const double _lookahead_ratio
    ) {
        waypoint_distance = _waypoint_distance;
        lookahead_ratio = _lookahead_ratio;
    }

    double ParamChangeRoute::getWaypointDistance() const { return waypoint_distance; }
    double ParamChangeRoute::getLookaheadRatio() const { return lookahead_ratio; }

    /**
     * 以降、経路分割系パラメータ(waypoint_editor_change_route.*)関連
     */
    ParamCurve::ParamCurve(
        const double _default_velocity,
        const double _default_detection_range_front,
        const double _default_detection_range_rear,
        const int _curve_lookahead,
        const double _minimum_velocity,
        const double _minumum_detection_range,
        const double _enter_curve_threshold_rad,
        const double _exit_curve_threshold_rad,
        const int _window_size,
        const int _steps
    ) {
        default_velocity = _default_velocity;
        default_detection_range_front = _default_detection_range_front;
        default_detection_range_rear = _default_detection_range_rear;
        curve_lookahead = _curve_lookahead;
        minimum_velocity = _minimum_velocity;
        minumum_detection_range = _minumum_detection_range;
        enter_curve_threshold_rad = _enter_curve_threshold_rad;
        exit_curve_threshold_rad = _exit_curve_threshold_rad;
        window_size = _window_size;
        steps = _steps;
    }

    double ParamCurve::getDefaultVelocity() const { return default_velocity; }
    double ParamCurve::getDefaultDetectionRangeFront() const { return default_detection_range_front; }
    double ParamCurve::getDefaultDetectionRangeRear() const { return default_detection_range_rear; }
    int ParamCurve::getCurveLookahead() const { return curve_lookahead; }
    double ParamCurve::getMinimumVelocity() const { return minimum_velocity; }
    double ParamCurve::getMinumumDetectionRange() const { return minumum_detection_range; }
    double ParamCurve::getEnterCurveThresholdRad() const { return enter_curve_threshold_rad; }
    double ParamCurve::getExitCurveThresholdRad() const { return exit_curve_threshold_rad; }
    int ParamCurve::getWindowSize() const { return window_size; }
    int ParamCurve::getSteps() const { return steps; }
}