#ifndef WAYPOINT_EDITOR_PARAM_H
#define WAYPOINT_EDITOR_PARAM_H

#include <waypoint_editor_base.h>

namespace WaypointEditor {
    class ParamGeneral {
        public:
            ParamGeneral(
                const std::string _output,
                const std::string _name,
                const double _default_obstacle_detection_distance_change,
                const double _default_minimum_target_point_distance,
                const double _default_pause_time,
                const double _default_cargo_range
            );

            std::string getOutput() const;
            std::string getName() const;
            double getDefaultObstacleDetectionDistanceChange() const;
            double getDefaultMinimumTargetPointDistance() const;
            double getDefaultPauseTime() const;
            double getDefaultCargoRange() const;

            typedef std::shared_ptr<const WaypointEditor::ParamGeneral> ConstPtr;
        private:
            std::string output;
            std::string name;
            double default_obstacle_detection_distance_change;
            double default_minimum_target_point_distance;
            double default_pause_time;
            double default_cargo_range;
    };

    class ParamChangeRoute {
        public:
            ParamChangeRoute(
                const double _waypoint_distance,
                const double _lookahead_ratio
            );

            double getWaypointDistance() const;
            double getLookaheadRatio() const;

            typedef std::shared_ptr<const WaypointEditor::ParamChangeRoute> ConstPtr;
        private:
            double waypoint_distance;
            double lookahead_ratio;
    };

    class ParamCurve {
        public:
            ParamCurve(
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
            );

            double getDefaultVelocity() const;
            double getDefaultDetectionRangeFront() const;
            double getDefaultDetectionRangeRear() const;
            int getCurveLookahead() const;
            double getMinimumVelocity() const;
            double getMinumumDetectionRange() const;
            double getEnterCurveThresholdRad() const;
            double getExitCurveThresholdRad() const;
            int getWindowSize() const;
            int getSteps() const;

            typedef std::shared_ptr<const WaypointEditor::ParamCurve> ConstPtr;
        private:
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
    };
}

#endif /* WAYPOINT_EDITOR_PARAM_H */