#ifndef WAYPOINT_EDITOR_CURVE_DETECTION_H
#define WAYPOINT_EDITOR_CURVE_DETECTION_H

#include <waypoint_editor_base.h>

namespace WaypointEditor {
    struct Position {
        double x;
        double y;
    };

    struct StepParameter {
        double score;
        double velocity_scale;
        double range_scale;
    };

    class CurveDetection {
        public:
            CurveDetection() {}
            void run(
                const WaypointEditor::ParamCurve::ConstPtr& curve,
                std::deque<WaypointEditor::Data::Ptr>& waypoints
            );
        private:
            void calculateScore(
                const WaypointEditor::ParamCurve::ConstPtr& curve,
                std::deque<WaypointEditor::Data::Ptr>& waypoints
            );
            void filterScore(
                const WaypointEditor::ParamCurve::ConstPtr& curve,
                std::deque<WaypointEditor::Data::Ptr>& waypoints
            );
            void detect(
                const WaypointEditor::ParamCurve::ConstPtr& curve,
                std::deque<WaypointEditor::Data::Ptr>& waypoints
            );
            void detectEnterCurve(
                const WaypointEditor::ParamCurve::ConstPtr& curve,
                std::deque<WaypointEditor::Data::Ptr>& waypoints
            );
            void detectExitCurve(
                const WaypointEditor::ParamCurve::ConstPtr& curve,
                std::deque<WaypointEditor::Data::Ptr>& waypoints
            );
            std::deque<WaypointEditor::StepParameter> createStepParameter(const WaypointEditor::ParamCurve::ConstPtr& curve);
            WaypointEditor::StepParameter findStep(
                std::deque<WaypointEditor::StepParameter>& step_parameter,
                double score
            );
            WaypointEditor::Position rotatePosition(double x, double y, double yaw);
            double median(std::deque<double>& list);

    };

    static int LEFT = 1;
    static int RIGHT = 2;
}

#endif /* WAYPOINT_EDITOR_CURVE_DETECTION_H */