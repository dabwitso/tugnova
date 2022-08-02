#include <waypoint_editor.h>

namespace WaypointEditor {
    /**
     * カーブ判定処理の開始.
     */
    void CurveDetection::run(
        const WaypointEditor::ParamCurve::ConstPtr& curve,
        std::deque<WaypointEditor::Data::Ptr>& waypoints
    ) {
        calculateScore(curve, waypoints);
        filterScore(curve, waypoints);
        detect(curve, waypoints);

        std::for_each(waypoints.begin(), waypoints.end(), [](WaypointEditor::Data::Ptr data) {
            std::cout << 
                "ID: " << data->getWaypointId() << 
                " Score: " << data->getScore() <<
                " FilteredScore: " << data->getFilteredScore() <<
                " Winker: " << data->getWinkerPoint() << 
                " Velocity: " << data->getVelocity() <<
                " R: " << data->getDetectionRangeFrontR() <<
                " L: " << data->getDetectionRangeFrontL() << std::endl;
        });
    }

    /**
     * スコアの算出.
     */
    void CurveDetection::calculateScore(
        const WaypointEditor::ParamCurve::ConstPtr& curve,
        std::deque<WaypointEditor::Data::Ptr>& waypoints
    ) {
        std::size_t waypoints_size = waypoints.size();
        int count = 0;
        int lookahead = 0;

        while (count < waypoints_size) {
            if (count + curve->getCurveLookahead() < waypoints_size) {
                lookahead = count + curve->getCurveLookahead();
            } else {
                lookahead = waypoints_size - 1;
            }

            WaypointEditor::Data::Ptr first_waypoint = waypoints[count];
            WaypointEditor::Data::Ptr last_waypoint = waypoints[lookahead];
            count++;

            WaypointEditor::Position local_position = rotatePosition(
                last_waypoint->getX() - first_waypoint->getX(),
                last_waypoint->getY() - first_waypoint->getY(),
                first_waypoint->getYaw()
            );

            double score = 0.0;
            if (local_position.x != 0.0) { score = std::atan(local_position.y / local_position.x); }
            first_waypoint->setScore(score);
        }
    }

    /**
     * フィルタ後のスコア算出.
     */
    void CurveDetection::filterScore(
        const WaypointEditor::ParamCurve::ConstPtr& curve,
        std::deque<WaypointEditor::Data::Ptr>& waypoints
    ) {
        std::deque<double> score_list;
        std::size_t waypoints_size = waypoints.size();
        int count = 0;

        while (count < waypoints_size) {
            int end = curve->getWindowSize() + count;
            if (waypoints_size - count < curve->getWindowSize()) {
                end = waypoints_size;
            }

            score_list.clear();
            std::for_each(waypoints.begin() + count, waypoints.begin() + end, [&curve, &score_list](WaypointEditor::Data::Ptr& data) {
                score_list.push_back(data->getScore());
            });

            waypoints[count]->setFilteredScore(median(score_list));
            count++;
        }
    }

    /**
     * カーブを検知.
     */
    void CurveDetection::detect(
        const WaypointEditor::ParamCurve::ConstPtr& curve,
        std::deque<WaypointEditor::Data::Ptr>& waypoints
    ) {
        std::deque<WaypointEditor::StepParameter> step_parameter = createStepParameter(curve);

        detectEnterCurve(curve, waypoints);
        detectExitCurve(curve, waypoints);

        std::for_each(waypoints.begin(), waypoints.end(), [&](WaypointEditor::Data::Ptr& data) {
            if (data->getWinkerPoint() == RIGHT || data->getWinkerPoint() == LEFT) {
                WaypointEditor::StepParameter parameter = findStep(step_parameter, data->getScore());

                data->setVelocity(curve->getDefaultVelocity() * parameter.velocity_scale);
                double short_range_front = curve->getDefaultDetectionRangeFront() * parameter.range_scale;
                double long_range_front = 2 * curve->getDefaultDetectionRangeFront() - short_range_front;
                double short_range_rear = curve->getDefaultDetectionRangeRear() * parameter.range_scale;
                double long_range_rear = 2 * curve->getDefaultDetectionRangeRear() - short_range_rear;

                if (data->getWinkerPoint() == RIGHT) {
                    data->setDetectionRangeFrontR(long_range_front);
                    data->setDetectionRangeFrontL(short_range_front);
                    data->setDetectionRangeRearR(long_range_rear);
                    data->setDetectionRangeRearL(short_range_rear);
                } else {
                    data->setDetectionRangeFrontR(short_range_front);
                    data->setDetectionRangeFrontL(long_range_front);
                    data->setDetectionRangeRearR(short_range_rear);
                    data->setDetectionRangeRearL(long_range_rear);
                }
            } else {
                data->setVelocity(curve->getDefaultVelocity());
                data->setDetectionRangeFrontR(curve->getDefaultDetectionRangeFront());
                data->setDetectionRangeFrontL(curve->getDefaultDetectionRangeFront());
                data->setDetectionRangeRearR(curve->getDefaultDetectionRangeRear());
                data->setDetectionRangeRearL(curve->getDefaultDetectionRangeRear());
            }
        });
    }

    /**
     * カーブ進入を検知.
     */
    void CurveDetection::detectEnterCurve(
        const WaypointEditor::ParamCurve::ConstPtr& curve,
        std::deque<WaypointEditor::Data::Ptr>& waypoints
    ) {
        std::for_each(waypoints.begin(), waypoints.end(), [&curve](WaypointEditor::Data::Ptr& data) {
            if (data->getFilteredScore() <= -curve->getEnterCurveThresholdRad()) {
                data->setWinkerPoint(RIGHT);
            } else if (data->getFilteredScore() >= curve->getEnterCurveThresholdRad()) {
                data->setWinkerPoint(LEFT);
            }
        });
    }

    /**
     * カーブ脱出を検知.
     */
    void CurveDetection::detectExitCurve(
        const WaypointEditor::ParamCurve::ConstPtr& curve,
        std::deque<WaypointEditor::Data::Ptr>& waypoints
    ) {
        // 1番目のWaypointに対しては前後比較が出来ないので無視する
        int count = 1;
        while (count < waypoints.size()) {
            WaypointEditor::Data::Ptr previous = waypoints[count - 1];
            WaypointEditor::Data::Ptr current = waypoints[count];
            count++;

            if (current->getWinkerPoint() != 0 || previous->getWinkerPoint() == 0) {
                continue;
            }

            if (std::abs(current->getFilteredScore()) >= curve->getExitCurveThresholdRad()) {
                current->setWinkerPoint(previous->getWinkerPoint());
            }
        }
    }

    /**
     * パラメータ作成.
     */
    std::deque<WaypointEditor::StepParameter> CurveDetection::createStepParameter(
        const WaypointEditor::ParamCurve::ConstPtr& curve
    ) {
        std::deque<double> scale_list;
        double scale_diff = 1.0 / (static_cast<double>(curve->getSteps()) - 1.0);
        double total = 1.0 - scale_diff;
        for (int i = 0; i < curve->getSteps(); i++) {
            total += scale_diff;
            scale_list.push_back(total);
        }

        std::deque<WaypointEditor::StepParameter> step_parameter;
        std::size_t size = scale_list.size();
        int count = 1;

        WaypointEditor::StepParameter parameter;
        parameter.score = 0.0;
        parameter.velocity_scale = 1.0;
        parameter.range_scale = 1.0;
        step_parameter.push_back(parameter);

        std::for_each(scale_list.begin(), scale_list.end(), [&curve, &step_parameter, &count, &size](double scale) {
            WaypointEditor::StepParameter parameter;
            parameter.score = curve->getEnterCurveThresholdRad() * scale;
            parameter.velocity_scale = 1.0 - ((1.0 - (curve->getMinimumVelocity() / curve->getDefaultVelocity())) / size) * count;
            parameter.range_scale = 1.0 - ((1.0 - (curve->getMinumumDetectionRange() / curve->getDefaultDetectionRangeFront())) / size) * count;
            step_parameter.push_back(parameter);
            count++;
        });

        return step_parameter;
    }

    /**
     * スコアから値変動のパラメータを選択.
     */
    WaypointEditor::StepParameter CurveDetection::findStep(
        std::deque<WaypointEditor::StepParameter>& step_parameter, 
        double filtered_score
    ) {
        int count = 0;
        int result = 0;
        std::for_each(step_parameter.begin(), step_parameter.end(), [&filtered_score, &count, &result](WaypointEditor::StepParameter& parameter) {
            if (std::abs(filtered_score) > parameter.score) { result = count; }
            count++;
        });

        return step_parameter[result];
    }

    /**
     * ローカル座標に合わせて回転.
     */
    WaypointEditor::Position CurveDetection::rotatePosition(
        double x, double y, double yaw
    ) {
        WaypointEditor::Position position;

        double reverse_yaw = -yaw;
        position.x = x * cos(reverse_yaw) - y * sin(reverse_yaw);
        position.y = x * sin(reverse_yaw) + y * cos(reverse_yaw);

        return position;
    }

    /**
     * 中央値計算.
     */
    double CurveDetection::median(std::deque<double>& list) {
        std::size_t size = list.size();
        std::sort(list.begin(), list.end());

        if (size % 2 == 0) {
            return (list[size / 2] + list[(size / 2) - 1]) / 2;
        } else {
            return list[size / 2];
        }
    }
}