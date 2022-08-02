#include <waypoint_editor.h>

namespace WaypointEditor {
    /**
     * waypointごとの情報.
     */
    Data::Data(
        const int _waypoint_id,
        const double _x,
        const double _y,
        const double _z,
        const double _yaw,
        const int _sensor2d
    ) {
        waypoint_id = _waypoint_id;
        x = _x;
        y = _y;
        z = _z;
        yaw = _yaw;
        sensor2d = _sensor2d;

        stop = 0;
        split = 0;
        pause = 0;
//shutter
        shutter_point = 0;

        input_targets.clear();
        output_targets.clear();

        score = 0.0;
        filtered_score = 0.0;

        velocity = 0.0;
        detection_range_front_l = 0.0;
        detection_range_front_r = 0.0;
        detection_range_rear_l = 0.0;
        detection_range_rear_r = 0.0;
        winker_point = 0;
        attention_broadcast = 0;
    }

    /* 以下、waypoint_saver_marker記録関連 */
    int Data::getWaypointId() { return waypoint_id; }
    double Data::getX() { return x; }
    double Data::getY() { return y; }
    double Data::getZ() { return z; }
    double Data::getYaw() { return yaw; }
    void Data::setSensor2D(const int _sensor2d) { sensor2d = _sensor2d; }
    int Data::getSensor2D() { return sensor2d; }

    /* 以下、各コマンド関連 */
    void Data::setStop(const int _stop) { stop = _stop; }
    int Data::getStop() { return stop; }
//shutter
    void Data::setShutterPoint(const int _shutter_point) { shutter_point = _shutter_point; }
    int Data::getShutterPoint(){return shutter_point;}



    void Data::setInputTargets(const std::vector<std::string> _input_targets) { input_targets = _input_targets; }
    std::vector<std::string> Data::getInputTargets() { return input_targets; }
    void Data::setSplit(const int _split) { split = _split; }
    int Data::getSplit() { return split; }
    void Data::setPause(const int _pause) { pause = _pause; }
    int Data::getPause() { return pause; }
    void Data::setOutputTargets(const std::vector<carctl_msgs::output_target> _output_targets) { output_targets = _output_targets; }
    std::vector<carctl_msgs::output_target> Data::getOutputTargets() { return output_targets; }
    void Data::setHotspot(
        const double _x,
        const double _y,
        const double _z,
        const double _qx,
        const double _qy,
        const double _qz,
        const double _qw
    ) {
        hotspot.setData(_x, _y, _z, _qx, _qy, _qz, _qw);
    }
    void Data::clearHotspot() { hotspot.clear(); }
    WaypointEditor::Hotspot Data::getHotspot() { return hotspot; }

    /* 以下、カーブ判定処理関連 */
    void Data::setScore(const double _score) { score = _score; }
    double Data::getScore() { return score; }
    void Data::setFilteredScore(const double _filtered_score) { filtered_score = _filtered_score; }
    double Data::getFilteredScore() { return filtered_score; }

    void Data::setVelocity(const double _velocity) { velocity = _velocity; }
    double Data::getVelocity() { return velocity; }
    void Data::setDetectionRangeFrontL(const double _detection_range_front_l) { detection_range_front_l = _detection_range_front_l; }
    double Data::getDetectionRangeFrontL() { return detection_range_front_l; }
    void Data::setDetectionRangeFrontR(const double _detection_range_front_r) { detection_range_front_r = _detection_range_front_r; }
    double Data::getDetectionRangeFrontR() { return detection_range_front_r; }
    void Data::setDetectionRangeRearL(const double _detection_range_rear_l) { detection_range_rear_l = _detection_range_rear_l; }
    double Data::getDetectionRangeRearL() { return detection_range_rear_l; }
    void Data::setDetectionRangeRearR(const double _detection_range_rear_r) { detection_range_rear_r = _detection_range_rear_r; }
    double Data::getDetectionRangeRearR() { return detection_range_rear_r; }
    void Data::setWinkerPoint(const int _winker_point) { winker_point = _winker_point; }
    int Data::getWinkerPoint() { return winker_point; }
    void Data::setAttentionBroadcast(const int _attention_broadcast) { attention_broadcast = _attention_broadcast; }
    int Data::getAttentionBroadcast() { return attention_broadcast; }
}
