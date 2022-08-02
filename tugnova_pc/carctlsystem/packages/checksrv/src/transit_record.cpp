#include <transit_record_helper.h>

namespace CheckSrv {

    TransitRecord::TransitRecord(const int waypoint_id, const int waiting_state) {
        this->waypoint_id = waypoint_id;
        this->waiting_state = waiting_state;
        this->sent_flag = false;
    }

    /**
     * WaypointIDを返却.
     */
    int TransitRecord::getWaypointId() {
        return waypoint_id;
    }

    /**
     * WaitingStateを返却.
     */
    int TransitRecord::getWaitingState() {
        return waiting_state;
    }

    void TransitRecord::setWaitingState(const int waiting_state) {
        this->waiting_state = waiting_state;
    }

    /**
     * 送信済みを記録.
     */
    void TransitRecord::recordSent() {
        sent_flag = true;
    }

    /**
     * 送信済みか.
     */
    bool TransitRecord::isSent() {
        return sent_flag;
    }

}