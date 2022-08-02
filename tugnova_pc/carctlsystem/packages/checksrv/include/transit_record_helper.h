#ifndef TRANSIT_RECORD_HELPER_H
#define TRANSIT_RECORD_HELPER_H

#include <memory>
#include <vector>
#include <algorithm>
#include <cstddef>

namespace CheckSrv {
    class TransitRecord {
        public:
            TransitRecord(const int waypoint_id, const int waiting_state);
            int getWaypointId();
            int getWaitingState();
            void setWaitingState(const int waiting_state);
            void recordSent();
            bool isSent();

            typedef std::shared_ptr<CheckSrv::TransitRecord> Ptr;
        private:
            int waypoint_id;
            int waiting_state;
            bool sent_flag;

    };

    class TransitRecordHelper {
        public:
            TransitRecordHelper();
            void push(const int waypoint_id, const int waiting_state);
            std::vector<int> getList();
            void clear(const int exception_waypoint_id);
            void reserveClear();
            CheckSrv::TransitRecord::Ptr getTargetWaypoint();
            void passWaitingWaypoint(const int waypoint_id);
            void forcedPassWaitingWaypoint(const int n_th_later);
        private:
            std::vector<CheckSrv::TransitRecord::Ptr> list;
            int previous_waypoint_id;
            bool isReserve;

            void forcedClear();
    };
}


#endif /* TRANSIT_RECORD_HELPER_H */
