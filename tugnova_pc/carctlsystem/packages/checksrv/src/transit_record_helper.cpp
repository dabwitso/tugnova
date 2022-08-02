#include <transit_record_helper.h>
#include <ros/ros.h>

namespace CheckSrv {

    TransitRecordHelper::TransitRecordHelper() {
        forcedClear();
    }

    /**
     * 通過したWaypointIDを記録.
     * Waypointが飛んだ場合は、補完処理も行う.
     */
    void TransitRecordHelper::push(const int waypoint_id, const int waiting_state) {
        // initialpose発生後のsafety_waypoints受信に合わせてリストをクリアする.(こうしないとクリアできないタイミングがあるため)
        if (isReserve) { 
            std::cout << "isReserve" << std::endl;
            forcedClear();
        }

        // 主に2週目になった時や手動運転によるバッグ操作が行われた時に強制クリアされる.
        if (previous_waypoint_id > waypoint_id) {
            std::cout << "previous_waypoint_id > waypoint_id" << std::endl;
            forcedClear();
        } else if (previous_waypoint_id == waypoint_id) {
            return; // 重複防止
        }

        // 経路途中からで初期状態になった時でも、不要な確認を防止するため、リスト作成は開始地点からにする。
        if (list.empty()) { previous_waypoint_id = waypoint_id - 1; }

        for (int complement_waypoint_id = previous_waypoint_id + 1; complement_waypoint_id <= waypoint_id; complement_waypoint_id++) {
            int saved_waiting_sate = 0; // ショートカットで読み飛ばされた箇所は、結果として走行しているので、全て停止ではないと判定する
            if (complement_waypoint_id == waypoint_id) { saved_waiting_sate = waiting_state; }
            CheckSrv::TransitRecord::Ptr record(new CheckSrv::TransitRecord(complement_waypoint_id, saved_waiting_sate));
            list.push_back(record);
        }
        previous_waypoint_id = waypoint_id;
    }

    /**
     * 上位へ送信するWaypointIDの一覧を取得.
     */
    std::vector<int> TransitRecordHelper::getList() {
        std::vector<int> result;
        std::for_each(list.begin(), list.end(), [&](CheckSrv::TransitRecord::Ptr& record) {
            result.push_back(record->getWaypointId());
            record->recordSent();
        });
        return result;
    }

    /**
     * 上位へ送信済みのWaypointIDの記録のみクリアする.
     * なお、走行位置での信号機制御のためにも走行開始後は空配列が出来ないように最後に通過したWaypointは保持する.
     */
    void TransitRecordHelper::clear(const int exception_waypoint_id) {
        std::cout << "exception_waypoint_id " << exception_waypoint_id << std::endl;
        std::vector<CheckSrv::TransitRecord::Ptr>::const_iterator newEnd = 
        std::remove_if(list.begin(), list.end(), [&](CheckSrv::TransitRecord::Ptr& record) {
            // 停止位置での再送信や、意図しない空配列の送信を防止するため、あえてクリアから除外したいWaypointを指定できるようにする
            return record->isSent() && (exception_waypoint_id != record->getWaypointId()) && (previous_waypoint_id != record->getWaypointId());
        });
        list.erase(newEnd, list.end());
    }

    /**
     * リストの処理化予約.
     */
    void TransitRecordHelper::reserveClear() {
        isReserve = true;
    }

    /**
     * 強制的に全てのWaypointIDの記録をクリアする.
     */
    void TransitRecordHelper::forcedClear() {
        previous_waypoint_id = 0;
        isReserve = false;
        list.clear();
    }

    /**
     * 車両が参照しなければならないWaypointIDを返却する.
     * 具体的には、リストの末尾から見て、最初の停止位置を参照し、停止位置がリスト内に無ければ末尾を参照する.
     */
    CheckSrv::TransitRecord::Ptr TransitRecordHelper::getTargetWaypoint() {
        if (list.empty()) { return NULL; }

        std::vector<CheckSrv::TransitRecord::Ptr>::const_reverse_iterator target =
        std::find_if(list.rbegin(), list.rend(), [](CheckSrv::TransitRecord::Ptr& record) {
            return (record->getWaitingState() != 0);
        });

        if (target == list.rend()) {
            return *(list.rbegin());
        } else {
            return *target;
        }
    }

    /**
     * 通過した停止位置を記録する.
     * ※pass系関数とclear系関数の違いについて。clear系は「上位との通信が完了した後」に行う行為で、pass系は「上位は関係なく通過した事だけ記録したい」時に行う。
     */
    void TransitRecordHelper::passWaitingWaypoint(const int waypoint_id) {
        std::cout << "passWaitingWaypoint " << waypoint_id << std::endl;
        std::vector<CheckSrv::TransitRecord::Ptr>::const_reverse_iterator target =
        std::find_if(list.rbegin(), list.rend(), [&waypoint_id](CheckSrv::TransitRecord::Ptr& record) {
            return (record->getWaypointId() == waypoint_id);
        });

        if (target == list.rend()) { return; }

        CheckSrv::TransitRecord::Ptr record = *target;
        record->setWaitingState(0);
    }

    /**
     * 現在位置から見て、閾値以上、通過した場合、強制的に通過したと見なす.
     */
    void TransitRecordHelper::forcedPassWaitingWaypoint(const int n_th_later) {
        std::cout << "forcedPassWaitingWaypoint " << n_th_later << std::endl;
        if (list.empty()) { return; }

        std::vector<CheckSrv::TransitRecord::Ptr>::const_reverse_iterator tail = list.rbegin();
        CheckSrv::TransitRecord::Ptr tailData = *tail;
        int target_waypoint_id = tailData->getWaypointId() - n_th_later;

        std::for_each(list.rbegin(), list.rend(), [&target_waypoint_id](CheckSrv::TransitRecord::Ptr& record) {
            if (target_waypoint_id > record->getWaypointId()) { record->setWaitingState(0); }
        });
    }
}
