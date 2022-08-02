#include <recover_localization.h>

namespace RecoverLocalization {

    Data::Data(const autoware_msgs::NDTStat::ConstPtr& msg) {
        ndt_stat = msg;
        median_result = 0.0;
    }

    /**
     * 記録したndt_statの取得.
     */
    autoware_msgs::NDTStat::ConstPtr Data::getNdtStat() const {
        return ndt_stat;
    }

    /**
     * 記録したスコア中央値の取得.
     */
    double Data::getMedianResult() {
        return median_result;
    }

    /**
     * スコア中央値の記録.
     */
    void Data::setMedianResult(double result) {
        median_result = result;
    }
}