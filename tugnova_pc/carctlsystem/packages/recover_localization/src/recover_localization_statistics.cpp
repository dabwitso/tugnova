#include <recover_localization.h>

namespace RecoverLocalization {

    /**
     * 計算対象の数値情報を蓄積.
     */
    void Statistics::accumulate(double number) {
        list.push_back(number);
    } 

    /**
     * 蓄積情報数.
     */
    std::size_t Statistics::size() {
        return list.size();
    }

    /**
     * 蓄積情報の破棄.
     */
    void Statistics::clear() {
        list.clear();
    }

    /**
     * 中央値計算.
     */
    double Statistics::median() {
        std::size_t size = list.size();
        std::sort(list.begin(), list.end());

        if (size % 2 == 0) {
            return (list[size / 2] + list[(size / 2) - 1]) / 2;
        } else {
            return list[size / 2];
        }
    }

    /**
     * 標準偏差計算.
     */
    double Statistics::stdev() {
        return std::sqrt(variance(average()));
    }

    /**
     * 平均値計算.
     */
    double Statistics::average() {
        double summation = 0.0;
        std::for_each(list.begin(), list.end(), [&summation](double data) {
            summation += data;
        });

        return (summation / list.size());
    }

    /**
     * 分散計算.
     */
    double Statistics::variance(double ave) {
        std::deque<double> square_deviation;

        std::for_each(list.begin(), list.end(), [&square_deviation, &ave](double data) {
            square_deviation.push_back(std::pow((data - ave), 2.0));
        });

        double summation = 0.0;
        std::for_each(square_deviation.begin(), square_deviation.end(), [&summation](double data) {
            summation += data;
        });

        return (summation / square_deviation.size());
    }
}
