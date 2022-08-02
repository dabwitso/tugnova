#include <waypoint_editor.h>

namespace WaypointEditor {
    bool OutputCSV::run(
        const WaypointEditor::ParamGeneral::ConstPtr& general,
        const WaypointEditor::ParamChangeRoute::ConstPtr& change_route,
        const WaypointEditor::ParamCurve::ConstPtr& curve,
        std::deque<WaypointEditor::Data::Ptr>& waypoints
    ) {
        int split_count = getSplitCount(waypoints);
        std::size_t size = waypoints.size();
        int count = 1;
        int start = 0;

        std::string path = general->getOutput() + "/" + general->getName();
        std::string route_path = path + "/ROUTE";
        std::string index_text;
        int hotspots = 0;
        int extension_waypoint_count = getExtensionWaypointCount(curve->getDefaultVelocity(), change_route);
        WaypointEditor::Data::Ptr tail = *(waypoints.rbegin());

        if (split_count >= 2) {
            WaypointEditor::Data::Ptr target;
            if (size > extension_waypoint_count) {
                target = *(waypoints.rbegin() + extension_waypoint_count);
            } else {
                target = *(waypoints.begin());
            }
            target->setSplit(SPLIT_STATUS_TAIL);
        }

        while (count <= split_count) {
            try {
                // C側の処理が複雑になるため、指定ディレクトリに古いディレクトリやCSVファイルがあった場合を考慮しない。
                // ノードの起動スクリプト側で初期化対応すること。
                std::string branch_route_path = route_path + "/" + std::to_string(count);
                mkdir(branch_route_path.c_str(), 0775);

                std::string route_file = branch_route_path + "/" + general->getName() + "_" + std::to_string(count) + ".csv";
                std::string config_file = branch_route_path + "/" + general->getName() + "_" + std::to_string(count) + "_CONFIG.csv";
                std::ofstream route_fs(route_file.c_str(), std::ios::out);
                std::ofstream config_fs(config_file.c_str(), std::ios::out);
                route_fs.exceptions(std::ofstream::failbit | std::ofstream::badbit);
                config_fs.exceptions(std::ofstream::failbit | std::ofstream::badbit);

                int extension_end_point = 0;

                outputRouteCSVHeader(route_fs, waypoints[start]);
                outputConfigCSVHeader(config_fs);

                for (int position = start; position < size; position++) {
                    outputRouteData(route_fs, general, change_route, waypoints[position]);
                    outputConfigData(config_fs, waypoints[position]);

                    if (extension_end_point > 0 && position >= extension_end_point) {
                        extension_end_point = 0;
                        break;
                    }

                    WaypointEditor::Hotspot hotspot = waypoints[position]->getHotspot();
                    if (extension_end_point == 0 && hotspot.hasData()) {
                        hotspots++;
                        int number = count;
                        if (waypoints[position]->getSplit() == 1) { number++; } //ホットスポット登録と経路分割を同一の箇所で登録した場合、切替後の経路にホットスポットを紐づける
                        index_text += (outputHotspot(path, hotspots, hotspot) + ":" + std::to_string(number) + ",");
                    }

                    if (extension_end_point == 0 && waypoints[position]->getSplit() == 1) {
                        start = position + 1;
                        extension_end_point = position + extension_waypoint_count;
                    }
                }

                route_fs.close();
                config_fs.close();
            } catch(const std::exception& e) {
                return false;
            }

            count++;
        }

        try {
            outputIndex(route_path, general->getName(), index_text);
        } catch(const std::exception& e) {
            return false;
        }

        return true;
    }

    /**
     * 分割数の取得.
     */
    int OutputCSV::getSplitCount(std::deque<WaypointEditor::Data::Ptr>& waypoints) {
        int count = 1;
        std::for_each(waypoints.begin(), waypoints.end(), [&count](WaypointEditor::Data::Ptr& data) {
            if (data->getSplit()) { count++; }
        });

        return count;
    }

    /**
     * 延長経路数の取得.
     */
    int OutputCSV::getExtensionWaypointCount(const double velocity, const WaypointEditor::ParamChangeRoute::ConstPtr& change_route) {
        double velocity_ms = (velocity * 1000) / (60 * 60);
        return std::ceil((velocity_ms * change_route->getLookaheadRatio()) / change_route->getWaypointDistance());
    }

    /**
     * 経路ファイルへCSVヘッダの出力、ならびにWaypointID 0(ダミー行)の出力.
     */
    void OutputCSV::outputRouteCSVHeader(std::ofstream& route_fs, WaypointEditor::Data::Ptr& data) {
        std::size_t size = route_csv_header.size();
        int count = 1;

        std::for_each(route_csv_header.begin(), route_csv_header.end(), [&route_fs, &size, &count](std::string& column_name){
            route_fs << column_name;
            if (count < size) { route_fs << ","; }
            count++;
        });
        route_fs << std::endl;

        count = 1;
        std::for_each(route_csv_header.begin(), route_csv_header.end(), [&route_fs, &size, &count, &data](std::string& column_name){
            // 大半のカラムは値=0で良いが、ダミーであっても一部変更した方がよいものは変える
            if (column_name == "x") {
                route_fs << data->getX();
            } else if (column_name == "y") {
                route_fs << data->getY();
            } else if (column_name == "z") {
                route_fs << data->getZ();
            } else if (column_name == "yaw") {
                route_fs << data->getYaw();
            } else if (column_name == "target_traffic_light_camera") {
                route_fs << NULL_STR; // 未使用
            } else if (column_name == "obstacle_sensor_range") {
                route_fs << 1; // 固定
            } else if (column_name == "mode_change") {
                route_fs << 1; // 2Dセンサーは1がデフォルト
            } else {
                route_fs << 0;
            }

            if (count < size) { route_fs << ","; }
            count++;
        });
        route_fs << std::endl;
    }

    /**
     * 設定ファイルへCSVヘッダの出力.
     */
    void OutputCSV::outputConfigCSVHeader(std::ofstream& config_fs) {
        std::size_t size = config_csv_header.size();
        int count = 1;

        std::for_each(config_csv_header.begin(), config_csv_header.end(), [&config_fs, &size, &count](std::string& column_name){
            config_fs << column_name;
            if (count < size) { config_fs << ","; }
            count++;
        });
        config_fs << std::endl;
    }


    /**
     * 経路データの出力
     */
    void OutputCSV::outputRouteData(
        std::ofstream& route_fs,
        const WaypointEditor::ParamGeneral::ConstPtr& general,
        const WaypointEditor::ParamChangeRoute::ConstPtr& change_route,
        WaypointEditor::Data::Ptr& data
    ) {
        std::size_t size = route_csv_header.size();
        int count = 1;

        std::for_each(route_csv_header.begin(), route_csv_header.end(),
            [&route_fs, &size, &count, &data, &general, &change_route](std::string& column_name){

            if (column_name == "waypoint_id") {
                route_fs << data->getWaypointId();
            } else if (column_name == "x") {
                route_fs << data->getX();
            } else if (column_name == "y") {
                route_fs << data->getY();
            } else if (column_name == "z") {
                route_fs << data->getZ();
            } else if (column_name == "yaw") {
                route_fs << data->getYaw();
            } else if (column_name == "velocity") {
                route_fs << data->getVelocity();
            } else if (column_name == "change_flag") {
                route_fs << 0; // 未使用

            } else if (column_name == "shutter_point") {
                route_fs << 0; // 未使用
            } else if (column_name == "shutter_stop") {
                route_fs << 0; // 未使用
            } else if (column_name == "shutter_passed") {
                route_fs << 0; // 未使用

            } else if (column_name == "steering_flag") {
                route_fs << 0; // 未使用
            } else if (column_name == "accel_flag") {
                route_fs << 0; // 未使用
            } else if (column_name == "stop_flag") {
                route_fs << 0; // 未使用
            } else if (column_name == "event_flag") {
                route_fs << 0; // 未使用
            } else if (column_name == "traffic_light_detection") {
                route_fs << 0; // 固定
            } else if (column_name == "target_traffic_light_camera") {
                route_fs << NULL_STR; // 未使用
            } else if (column_name == "start") {
                route_fs << 0; // 未使用
            } else if (column_name == "goal") {
                route_fs << 0; // 未使用
            } else if (column_name == "stop_point") {
                route_fs << data->getStop();
            } else if (column_name == "detection_range_front_l") {
                route_fs << data->getDetectionRangeFrontL();
            } else if (column_name == "detection_range_front_r") {
                route_fs << data->getDetectionRangeFrontR();
            } else if (column_name == "detection_range_rear_l") {
                route_fs << data->getDetectionRangeRearL();
            } else if (column_name == "detection_range_rear_r") {
                route_fs << data->getDetectionRangeRearR();
            } else if (column_name == "fatal_area_left") {
                route_fs << 0; // 未使用
            } else if (column_name == "fatal_area_right") {
                route_fs << 0; // 未使用
            } else if (column_name == "cargo_left") {
                route_fs << general->getDefaultCargoRange();
            } else if (column_name == "cargo_right") {
                route_fs << general->getDefaultCargoRange();
            } else if (column_name == "obstacle_detection_distance_change") {
                route_fs << general->getDefaultObstacleDetectionDistanceChange();
            } else if (column_name == "winker_point") {
                route_fs << data->getWinkerPoint();
            } else if (column_name == "obstacle_sensor_range") {
                route_fs << data->getSensor2D();
            } else if (column_name == "mode_change") {
                route_fs << 1; // 固定
            } else if (column_name == "attention_broadcast") {
                route_fs << 0; // 未使用
            } else if (column_name == "inter_vehicular_distance") {
                route_fs << 0; // 未使用
            } else if (column_name == "minimum_target_point_distance") {
                route_fs << general->getDefaultMinimumTargetPointDistance();
            } else if (column_name == "target_point_ratio") {
                route_fs << change_route->getLookaheadRatio();
            } else if (column_name == "voxel_leaf_size") {
                route_fs << 0; // 未使用
            } else if (column_name == "local_max_slope") {
                route_fs << 0; // 未使用
            } else if (column_name == "preliminary_output") {
                route_fs << 0; // 固定
            } else if (column_name == "route_change") {
                route_fs << data->getSplit();
            } else if (column_name == "pause_point") {
                route_fs << data->getPause();
            } else if (column_name == "pause_time") {
                route_fs << general->getDefaultPauseTime();
            } else if (column_name == "cross_point") {
                route_fs << 0; // 固定
            } else if (column_name == "cross_point_detection_area_left") {
                route_fs << 0; // 固定
            } else if (column_name == "cross_point_detection_area_right") {
                route_fs << 0; // 固定
            } else if (column_name == "cross_point_detection_area_front") {
                route_fs << 0; // 固定
            } else if (column_name == "cross_point_detection_area_back") {
                route_fs << 0; // 固定
            } else { // 初期値がセット出来ていないカラムに対して行う設定（通常は入らない）
                route_fs << 0;
            }

            if (count < size) { route_fs << ","; }
            count++;
        });
        route_fs << std::endl;
    }

    /**
     * 設定データの出力.
     */
    void OutputCSV::outputConfigData(std::ofstream& config_fs, WaypointEditor::Data::Ptr& data) {
        std::size_t size = config_csv_header.size();

        int count = 1;

        std::for_each(config_csv_header.begin(), config_csv_header.end(), [&](std::string& column_name){

            if (column_name == "waypoint_id") {
                config_fs << data->getWaypointId();
            } else if (column_name == "location_name") {
                config_fs << LOCALTION_NAME; // 固定
            } else if (column_name == "input_target_equip1") {
                config_fs << getInputTarget(data->getInputTargets(), 0);
            } else if (column_name == "input_target_equip2") {
                config_fs << getInputTarget(data->getInputTargets(), 1);
            } else if (column_name == "input_target_equip3") {
                config_fs << getInputTarget(data->getInputTargets(), 2);
            } else if (column_name == "output_target_equip1") {
                config_fs << getOutputTarget(data->getOutputTargets(), 0).target_id;
            } else if (column_name == "output_value1") {
                config_fs << getOutputTarget(data->getOutputTargets(), 0).value;
            } else if (column_name == "output_target_equip2") {
                config_fs << getOutputTarget(data->getOutputTargets(), 1).target_id;
            } else if (column_name == "output_value2") {
                config_fs << getOutputTarget(data->getOutputTargets(), 1).value;
            } else if (column_name == "output_target_equip3") {
                config_fs << getOutputTarget(data->getOutputTargets(), 2).target_id;
            } else if (column_name == "output_value3") {
                config_fs << getOutputTarget(data->getOutputTargets(), 2).value;
            } else { // 初期値がセット出来ていないカラムに対して行う設定（通常は入らない）
                config_fs << "";
            }

            if (count < size) { config_fs << ","; }
            count++;
        });
        config_fs << std::endl;
    }

    /**
     * Hotspotファイルを作成する.
     * 関数から戻る時、作成したホットスポットファイル名を返却する.
     */
    std::string OutputCSV::outputHotspot(const std::string path, const int hotspots, WaypointEditor::Hotspot& hotspot) {
        std::string filename = "HOTSPOT_" + std::to_string(hotspots) + "_" + std::to_string(hotspot.getX()) + "_" + std::to_string(hotspot.getY()) +  ".csv";
        std::string hotspot_file = path + "/HOTSPOTS/" + filename;
        std::ofstream hotspot_fs(hotspot_file.c_str(), std::ios::out);

        hotspot_fs << "0,0,0,0,"; // 後方互換のためのダミーデータ
        hotspot_fs << hotspot.getX() << ",";
        hotspot_fs << hotspot.getY() << ",";
        hotspot_fs << hotspot.getZ() << ",";
        hotspot_fs << hotspot.getRoll() << ",";
        hotspot_fs << hotspot.getPitch() << ",";
        hotspot_fs << hotspot.getYaw();

        hotspot_fs.close();
        return filename;
    }

    /**
     * Indexファイルを作成する.
     */
    void OutputCSV::outputIndex(const std::string route_path, const std::string name, std::string index_text) {
        std::string index_file = route_path + "/" + name + "_INDEX.csv";
        std::ofstream index_fs(index_file.c_str(), std::ios::out);

        if (index_text.length() >= 1) {
            index_text.erase(index_text.end()-1, index_text.end()); // 終端のカンマを除去
            index_fs << index_text;
        }
        index_fs.close();
    }

    /**
     * 入力設備情報を取り出す.
     */
    std::string OutputCSV::getInputTarget(std::vector<std::string> input_targets, int offset) {
        if (input_targets.size() <= offset) { return ""; }
        return *(input_targets.begin() + offset);
    }

    /**
     * 出力設備情報を取り出す.
     */
    carctl_msgs::output_target OutputCSV::getOutputTarget(std::vector<carctl_msgs::output_target> output_targets, int offset) {
        if (output_targets.size() <= offset) {
            carctl_msgs::output_target result;
            result.target_id = "";
            result.value = "";
            return result;
        }
        return *(output_targets.begin() + offset);
    }
}
