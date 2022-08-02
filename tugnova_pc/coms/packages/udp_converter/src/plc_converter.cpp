#include <plc_converter.h>

namespace Udp_ns
{
    PlcConverter::PlcConverter() : private_nh("~")
    {
        //Setup flags
        udp_direct_control_flag = false;
        use_low_pass_filter = false;
        use_median_filter = false;

        // twistCmdCallback初期値設定
        nowtime = ros::WallTime::now();
        runtime = ros::WallTime::now();
        stoptime = ros::WallTime::now();

        //Read parameters
        private_nh.param<bool>("direct_control_flag", udp_direct_control_flag, false);
        private_nh.param<bool>("use_low_pass_filter", use_low_pass_filter, false);
        private_nh.param<bool>("use_median_filter", use_median_filter, false);
        private_nh.param<double>("lowpass_filter_cutoff_frequency_wr", lowpass_filter_cutoff_frequency_wr, 50.0);
        private_nh.param<double>("lowpass_filter_cutoff_frequency_wl", lowpass_filter_cutoff_frequency_wl, 50.0);
        private_nh.param<double>("lowpass_filter_sampling_frequency_wr", lowpass_filter_sampling_frequency_wr, 100.0);
        private_nh.param<double>("lowpass_filter_sampling_frequency_wl", lowpass_filter_sampling_frequency_wl, 100.0);
        private_nh.param<double>("lowpass_filter_size_wr", lowpass_filter_size_wr, 20);
        private_nh.param<double>("lowpass_filter_size_wl", lowpass_filter_size_wl, 20);
        private_nh.param<double>("median_filter_size_wr", median_filter_size_wr, 5);
        private_nh.param<double>("median_filter_size_wl", median_filter_size_wl, 5);
        private_nh.param<double>("encoder_pulse_resolution", encoder_pulse_resolution, 1.0);
        private_nh.param<double>("receive_error_threshold", receive_error_threshold, 1.0);

        private_nh.param<std::string>("twist_cmd_topic", twist_cmd_topic, "/twist_cmd");
        private_nh.param<std::string>("AccelCmd_topic", AccelCmd_topic, "/AccelCmd");
        private_nh.param<std::string>("SteerCmd_topic", SteerCmd_topic, "/SteerCmd");
        private_nh.param<std::string>("BrakeCmd_topic", BrakeCmd_topic, "/BrakeCmd");
        // private_nh.param<std::string>("LampCmd_topic", LampCmd_topic, "/LampCmd");
        private_nh.param<std::string>("IndicatorCmd_topic", IndicatorCmd_topic, "/IndicatorCmd");
        private_nh.param<std::string>("gear_cmd_topic", gear_cmd_topic, "/gear_cmd");
        private_nh.param<std::string>("mode_cmd_topic", mode_cmd_topic, "/mode_cmd");
        private_nh.param<std::string>("plc_errormsg_topic", plc_errormsg_topic, "/plc_errormsg");

        private_nh.param<std::string>("waypoint_topic", waypoint_topic, "/safety_waypoints");

        private_nh.param<std::string>("coms", coms_str, "green");
        private_nh.param<int32_t>("waittime", waittime, 3.0);
        private_nh.param<std::string>("lidar_vel_topic", lidar_vel_topic, "/current_velocity"); // 20200908@add : golf_cart driver
        private_nh.param<std::string>("stopflg_topic", stopflg_topic, "/stop_flg");             // 20210209@add : acc_frc_ref (running Botton)

        private_nh.param<double>("/vehicle_info/wheel_base", wheel_base, 1.14);
        private_nh.param<double>("/vehicle_info/minimum_turning_radius", minimum_turning_radius, 1.6);
        private_nh.param<double>("/vehicle_info/maximum_steering_angle", maximum_steering_angle, 65.0);
        private_nh.param<double>("/vehicle_info/tire_radius", tire_radius, 0.2);
        private_nh.param<double>("/vehicle_info/tire_distance", tire_distance, 1.14);

        fprintf(stdout, "wheel_base: %lf\n", wheel_base);
        fprintf(stdout, "minimum_turning_radius: %lf\n", minimum_turning_radius);
        fprintf(stdout, "maximum_steering_angle: %lf\n", maximum_steering_angle);
        fprintf(stdout, "tire_radius: %lf\n", tire_radius);
        fprintf(stdout, "tire_distance: %lf\n", tire_distance);


        //Setup coms
        if (coms_str == "green")
        {
            coms = GREEN;
            accelGain = 0.7;
            brakeGain = 0.2;
        }
        else if (coms_str == "yukuri")
            coms = YUKURI;
        else
            ROS_ERROR("COMS type has been set wrongly.");

        //Setup subscribers
        switch (coms)
        {
        case GREEN:
            udp_sensor_packet_sub = nh.subscribe("/plc_sensor_packet", 1,
                                                 &PlcConverter::udpSensorPacketGreenComsCallback, this);
            break;
        case YUKURI:
            udp_sensor_packet_sub = nh.subscribe("/plc_sensor_packet", 1,
                                                 &PlcConverter::udpSensorPacketCallback, this);
            break;
        }
        liadr_vel_sub = nh.subscribe(lidar_vel_topic, 1, &PlcConverter::lidarvelCallback, this); // 20200908@add : golf_cart driver
        stopflg_sub = nh.subscribe(stopflg_topic, 1, &PlcConverter::stopflgCallback, this);      // 20210209@add : acc_frc_ref (running Botton)
        twist_cmd_sub = nh.subscribe(twist_cmd_topic, 1, &PlcConverter::twistCmdCallback, this);
        AccelCmd_sub = nh.subscribe(AccelCmd_topic, 1, &PlcConverter::accelCmdCallback, this);
        SteerCmd_sub = nh.subscribe(SteerCmd_topic, 1, &PlcConverter::steerCmdCallback, this);
        BrakeCmd_sub = nh.subscribe(BrakeCmd_topic, 1, &PlcConverter::brakeCmdCallback, this);
        // LampCmd_sub = nh.subscribe(LampCmd_topic, 1, &PlcConverter::lampCmdCallback, this);
        //↓削除_3-3-4_走行制御_ウィンカーライトを点灯
        // IndicatorCmd_sub = nh.subscribe(IndicatorCmd_topic, 1, &PlcConverter::indicatorCmdCallback, this);
        gear_cmd_sub = nh.subscribe(gear_cmd_topic, 1, &PlcConverter::gearCmdCallback, this);
        // mode_cmd_sub = nh.subscribe(mode_cmd_topic, 1, &PlcConverter::modeCmdCallback, this);
        plc_errormsg_sub = nh.subscribe(plc_errormsg_topic, 1, &PlcConverter::plcErrormsgCallback, this);
        waypoint_sub = nh.subscribe(waypoint_topic, 1, &PlcConverter::waypointCallback, this);
        emergency_stop_sub = nh.subscribe("emergency_stop_state", 1, &PlcConverter::EmergencyStopCallback, this);
        sub_SrvStart = nh.subscribe("SrvStartFlg", 10, &PlcConverter::SrvStartCallBack, this);
        sub_PouseStart = nh.subscribe("PouseStartFlg", 10, &PlcConverter::PouseStartCallBack, this);
        sub_ConnectStart = nh.subscribe("ConnectStartFlg", 10, &PlcConverter::ConnectStartCallBack, this);
        sub_BatteryStart = nh.subscribe("BatteryStartFlg", 10, &PlcConverter::BatteryStartCallBack, this);
        sub_scan_respawn_result = nh.subscribe("/scan_respawn_result", 10, &PlcConverter::scanRespawnResultCallback, this);
        sub_monitor_status = nh.subscribe("/monitor_status", 10, &PlcConverter::monitorstatusCallback, this);

        //Setup publishers
        curr_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/curr_twist", 1);
        curr_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/curr_pose", 1);
        udp_control_packet_pub = nh.advertise<udp_msgs::UdpControlPacket>("/plc_control_packet_raw", 1);
        pub_emergency_error = nh.advertise<carctl_msgs::emergency_status>("emergency_error", 10);
        pub_monitor_status = nh.advertise<carctl_msgs::monitor_status>("/monitor_status", 10);

        //Setup filters
        if (use_median_filter)
        {
            median_wr.setSize(median_filter_size_wr);
            median_wl.setSize(median_filter_size_wl);
        }
        if (use_low_pass_filter)
        {
            lowpass_wr.setFrequency(lowpass_filter_cutoff_frequency_wr, lowpass_filter_sampling_frequency_wr);
            lowpass_wl.setFrequency(lowpass_filter_cutoff_frequency_wl, lowpass_filter_sampling_frequency_wl);
            lowpass_wr.setSize(lowpass_filter_size_wr);
            lowpass_wl.setSize(lowpass_filter_size_wl);
        }

        //Setup variables
        ros::Time now = ros::Time::now();
        previous_time = now;
        udp_sensor_packet_updated_time = now;

        twist_cmd_topic_receive_time = calc_time();
        emergency_stop_state = RUN;
        srv_start_flg = OFF;
        pouse_start_flg = OFF;
        connect_start_flg = OFF;
        battery_start_flg = OFF;
        is_detection_lidar = NO_DETECTION;
        cancel_possible_emergency_error_by_scan = NOT_OCCURRED;
    };

    void PlcConverter::run()
    {
        ros::Rate loop_rate(100);
        while (ros::ok())
        {
            private_nh.param<bool>("direct_control_flag", udp_direct_control_flag, false);
            ros::spinOnce();

            if (udp_control_packet_msg.wd_count > 10000)
                udp_control_packet_msg.wd_count = 0;
            else
                udp_control_packet_msg.wd_count++;

            //トピックの受信間隔を取得
            bool is_receive_error = false;
            int16_t emergency_state = INFO;

            //異常メッセージ
            int16_t monitor_status_state = INFO;
            std::string monitor_status_id = NOMAL_ID;

            if (udp_sensor_packet_msg.ECUMode != 0)
            {
                //自動運転中のみ受信間隔異常を検知する
                if (check_topic_receive_time(twist_cmd_topic_receive_time))
                {
                    //受信間隔が一定時間以上経過している場合、緊急停止異常を出力し、緊急停止を行う
                    emergency_state = ERROR;
                    is_receive_error = true;
                    cancel_possible_emergency_error_by_scan = OCCURRED;
                }
            }

            if (cancel_possible_emergency_error_by_scan == OCCURRED)
            {
                //緊急停止している場合、自己位置復帰をするまで緊急停止状態を継続する
                emergency_state = ERROR;
                is_receive_error = true;
                monitor_status_state = ERROR;
                monitor_status_id = ERROR_ID;
            }

            publish_emergency_error(emergency_state);
            publishMonitorStatus(monitor_status_state, monitor_status_id);

            //車両挙動指示の設定。
            std::string vehicle_motion_cmd_string = "";
            if (is_receive_error || emergency_stop_state == STOP)
            {
                //「トピックの受信間隔が一定時間異常経過している場合」または「緊急停止指示を受信している場合」、A1.緊急停止
                udp_control_packet_msg.vehicle_motion_cmd = A1;
                vehicle_motion_cmd_string = "A1";
            }
            else if (is_detection_lidar == DETECTION)
            {
                //障害物を検知している場合、D2.予防停止
                udp_control_packet_msg.vehicle_motion_cmd = D2;
                vehicle_motion_cmd_string = "D2";
            }
            else if (pouse_start_flg == OFF || connect_start_flg == OFF || battery_start_flg == OFF)
            {
                //一旦停止地点で停止中の場合、E1.判断停止
                udp_control_packet_msg.vehicle_motion_cmd = E1;
                vehicle_motion_cmd_string = "E1";
            }
            else if (srv_start_flg == OFF)
            {
                //停止箇所で停止している場合、E2.計画停止
                udp_control_packet_msg.vehicle_motion_cmd = E2;
                vehicle_motion_cmd_string = "E2";
            }
            else
            {
                //どの車両挙動指示にも当てはまらない場合、初期値を送信
                udp_control_packet_msg.vehicle_motion_cmd = EMERGENCY_INIT;
                vehicle_motion_cmd_string = "00";
            }

            ROS_WARN("run: vehicle_motion_cmd:[%s]", vehicle_motion_cmd_string.c_str());
            publishMsgs();
            //Need to add time-out function
            loop_rate.sleep();
        }
    };

    // 現在時刻をミリ秒で取得する
    double PlcConverter::calc_time()
    {
        struct ::timespec getTime;
        clock_gettime(CLOCK_MONOTONIC, &getTime);
        return (getTime.tv_sec + getTime.tv_nsec * 1e-9) * 1000;
    };

    // トピックの受信間隔が閾値以上空いてる場合trueを返す
    bool PlcConverter::check_topic_receive_time(double topic_receive_time)
    {
        double topic_receive_diff_time = calc_time() - topic_receive_time;
        return ((receive_error_threshold * 1000.0) <= topic_receive_diff_time) ? true : false;
    };

    // 緊急停止指示ノードに異常を通知する
    double PlcConverter::publish_emergency_error(int16_t status)
    {
        //受信しているトピックの内一つでも間隔が空いていた場合は受信エラーとする
        carctl_msgs::emergency_status msg;
        //ノード名は固定
        msg.service_name = "plc_converter";
        //状態を格納
        msg.status = status;
        pub_emergency_error.publish(msg);
    };

    //エラーを出力する
    double PlcConverter::publishMonitorStatus(int16_t status, std::string error_msg)
    {
        //受信しているトピックの内一つでも間隔が空いていた場合は受信エラーとする
        carctl_msgs::monitor_status msg;
        //ノード名は固定
        msg.service_name = "plc_converter";
        //状態を格納
        msg.status = status;
        //エラーメッセージを格納
        msg.error_msg = error_msg;
        pub_monitor_status.publish(msg);
    };

    void PlcConverter::publishMsgs()
    {
        curr_twist_pub.publish(curr_twist_msg);
        curr_pose_pub.publish(curr_pose_msg);
        udp_control_packet_pub.publish(udp_control_packet_msg);
    };

    void PlcConverter::udpSensorPacketCallback(const udp_msgs::UdpSensorPacket &msg)
    {
        udp_sensor_packet_msg = msg;
        udp_sensor_packet_updated_time = ros::Time::now();

        computeEncoderSpeed(encoder_pulse_resolution, tire_radius, tire_distance, use_low_pass_filter, use_median_filter);
    };

    void PlcConverter::udpSensorPacketGreenComsCallback(const udp_msgs::UdpSensorPacket_GreenComs &msg)
    {
        udp_sensor_packet_green_coms_msg = msg;
        udp_sensor_packet_updated_time = ros::Time::now();

        computeEncoderSpeed(encoder_pulse_resolution, tire_radius, tire_distance, use_low_pass_filter, use_median_filter);
    };

    void PlcConverter::twistCmdCallback(const geometry_msgs::TwistStamped &msg)
    {
        //受信時刻を取得
        twist_cmd_topic_receive_time = calc_time();

        if (!udp_direct_control_flag)
        {
            switch (coms)
            {
            case GREEN:
            {
                float vel = udp_control_packet_msg.pc_speed_mps;
                float velref = clipValue((float)msg.twist.linear.x / 3.6f, 0.0f, 10.0f);

                float veldiff = velref - vel;

                float acc_pdl = 0.0f;
                float brk_pdl = -0.07f;

                //Case 1: Reference velocity is smaller than 0.1[m/s] -> stop
                if (velref < 0.1)
                {
                    acc_pdl = 0.0f;
                    brk_pdl = brakeGain * (30.0f / 3.6f - vel);
                    brk_pdl = clipValue(brk_pdl, 0.5f, 1.0f);
                }
                //Case 2: Reference velocity is much bigger than current velocity -> acceleration
                else if (veldiff > 10.0f / 3.6f)
                {
                    acc_pdl = accelGain * veldiff;
                    brk_pdl = -0.07f;
                }
                //Case 3: Reference velocity is much smaller than current velocity -> deceleration
                else if (veldiff < -10.0f / 3.6f)
                {
                    acc_pdl = 0.0f;
                    brk_pdl = -clipValue(accelGain * veldiff, 0.0f, 1.0f);
                }
                //Case 4: Engine brake
                else
                {
                    acc_pdl = clipValue(accelGain * veldiff, 0.0f, 1.0f);
                    brk_pdl = -0.07f;
                }

                acc_pdl = clipValue(acc_pdl, 0.0f, 1.0f);
                brk_pdl = clipValue(brk_pdl, -0.07f, 1.0f);

                int tmp = (int)(2026 + acc_pdl * (8720 - 2026));
                // udp_control_packet_msg.acc_frc_ref = tmp;

                tmp = brk_pdl * 6000;
                udp_control_packet_msg.brk_frc_ref = tmp;

                break;
            }
            case YUKURI:
            {
                // 警告音
                int idel_sound = 0;    // 無音
                int warning_sound = 1; // 警告：ブーブーブー
                int start_sound = 2;   // 走行開始：プルルル
                int moving_sound = 3;  // 走行音：カンカンカン
                int arrival_sound = 4; // 到着音

                // 表示灯
                int yellow_blinking_02 = 1; // 黄色点滅（0.2秒間隔）：走行中
                int yellow_blinking_10 = 2; // 黄色点滅（1.0秒間隔）：停止中
                int red_blinking = 3;       // 赤点滅：障害中

                // 停止速度
                float stop_twist = 0.0;

                // 走行値（acc_vel_ref）・警告音（brk_cmode）・表示灯（acc_pos_ref）をセット
                // 走行ステータス
                int veicle_status_ = veicle_status;
                // 実速度（PLCからの値）
                float PLC_vel = udp_sensor_packet_msg.PLC_vel;
                // 自動運転切り替えスイッチ
                int ECUMode = udp_sensor_packet_msg.ECUMode;
                // 経過時間
                ros::WallDuration elapsed;
                // 現在の速度
                float twistx = msg.twist.linear.x;
                // 現在のタイヤ角
                float angularz = msg.twist.angular.z;
                // 現在時刻の取得
                nowtime = ros::WallTime::now();
                // strftime(date, sizeof(date), "%H:%M:%S", nowtime);

                // 手動運転時
                if (ECUMode == 0)
                {
                    fprintf(stdout, "DEBUG:twistCmdCallback:手動走行中\n");
                    // 停止、IDLE、黄色点滅（1秒間隔）
                    drivingValueControl(stop_twist, angularz, idel_sound, yellow_blinking_10);
                    last_twistx = twistx;
                    break;
                }

                // 障害時停止
                if (veicle_status_ == 1)
                {
                    fprintf(stdout, "DEBUG:twistCmdCallback:障害時停止中\n");
                    // 停止、FAILURE、赤点滅
                    drivingValueControl(stop_twist, angularz, warning_sound, red_blinking);
                    last_twistx = twistx;
                    break;
                }

                // 通常停止時
                if (twistx == 0.0)
                {
                    // 停止箇所到着
                    if (last_twistx > 0.0)
                    {
                        // 停止時のタイマーセット
                        stoptime = ros::WallTime::now();
                        fprintf(stdout, "DEBUG:twistCmdCallback:停止箇所到着\n");
                    }

                    // 経過時間のセット
                    elapsed = nowtime - stoptime;
                    // 停止箇所到着、一定時間到着音を鳴らす
                    if (elapsed.sec < waittime)
                    {
                        fprintf(stdout, "DEBUG:twistCmdCallback:停止箇所到着、経過時間[%d]\n", elapsed.sec);
                        // 停止、到着音、黄色点滅（1秒間隔）
                        drivingValueControl(twistx, angularz, arrival_sound, yellow_blinking_10);
                    }
                    // 停止中(waittime経過後)
                    else if (elapsed.sec >= waittime)
                    {
                        fprintf(stdout, "DEBUG:twistCmdCallback:通常停止中\n");
                        // 停止、無音、黄色点滅（1秒間隔）
                        drivingValueControl(twistx, angularz, idel_sound, yellow_blinking_10);
                    }
                }
                // 走行開始
                else if (twistx > 0.0)
                {
                    if (last_twistx == 0.0)
                    {
                        // 走行開始のタイマーセット
                        runtime = ros::WallTime::now();
                        fprintf(stdout, "DEBUG:twistCmdCallback:発進開始前\n");
                    }

                    // 経過時間のセット
                    elapsed = nowtime - runtime;
                    // 発進待機中
                    if (elapsed.sec < waittime)
                    {
                        fprintf(stdout, "DEBUG:twistCmdCallback:発進待機中、経過時間[%d]\n", elapsed.sec);
                        // 停止、発進前の警告、黄色点滅（1秒間隔）
                        drivingValueControl(stop_twist, angularz, start_sound, yellow_blinking_10);
                    }
                    // 走行中
                    else if (elapsed.sec >= waittime)
                    {
                        fprintf(stdout, "DEBUG:twistCmdCallback:走行中\n");
                        // 走行値、走行音、黄色点滅（0.2秒間隔）
                        drivingValueControl(twistx, angularz, moving_sound, yellow_blinking_02);
                    }
                }
                else
                {
                    fprintf(stdout, "DEBUG:twistCmdCallback:例外 ");
                    fprintf(stdout, "走行値[%2.1lf]、経過時間[%d]\n", twistx * 3.6, elapsed.sec);
                }
                last_twistx = twistx;
            }
            }
        }
    }; // namespace Udp_ns

    void PlcConverter::accelCmdCallback(const autoware_msgs::AccelCmd &msg)
    {
        if (udp_direct_control_flag)
        {
            switch (coms)
            {
            case GREEN:
            {
                int tmp = clipValue((int)msg.accel, 0, 100);
                // udp_control_packet_msg.acc_frc_ref = (int)(2026 + (float)tmp / 100.0 * (8720 - 2026));
                break;
            }
            case YUKURI:
            {
                int tmp = clipValue((int)msg.accel, 0, 20);
                udp_control_packet_msg.acc_vel_ref = (float)tmp / 3.6f;
                break;
            }
            }
        }
    };

    void PlcConverter::steerCmdCallback(const autoware_msgs::SteerCmd &msg)
    {
        if (udp_direct_control_flag)
        {
            float tmp = clipValue((float)msg.steer / 17.5f, -35.0f, 35.0f);  //steering wheel angle -> tire angle
            udp_control_packet_msg.tir_ang_ref = (float)tmp * M_PI / 180.0f; //degree -> radian
        }
    };

    void PlcConverter::brakeCmdCallback(const autoware_msgs::BrakeCmd &msg)
    {
        if (udp_direct_control_flag)
        {
            switch (coms)
            {
            case GREEN:
            {
                int tmp = clipValue((int)msg.brake, 0, 100);
                udp_control_packet_msg.brk_frc_ref = (int)(-420 + (float)tmp / 100 * (6000 - (-420)));
                break;
            }

            case YUKURI:
            {
                int tmp = clipValue((int)msg.brake, 0, 100);
                break;
            }
            }
        }
    };

    // void PlcConverter::lampCmdCallback(const autoware_msgs::LampCmd &msg)
    // {
    //     switch (coms)
    //     {
    //     case GREEN: //Order with pedal operation
    //     {
    //         udp_control_packet_msg.acc_cmode = msg.l ? 1 : 0;
    //         udp_control_packet_msg.brk_cmode = msg.l ? 1 : 0;
    //         break;
    //     }
    //     case YUKURI:
    //     {
    //         udp_control_packet_msg.acc_cmode = msg.l ? 2 : 0; //int32
    //         udp_control_packet_msg.brk_cmode = msg.l ? 3 : 0;
    //         break;
    //     }
    //     }
    //     udp_control_packet_msg.tir_cmode = msg.r ? 3 : 0; //int32
    // };

    //↓削除_3-3-4_走行制御_ウィンカーライトを点灯
    // void PlcConverter::indicatorCmdCallback(const autoware_msgs::IndicatorCmd &msg)
    // {
    //     udp_control_packet_msg.winkerL = msg.l; //int32
    //     udp_control_packet_msg.winkerR = msg.r; //int32
    // };

    void PlcConverter::gearCmdCallback(const tablet_socket_msgs::gear_cmd &msg)
    {
        int tmp = msg.gear; //int32
        switch (tmp)
        {
        case 1: //D
            udp_control_packet_msg.gearD = 1;
            udp_control_packet_msg.gearR = 0;
            break;
        case 2: //R
            udp_control_packet_msg.gearD = 0;
            udp_control_packet_msg.gearR = 1;
            break;
        case 3: //B (but set as 0,0)
            udp_control_packet_msg.gearD = 0;
            udp_control_packet_msg.gearR = 0;
            break;
        case 4: //N
            udp_control_packet_msg.gearD = 1;
            udp_control_packet_msg.gearR = 1;
            break;
        }
    };

    // 20200908@add : golf_cart driver
    void PlcConverter::lidarvelCallback(const geometry_msgs::TwistStamped &msg)
    {                                                             //add
        udp_control_packet_msg.pc_speed_mps = msg.twist.linear.x; //
    };

    // waypointCallbackでauto_controlを利用するため廃止
    // void PlcConverter::modeCmdCallback(const tablet_socket_msgs::mode_cmd &msg)
    // {
    //     udp_control_packet_msg.auto_control = msg.mode; //int32
    // };

    // 20210209@add : acc_frc_ref (running Botton)
    void PlcConverter::stopflgCallback(const std_msgs::Int16 &msg)
    {
        // 発進ボタン
        udp_control_packet_msg.acc_frc_ref = msg.data;
    };

    void PlcConverter::plcErrormsgCallback(const std_msgs::Int16 &msg)
    {
        // 車両ステータス
        veicle_status = msg.data;
    };

    void PlcConverter::waypointCallback(const autoware_msgs::Lane &msg)
    {
        udp_control_packet_msg.auto_control = msg.waypoints[0].wpc.mode_change;
        udp_control_packet_msg.control_mode = msg.waypoints[0].wpc.osr;
        //↓追加_3-3-4_走行制御_ウィンカーライトを点灯
        switch (msg.waypoints[0].wpc.winker_point)
        {
        case 0: //ウィンカーなし
            udp_control_packet_msg.winkerL = 0;
            udp_control_packet_msg.winkerR = 0;
            break;
        case 1: //左ウィンカー
            udp_control_packet_msg.winkerL = 1;
            udp_control_packet_msg.winkerR = 0;
            break;
        case 2: //右ウィンカー
            udp_control_packet_msg.winkerL = 0;
            udp_control_packet_msg.winkerR = 1;
            break;
        case 3: //ハザード
            udp_control_packet_msg.winkerL = 0;
            udp_control_packet_msg.winkerR = 0;
            break;
        default: //その他
            fprintf(stdout, "DEBUG: waypointCallback:ウィンカー点灯値が正しくありません waypoint=[%d] winker_point=[%d]\n", msg.waypoints[0].wpc.waypoint_id, msg.waypoints[0].wpc.winker_point);
            break;
        }

        // 予備出力枠をPLCへ通知
        udp_control_packet_msg.gearR = msg.waypoints[0].wpc.preliminary_output;
    };

    void PlcConverter::EmergencyStopCallback(const std_msgs::Int16 &msg)
    {
        //緊急停止状態を更新
        emergency_stop_state = msg.data;
    };

    void PlcConverter::SrvStartCallBack(const std_msgs::Int16 &flg)
    {
        srv_start_flg = flg.data;
    };

    void PlcConverter::PouseStartCallBack(const std_msgs::Int16 &flg)
    {
        pouse_start_flg = flg.data;
    };

    void PlcConverter::ConnectStartCallBack(const std_msgs::Int16 &flg)
    {
        connect_start_flg = flg.data;
    };

    void PlcConverter::BatteryStartCallBack(const std_msgs::Int16 &flg)
    {
        battery_start_flg = flg.data;
    };

    void PlcConverter::monitorstatusCallback(const carctl_msgs::monitor_status &m_status)
    {

        std::string service_name = m_status.service_name;
        int status = m_status.status;
        std::string error_msg = m_status.error_msg;

        if (service_name == "velocity_set")
        {
            //障害物検知状態を取得
            is_detection_lidar = (status == 1) ? DETECTION : NO_DETECTION;
        }
    };

    void PlcConverter::scanRespawnResultCallback(const std_msgs::Int16 msg)
    {
        cancel_possible_emergency_error_by_scan = msg.data;
    };

    void PlcConverter::drivingValueControl(float acc_vel_ref_, float angularz_, int brk_cmode_, int acc_pos_ref_)
    {
        // 走行値（acc_vel_ref）・警告音（brk_cmode）・表示灯（acc_pos_ref）をセット
        udp_control_packet_msg.acc_vel_ref = acc_vel_ref_; // 速度
        udp_control_packet_msg.brk_cmode = brk_cmode_;     // 警告音
        udp_control_packet_msg.acc_pos_ref = acc_pos_ref_; // 表示灯
        fprintf(stdout, "DEBUG:drivingValueControl:警告音:[%d] 表示灯:[%d] 走行値:[%2.1lf]", brk_cmode_, acc_pos_ref_, acc_vel_ref_ * 3.6);
        //last_twistx = acc_vel_ref_;

        // Yaw rate -> steer angle (tire angle)
        // Put the value only when the velocity is not 0.
        // Do not set any value on tir_ang_ref when the velocity is 0 and just keep the previous value.
        if (acc_vel_ref_ > 0.1)
        {
            float tmp;
            tmp = wheel_base * angularz_ / acc_vel_ref_;
            tmp = atan(tmp);
            tmp = clipValue(tmp, (float)-60.0f / 180.0f * M_PI, (float)60.0f / 180.0f * M_PI);
            udp_control_packet_msg.tir_ang_ref = tmp;
            fprintf(stdout, "、タイヤ角:[%2.5lf]\n", tmp);
        }
        else
        {
            fprintf(stdout, "\n");
        }
    };

    double PlcConverter::clipValue(double input, double min, double max)
    {
        double output = input;
        output = (output > max ? max : output);
        output = (output < min ? min : output);

        return output;
    }

    float PlcConverter::clipValue(float input, float min, float max)
    {
        float output = input;
        output = (output > max ? max : output);
        output = (output < min ? min : output);

        return output;
    }

    int PlcConverter::clipValue(int input, int min, int max)
    {
        int output = input;
        output = (output > max ? max : output);
        output = (output < min ? min : output);

        return output;
    }

    void PlcConverter::computeEncoderSpeed(double encoderPulseResolution, double tireRadius, double tireDistance, bool useLowpassFilter, bool useMedianFilter)
    {
        //Pool sensor
        udp_msgs::UdpSensorPacket_GreenComs udp_sensor_packet_green_coms_now;
        udp_msgs::UdpSensorPacket udp_sensor_packet_now;
        switch (coms)
        {
        case GREEN:
            udp_sensor_packet_green_coms_now = udp_sensor_packet_green_coms_msg;
            break;
        case YUKURI:
            udp_sensor_packet_now = udp_sensor_packet_msg;
            break;
        }
        ros::Time now = udp_sensor_packet_updated_time;

        //Get time difference
        uint64_t diff_time_nsec = now.toNSec() - previous_time.toNSec();
        double diff_time_sec = diff_time_nsec * 1e-09;

        double dt = diff_time_sec;
        if (dt > MAX_DIFF_TIME)
        {
            previous_time = now;

            //Get pulse and calculate difference
            double pulse_rr;
            double pulse_rl;
            double d_pulse_rr;
            double d_pulse_rl;

            switch (coms)
            {
            case GREEN:
                pulse_rr = udp_sensor_packet_green_coms_now.R_Tire;
                pulse_rl = udp_sensor_packet_green_coms_now.L_Tire;
                d_pulse_rr = udp_sensor_packet_green_coms_now.R_Tire - udp_sensor_packet_green_coms_msg_previous.R_Tire;
                d_pulse_rl = udp_sensor_packet_green_coms_now.L_Tire - udp_sensor_packet_green_coms_msg_previous.L_Tire;
                break;
            case YUKURI:
                pulse_rr = udp_sensor_packet_now.R_Tire;
                pulse_rl = udp_sensor_packet_now.L_Tire;
                d_pulse_rr = udp_sensor_packet_now.R_Tire - udp_sensor_packet_msg_previous.R_Tire;
                d_pulse_rl = udp_sensor_packet_now.L_Tire - udp_sensor_packet_msg_previous.L_Tire;
                break;
            }

            if (d_pulse_rr < -THRESHOLD)
            {
                d_pulse_rr += MAX_COUNT - MIN_COUNT;
            }
            if (d_pulse_rr > THRESHOLD)
            {
                d_pulse_rr -= MAX_COUNT - MIN_COUNT;
            }
            if (d_pulse_rl < -THRESHOLD)
            {
                d_pulse_rl += MAX_COUNT - MIN_COUNT;
            }
            if (d_pulse_rl > THRESHOLD)
            {
                d_pulse_rl -= MAX_COUNT - MIN_COUNT;
            }

            //Compute wheel rotation speed
            double wr = d_pulse_rr * 2 * M_PI / (encoderPulseResolution * dt);
            double wl = d_pulse_rl * 2 * M_PI / (encoderPulseResolution * dt);

            double wr_raw = wr;
            double wl_raw = wl;

            //Apply filters
            if (use_median_filter)
            {
                median_wr.savePreviousInput(wr);
                wr = median_wr.compute();
                median_wl.savePreviousInput(wl);
                wl = median_wl.compute();
            }

            if (use_low_pass_filter)
            {
                lowpass_wr.savePreviousInput(wr);
                wr = lowpass_wr.compute();
                lowpass_wl.savePreviousInput(wl);
                wl = lowpass_wl.compute();
            }

            //Compute wheel speed
            double vrr = tireRadius * wr; //m/sec
            double vrl = tireRadius * wl; //m/sec

            double vrr_raw = tireRadius * wr_raw; //m/sec
            double vrl_raw = tireRadius * wl_raw; //m/sec

            double v = clipValue((vrr + vrl) * 0.5, 0.0, 60.0 / 3.6);
            double w = (-vrl + vrr) * 0.5 / (0.5 * tireDistance);

            double v_raw = (vrr_raw + vrl_raw) * 0.5;
            double w_raw = (-vrl_raw + vrr_raw) * 0.5 / (0.5 * tireDistance);

            //Pack the calculated values into the messages
            curr_twist_msg.header.stamp = ros::Time::now();
            //curr_twist_msg.header.frame_id = "/base_link";
            curr_twist_msg.twist.linear.x = v;
            curr_twist_msg.twist.linear.y = 0.0;
            curr_twist_msg.twist.linear.z = 0.0;
            curr_twist_msg.twist.angular.x = 0.0;
            curr_twist_msg.twist.angular.y = 0.0;
            curr_twist_msg.twist.angular.z = w;

            //Pack the velocity into the control packet also
            //udp_control_packet_msg.pc_speed_mps = v; //20200908@del golf_cart driver
            udp_control_packet_msg.pc_yawrate = w;
            udp_control_packet_msg.pc_speedr_mps = vrr;
            udp_control_packet_msg.pc_speedl_mps = vrl;

            switch (coms)
            {
            case GREEN:
                udp_sensor_packet_green_coms_msg_previous = udp_sensor_packet_green_coms_now;
                break;
            case YUKURI:
                udp_sensor_packet_msg_previous = udp_sensor_packet_now;
                break;
            }
        }
    };

    void lowpassFilter::setSize(int sizein)
    {
        if (sizein < 1)
        {
            sizein = 1;
        }
        size = sizein;

        x.resize(size);
        y.resize(size);
        for (int n = 0; n < size; n++)
        {
            x[n] = 0.0;
        }
        for (int n = 0; n < size; n++)
        {
            y[n] = 0.0;
        }
    };

    lowpassFilter::lowpassFilter()
    {
        setSize(10);
        b0 = 0.0;
        b1 = 0.0;
        b2 = 0.0;
        a0 = 0.0;
        a1 = 0.0;
        a2 = 0.0;
    };

    lowpassFilter::lowpassFilter(int sizein)
    {
        setSize(sizein);
        b0 = 0.0;
        b1 = 0.0;
        b2 = 0.0;
        a0 = 0.0;
        a1 = 0.0;
        a2 = 0.0;
    };

    void lowpassFilter::setFrequency(double cutoffFrequency, double samplingFrequency)
    {
        double fd = cutoffFrequency;
        double fs = samplingFrequency;
        double q = 1 / sqrt(2.0);

        //filter coefficient in time domain
        double omega = 2.0 * M_PI * fd / fs;
        double alpha = sin(omega) / (2.0 * q);
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos(omega);
        a2 = 1.0 - alpha;
        b0 = (1.0 - cos(omega)) / 2.0;
        b1 = 1.0 - cos(omega);
        b2 = (1.0 - cos(omega)) / 2.0;
    };

    double lowpassFilter::compute()
    {

        double in1 = 0.0;
        double in2 = 0.0;
        double out1 = 0.0;
        double out2 = 0.0;

        for (int i = 0; i < size; i++)
        {
            y[i] = b0 / a0 * x[i] + b1 / a0 * in1 + b2 / a0 * in2 - a1 / a0 * out1 - a2 / a0 * out2;

            in2 = in1;
            in1 = x[i];
            out2 = out1;
            out1 = y[i];
        }

        return y[size - 1];
    };

    void lowpassFilter::savePreviousInput(double xnew)
    {
        for (int i = 0; i < size - 1; i++)
        {
            x[i] = x[i + 1];
        }
        x[size - 1] = xnew;
    };

    void medianFilter::setSize(int sizein)
    {
        if (sizein < 1)
        {
            sizein = 1;
        }
        size = sizein;

        x.resize(size);
        y.resize(size);
        for (int n = 0; n < size; n++)
        {
            x[n] = 0.0;
        }
        for (int n = 0; n < size; n++)
        {
            y[n] = 0.0;
        }
    };

    medianFilter::medianFilter()
    {
        setSize(5);
    };

    medianFilter::medianFilter(int sizein)
    {
        setSize(sizein);
    };

    double medianFilter::compute()
    {
        std::vector<double> xSorted = x;
        std::sort(xSorted.begin(), xSorted.end());

        int center = floor(size / 2.0);
        if (center > size)
        {
            center = size;
        }
        if (center < 0)
        {
            center = 0;
        }

        return xSorted[center];
    };

    void medianFilter::savePreviousInput(double xnew)
    {
        for (int i = 0; i < size - 1; i++)
        {
            x[i] = x[i + 1];
        }
        x[size - 1] = xnew;
    };

} // namespace Udp_ns

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plc_converter");

    Udp_ns::PlcConverter plc_converter;
    plc_converter.run();
}
