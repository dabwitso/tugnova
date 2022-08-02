#include <mabx_connector.h>

namespace Mabx {

    MabxReceiver::MabxReceiver(unsigned short _port) : UdpReceiver(_port) {  
        monitor_status_pub = nh.advertise<carctl_msgs::monitor_status>("/monitor_status", 10);
        watchdog_timer_thread = std::thread(&MabxReceiver::watchdog_timer, this);
        is_watchdog_alive = true;
    };

    MabxReceiver::~MabxReceiver() {
        is_watchdog_alive = false;
        watchdog_timer_thread.join();
        close(socket_var);
    };

    /**
     * MABXからのデータ受信部.
     */
    void MabxReceiver::udpPacketReceive() {
        recv(socket_var, &mabx_receiver_packet, sizeof(mabx_receiver_packet), 0);
        current[mabx_receiver_packet.id] =  mabx_receiver_packet;
    }

    /**
     * 監視処理.
     */
    void MabxReceiver::watchdog_timer() {        
        while (is_watchdog_alive) {
            carctl_msgs::monitor_status msg;
            msg.service_name = "mabx";
            msg.status = INFO;
            msg.error_msg = "INFO MABX status ok";

            for (const auto& itr : current) {
                uint16_t id = itr.first;
                carctl_msgs::MabxReceiverPacket current_packet = itr.second;

                if (previos.count(id)) { // Exist?
                    carctl_msgs::MabxReceiverPacket previous_packet = previos[id];

                    if (current_packet.vehicle_diagCode != 0) {
                        ROS_ERROR("[DiagCode] id: %d, wdt: %d, code: %d \n", current_packet.id, current_packet.vehicle_WDT, current_packet.vehicle_diagCode);
                        msg.status = ERROR;
                        std::ostringstream diag_code;
                        diag_code << (uint)current_packet.vehicle_diagCode;
                        msg.error_msg = "mabx_1_" + diag_code.str();
                        break;
                    }

                    if (current_packet.vehicle_WDT == previous_packet.vehicle_WDT) {
                        ROS_ERROR("[WatchDog] id: %d, wdt: %d, code: %d \n", current_packet.id, current_packet.vehicle_WDT, current_packet.vehicle_diagCode);
                        msg.status = ERROR;
                        msg.error_msg = "mabx_1_0"; // watchdog error
                        break;
                    }
                }

                previos[id] = current_packet;
            }

            monitor_status_pub.publish(msg);

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    /**
     * メイン関数.
     */
    void MabxReceiver::run() {
        ros::Rate loop_rate(100);
        while(ros::ok()){
            ros::spinOnce();

            udpPacketReceive();

            loop_rate.sleep();
        }
    };
};