#!/usr/bin/env python3
# -*- cofing: utf-8 -*-
import ast
import os
import time
from multiprocessing import Process, Queue

import bluetooth
import rospy
from std_msgs.msg import Int8

queue_to_connect = Queue()
queue_passed_shutter = Queue()
# queue_launch_client = Queue()

serverMACaddress = "E4:5F:01:6E:0A:E5"
PORT = 1

class ShutterError(Exception):
    """shutter error message"""
    pass

class ShutterConnectionTimeOut(Exception):
    """shutter server connection time over """
    pass

class ROSPassedReceiveTimeOut(Exception):
    """ROS passed flag receive time over"""
    pass

class BLEClientSocket(object):
    def __init__(self, mac_address: str, port: int, buffer: int):
        self.__mac_addr = mac_address
        self.__port = port
        self.__buffer = buffer
        self.__ble_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

    def __del__(self):
        self.disconnect()

    @property
    def buffer(self):
        return self.__buffer

    def disconnect(self):
        self.__ble_sock.close()

    def send_message(self, msg: str):
        self.__ble_sock.send(msg)

    def receive_massage(self, buf: int=1024):
        message = self.__ble_sock.recv(buf).decode('utf-8')
        return message

    def __reconnect(self):
        self.__ble_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        return

    def connect_bluetooth(self, timeout: float):
        """If no connection established within specified time, return NG

        Args:
            timeout (float): connection limit time

        Returns:
            (bool):
        """
        start_time = time.time()
        limit_time = float(timeout)
        while True:
            diff_time = time.time() - start_time
            if diff_time < limit_time:
                try:
                    rospy.loginfo('connecting...')
                    self.__ble_sock.connect((self.__mac_addr, self.__port))
                    time.sleep(0.5)
                    rospy.loginfo('Successfully Connected!!')
                    # publish to ros topic
                    os.system('rostopic pub -1 /bluetooth_connection std_msgs/Int8 "data: 1"')
                    return True
                except bluetooth.BluetoothError:
                    os.system('rostopic pub -1 /bluetooth_connection std_msgs/Int8 "data: 2"')
                    # connect retry
                    rospy.logdebug('connect retry...')
                    self.__reconnect()
                    time.sleep(0.5)
                except KeyboardInterrupt:
                    self.disconnect()
                    return False
                except Exception as err:
                    rospy.logerr(err)
                    # publish to ros topic signifying connection error occured
                    os.system('rostopic pub -1 /bluetooth_connection std_msgs/Int8 "data: 3"')
                    self.disconnect()
                    return False
            else:
                rospy.loginfo('connect time over threshold. Closing socket...')
                os.system('rostopic pub -1 /bluetooth_connection std_msgs/Int8 "data: 4"')
                raise ShutterConnectionTimeOut('shutter connnect time over')

class TagunovaBluetooth(BLEClientSocket):
    """
    ROS publisher topic:
        /shutter_status std_msgs (String): open, failure
        /bluetooth_connection std_msgs (Int8): 1, 2, 3, 4
    """
    def __init__(self, mac_address: str, port: int):
        super().__init__(mac_address, port, buffer=1024)
        self.connect_start_flag = False

    def __del__(self):
        return super().__del__()

    def clear_queue(self, queue: Queue):
        """Queueにあるデータをすべてクリアする"""
        while not queue.empty():
            res = queue.get_nowait()
            rospy.loginfo(f'Queue data: {res}')

    def recv_opened_shutter(self):
        """開き端をもらったらROSに侵入可を渡す
        (Pass data to ROS from shutter)
        """
        while True:
            # Pass data to ROS
            recv_data = self.receive_massage(self.buffer)
            rospy.logdebug(f'receive data: {recv_data}')
            dict_data = ast.literal_eval(recv_data)
            rospy.logdebug(f"flag: {dict_data['flag']}")
            rospy.loginfo(f"shutter received message: {dict_data['message']}")
            if dict_data['flag']:
                # send shutter open to ROS
                os.system('rostopic pub -1 /shutter_status std_msgs/String "data: open"')
                rospy.loginfo('shutter passing...')
                time.sleep(0.5)
                self.connect_start_flag = False
                break
            else:
                # send shutter failure to ROS
                os.system('rostopic pub -1 /shutter_status std_msgs/String "data: failure"')
                rospy.logerr('ROS published Shutter status "failure"')
                raise ShutterError(f"shutter error message: {dict_data['message']}")

    def recv_passed_ros(self):
        """ROSから通過完了をもらったらシャッターに通過完了を渡す
        (Command from ROS to shutter)
        """
        start_time = time.time()
        limit_time = float(15)
        # wait for Tugnova to pass shutter within time limit
        rospy.loginfo('waiting for passed shutter cmd from ROS')
        while True:
            tim = time.time() - start_time
            if tim < limit_time:
                if not queue_passed_shutter.empty():
                    self.clear_queue(queue_passed_shutter)
                    send_passed_cmd = 'passed shutter'
                    self.send_message(send_passed_cmd)
                    rospy.loginfo(f'send message: {send_passed_cmd}')
                    rospy.loginfo('disconnect bluetooth')
                    self.disconnect()
                    rospy.logdebug('wait connect request for ROS')
                    break
            else:
                raise ROSPassedReceiveTimeOut('No shutter passthrough acknowledgement from Tugnova to shutter')

    def run(self):
        rospy.loginfo('Main: wait connect request from Tugnova')
        while not rospy.is_shutdown():
            # get connection request from ROS
            if not queue_to_connect.empty():
                self.clear_queue(queue_to_connect)
                self.connect_start_flag = True
            try:
                if self.connect_start_flag:
                    if self.connect_bluetooth(timeout=10):
                        send_data = 'open shutter'
                        self.send_message(send_data)
                        rospy.loginfo('waiting for shutter opened...')

                        self.recv_opened_shutter()
                        # send passing shutter cmd from ROS
                        self.recv_passed_ros()
                    else:
                        rospy.logerr('connect NG')
            except KeyboardInterrupt:
                self.disconnect()
                break
            except Exception as err:
                rospy.logerr(err)
                self.disconnect()
            finally:
                self.connect_start_flag = False

    # def client_start(self):
    #     while True:
    #         if not queue_launch_client.empty():
    #             print('Launching bluetooth client')
    #             self.clear_queue(queue_launch_client)
    #             try:
    #                 self.run()
    #             except Exception as err:
    #                 rospy.logerr(err)
    #             finally:
    #                 print('closing bluetooth client')
    #                 #os.system("rosnode kill /bluetooth_client")

class BluetoothClientSubscriber(object):
    """bluetooth connection node subscriber

    topic name:
        /shutter_point (Int8): receive bluetooth connect point from waypoint.csv
        /shutter_passed (Int8): receive shutter passed point from waypoint.csv
    """
    def __init__(self):
        rospy.Subscriber('shutter_point', Int8, self.callbackFromTugnovaShutterNode)
        rospy.Subscriber("shutter_passed", Int8, self.callbackShutterPassed)

    def __del__(self):
        print('shuttdown ros subscriber')

    def callbackFromTugnovaShutterNode(self, msg: Int8):
        rospy.loginfo(f'I received "shutter point reached" signal: {msg.data}')
        # launch bluetooth client
        # queue_launch_client.put({"data":"launch"})
        # sleep to allow client to boot
        # time.sleep(3.0)

        # initiate bluetooth connection request
        queue_to_connect.put({"data":msg.data})

    def callbackShutterPassed(self, msg: Int8):
        rospy.loginfo(f'Tugnova passed through shutter: {msg.data}')
        queue_passed_shutter.put({"data":msg.data})


def main():
    rospy.init_node("bluetooth_client", anonymous=False, log_level=rospy.DEBUG)
    sub = BluetoothClientSubscriber()
    client = TagunovaBluetooth(serverMACaddress, PORT)

    p1 = Process(target=client.run, daemon=True)
    p1.start()
    print("Ros alive: %s\n"%(p1.is_alive()))
    rospy.spin()
    del sub
    p1.join()
    p1.terminate()
    print("Ros alive: %s\n"%(p1.is_alive()))

if __name__=="__main__":
    main()
