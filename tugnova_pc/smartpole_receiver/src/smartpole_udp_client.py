#! /usr/bin/env python
import socket
from socketserver import UDPServer
import rospy
from multiprocessing import Process, Queue, ProcessError, Lock
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

class SmartPoleUDPClient(Process):

    def __init__(self, ip, port):
        Process.__init__(self, daemon=True)
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def run(self):

        self.sock.bind(self.ip, self.port)

        while True:

            data, addr = self.sock.recvfrom(1024)

            try:
                infoLock.acquire()
                infoQ.put(data)
            except Exception:
                pass
            finally:
                infoLock.release()

            if KeyboardInterrupt:
                break


class RosNode(Process):

    def __init__(self, node_name):
        Process.__init__(self, daemon=True)
        rospy.init_node(node_name)
        self.pub = rospy.Publisher("/mgrs_text", String, queue_size=10)

    def run(self):

        while not rospy.is_shutdown():

            try:
                if infoQ.empty():
                    data = infoQ.get()
                    message_to_send = String()
                    message_to_send.data = data
                    self.pub(message_to_send)

            except KeyboardInterrupt:
                break

            except Exception as e:
                print("Exception was " + str(e))
                pass

            finally:
                infoLock.release()

    def convert_coordinates(self):
        pass


if __name__ == "__main__":
    infoQ = Queue()
    infoLock = Lock()
    udpClient = SmartPoleUDPClient(UDP_IP, UDP_PORT)
    rosNode = RosNode("smartpole_client")
    udpClient.start()
    print("UDP Client alive : %s\n".format(udpClient.is_alive()))
    rosNode.start()
    print("ROS Node alive : %s\n".format(udpClient.is_alive()))
    udpClient.join()
    rosNode.join()
    udpClient.terminate()
    print("UDP Client alive : %s\n".format(udpClient.is_alive()))
    rosNode.terminate()
    print("ROS Node alive : %s\n".format(udpClient.is_alive()))
    exit(0)