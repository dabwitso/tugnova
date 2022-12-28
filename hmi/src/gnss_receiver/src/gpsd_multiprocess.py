#! /usr/bin/env python3

from multiprocessing import Process, Queue
from threading import Lock
import rospy
import gps
from gps_common.msg import GPSFix
from gnss_msgs.msg import TugnovaGPS

NaN = float('nan')
coordinateQ = Queue()

class gpsdClient(Process):

    def __init__(self, mode):

        Process.__init__(self, daemon=True)
        self.mode = self.mode_str_to_obj(mode)
        
        if self.mode != None:
            self.client = gps
            self.session = gps.gps(mode=self.mode)
        else:
            raise Exception('Mode is Not Valid')

        self.altitude = NaN
        self.latitude = self.longitude = 0.0
        self.speed = NaN
        self.time = NaN
        self.coord_dictionary = dict()

    def run(self):
        
        while True:

            try:
                self.read_gnss_receiver()
            
            except KeyboardInterrupt:
                print('Breaking GPSD CLIENT')
                break
    
    def mode_str_to_obj(self, mode):
        
        if mode == 'enable':
            return gps.WATCH_ENABLE
        elif mode == 'disable':
            return gps.WATCH_DISABLE
        elif mode == 'json':
            return gps.WATCH_JSON
        elif mode == 'nmea':
            return gps.WATCH_NMEA
        elif mode == 'rare':
            return gps.WATCH_RARE
        elif mode == 'raw':
            return gps.WATCH_RAW
        else:
            return None

    def read_gnss_receiver(self):
        
        while 0 == self.session.read():
            
            if not (gps.MODE_SET and self.session.valid):
                continue

            self.update_time()
            self.update_coordinates()
            self.update_json()
            self.update_q()

    def update_time(self):
        
        if self.client.TIME_SET and self.session.valid:
            self.time = self.session.fix.time
        else:
            self.time = NaN

    def update_coordinates(self):

        if (self.check_if_finite(self.session.fix.latitude) and
            self.check_if_finite(self.session.fix.longitude)):
            self.latitude = self.session.fix.latitude
            self.longitude = self.session.fix.longitude
        else:
            self.latitude = NaN
            self.longitude = NaN
    
    def update_speed(self, value):

        if self.check_if_finite(self.session.fix.speed):
            self.speed = value
        else:
            self.speed = NaN

    def check_if_finite(self, fix_value):

        try:
            return self.client.isfinite(fix_value)
        except:
            return False

    def update_json(self):

        self.coord_dictionary['time'] = self.time
        self.coord_dictionary['latitude'] = self.latitude
        self.coord_dictionary['longitude'] = self.longitude

    def update_q(self):

        if coordinateQ.empty():
            coordinateQ.put(self.coord_dictionary)


class rosNode(Process):

    def __init__(self, node_name):

        Process.__init__(self, daemon=True)
        rospy.init_node(node_name)
        self.pub = rospy.Publisher('/gnss_signal', TugnovaGPS, queue_size=10)
        self.rate = rospy.Rate(1/3)
        self.counter = 0

    def run(self):
        
        while not rospy.is_shutdown():

            try:

                self.check_queue()

            except KeyboardInterrupt:
                print('Breaking ROS NODE CHECKING Q')
                break
                
    def check_queue(self):

        if not coordinateQ.empty():

            try:
                coordinates = coordinateQ.get()
                gpsMessage = TugnovaGPS()
                gpsMessage.header.frame_id = "gps"
                gpsMessage.header.stamp = rospy.Time.now()
                latitude = coordinates['latitude']
                longitude = coordinates['longitude']
                print(latitude, type(latitude), longitude, type(longitude))
                gpsMessage.latitude = float(31.002123124124)
                gpsMessage.longitude = float(127.0012312312)
                print(gpsMessage)
                self.pub.publish(gpsMessage)
                print('Published ======' + str(self.counter))
                self.counter += 1
                coordinateQ.empty()
                self.rate.sleep()
                
            except:
                print("Passing")
                pass


if __name__ == '__main__':

    gps_process = gpsdClient('enable')
    node_process = rosNode('gps_receiver')
    gps_process.start()
    print("GPS CLIENT ALIVE: %s\n" %(gps_process.is_alive()))
    node_process.start()
    print("ROS NODE ALIVE: %s\n" %(node_process.is_alive()))
    gps_process.join()
    node_process.join()
    gps_process.terminate()
    node_process.terminate()
    gps_process.session.close()
    exit(0)

