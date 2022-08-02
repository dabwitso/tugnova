import rospy
from marionet.msg import MarionetteMessage
import socketio
import time
from multiprocessing import Process, Queue

# Variable constants
SERVER_IP = "http://192.168.0.20"
PORT = 5005
TAGNOVA_ID = 1
SEND_STATUS_INTERVAL = 10 # send status update every 10 secs
CALLBACK_FREQUENCY = 1 # the publishers in other nodes are publishing at 1 hz

# create queue to enable data passing between two processes
queue_to_marionette = Queue()
counter_status=[0] # [counter, task_status]

def ros_run():
    def stats_update():
        status_to_marionette = {"tagnova_id":TAGNOVA_ID,
                                "stats":{"plc_status":plc_status,
                                         "battery": battery}
                                }

        if queue_to_marionette.empty():
            queue_to_marionette.put(status_to_marionette)
        else:
            queue_to_marionette.get() # remove previous data
            queue_to_marionette.put(status_to_marionette)

    def callbackFromStats(msg):
        global plc_status, battery

        plc_status = msg.plc_status
        battery = msg.battery

        #rospy.loginfo("PLC status: " + plc_status + " Battery: " + battery+" %")

        if counter_status[0] == (SEND_STATUS_INTERVAL*CALLBACK_FREQUENCY):
            stats_update()
            counter_status[0] = 0
        counter_status[0] += 1


    rospy.init_node("marionette_publisher", anonymous=False)
    rospy.Subscriber("marionette_status", MarionetteMessage, callbackFromStats)

    rospy.spin()

def publisher():

    sio=socketio.Client()

    @sio.event
    def connect():
        print('connection established')

    @sio.event
    def connect_error(data):
        print('The connection failed!')

    @sio.event
    def disconnect():
        print('disconected from server')

    @sio.on('*')
    def cath_all(event, data):
        print("something happened")

    server_connect = SERVER_IP+":"+str(PORT)
    sio.connect(server_connect, namespaces=['/HMIMarionette'])

    while (sio.connected): #CHECK CONDITION

        # print("Enter sending loop")
        sio.emit("client2server",queue_to_marionette.get(), namespace='/HMIMarionette')
        time.sleep(1)

if __name__=="__main__":
    interval_counter = 0
    p1 = Process(target=ros_run)
    p1.start()
    print("Ros alive: %s\n"%(p1.is_alive()))
    publisher()
    p1.join()
    p1.terminate()
    print("Ros alive: %s\n"%(p1.is_alive()))
