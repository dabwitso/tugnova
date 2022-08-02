#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from communication_msgs.msg import Stats
import socketio
import eventlet
import time
import os
from eventlet import Queue as GreenQueue
from multiprocessing import Process, Queue



# Variable constants
BIND_IP = "";
PORT = 5600;
TAGNOVA_ID = 1;
SEND_STATUS_INTERVAL = 10 # send status update every 10 secs
CALLBACK_FREQUENCY = 1 # the publishers in other nodes are publishing at 1 hz

# create queue to enable data passing between two processes
queue_to_freedom = Queue()
queue_waitpoint = Queue()
queue_job_status = Queue()
queue_traffic_light = Queue()
queue_magnet_drive = Queue()
queue_drive_mode = Queue()
queue_shutter_error = Queue()
counter_status=[0,"unknown"] # [counter, task_status]


def server():
    sio = socketio.Server()
    app = socketio.WSGIApp(sio)
    eventlet.monkey_patch()

    @sio.event
    def post_traffic_light_flg(sid, data):
        if not queue_traffic_light.empty():
            sio.emit("traffic_light_flag",queue_traffic_light.get())

    @sio.event
    def cargo_task(sid,data):
        if data["state"]:
            os.system('rostopic pub --once /cargo_task_state std_msgs/Int16 "data: {}"'.format(data["state"]))

    @sio.event
    def traffic_light_response(sid,data):
        # if true, start tugnova drive immediately
        if data["status"]:
            os.system('rostopic pub --once /GpioStartFlg std_msgs/Int16 "data: 1"')

    @sio.event
    def send_drive_mode(sid, data):
        if not queue_drive_mode.empty():
            msg = queue_drive_mode.get()
            sio.emit("tugnova_mode_change", msg)

    @sio.event
    def magnet_drive_state(sid,data):
        if queue_magnet_drive.empty():
            pass
        else:
            msg = queue_magnet_drive.get()
            print("PRINT MAGNET UPDATE WITH " + str(msg))
            sio.emit("magnet_drive_status", msg)

    @sio.event
    def get_command(sid, data):
        os.system(data["command"])

    @sio.event
    def start_drive(sid,data):
        if data["command"] == "start":
            os.system('rostopic pub --once /GpioStartFlg std_msgs/Int16 "data: 1"')
            time.sleep(2)
            for i in range(2):
                os.system('rostopic pub --once /waitpoint_clear std_msgs/String "data: clear"')
                time.sleep(2)

    @sio.event
    def clear_waitpoint(sid, data):
        if data["command"] == "clear":
            os.system('rostopic pub --once /GpioStartFlg std_msgs/Int16 "data: 1"')
            time.sleep(2)
            for i in range(2):
                os.system('rostopic pub --once /waitpoint_clear std_msgs/String "data: clear"')
                time.sleep(2)

    @sio.event
    def send_waitpoint_update(sid, data):
        if queue_waitpoint.empty():
            pass
        else:
            sio.emit("waitpoint_update",queue_waitpoint.get())

    @sio.event
    def receive_job(sid, data):
        print("I received %s from client" %(data["destination"]))
        # Publish once to ros node
        cmd = 'rostopic pub --once /server_request std_msgs/String %s'%(data["destination"])
        os.system(cmd)
    @sio.event
    def send_tagnova_stats(sid, data):
        if queue_to_freedom.empty():
            pass
        else:
            print("Sending stats to Freedom")
            sio.emit("tagnova_stats",queue_to_freedom.get())

    @sio.event
    def shutter_error(sid,data):
        if queue_shutter_error.empty():
            pass
        else:
            print("Shutter failure occured")
            sio.emit("get_shutter_error", queue_shutter_error.get())

    @sio.event
    def send_task_update(sid, data):
        if queue_job_status.empty():
            pass
        else:
            job_status = queue_job_status.get()
            rospy.loginfo("parsing: %s",job_status)
            if job_status["job"] == "Accepted":
                print("Sending job_accepted to Freedom")
                sio.emit("job_accepted",job_status)
            elif job_status["job"] == "Completed":
                print("Sending job_completed to Freedom")
                sio.emit("job_completed",job_status)
            elif job_status["job"] == "LOW_BATTERY":
                print("Job aborted due to low battery error")
                sio.emit("job_aborted",job_status)
            elif job_status["job"] == "Invalid_job_name":
                print("Job aborted due to Invalid job command from Freedom")
                sio.emit("job_aborted",job_status)
            else:
                pass



    eventlet.wsgi.server(eventlet.listen((BIND_IP,PORT)),app, socket_timeout=None)

def ros_run():
    global task
    global previous_mode
    previous_mode = 100
    task = ""
    def stats_update():
        tagnova_stats = {"tagnova_id":TAGNOVA_ID,
                         "status":counter_status[1],
                         "stats":{"position":[x_pos ,y_pos, z_pos],
                                  "speed": speed,
                                  "waypoint_id": waypoint_id,
                                  "current_job":task,
                                  "plc_error":plc_error,
                                  "battery":battery}
                         }

        if queue_to_freedom.empty():
            queue_to_freedom.put(tagnova_stats)
        else:
            queue_to_freedom.get() # remove previous data
            queue_to_freedom.put(tagnova_stats)

    def callbackFromWaitpointClear(msg):
        rospy.loginfo("Waitpoint status: %s"%(msg.data))
        rospy.loginfo("Tagnova will start moving soon")

    def callbackFromFreedom(msg):
        rospy.loginfo("I received %s command from Freedom"%(msg.data))
        task = msg.data
        queue_job_status.put({"job":"Accepted"})

    def callbackFromJobHandler(msg):
        if msg.data:
            data = msg.data
            if queue_job_status.empty():
                queue_job_status.put({"job":data})
            else:
                queue_job_status.get() # clear old data
                queue_job_status.put({"job":data})

            counter_status[1] = msg.data;


    def callbackFromWaitpoint(msg):
        if msg.data:
            rospy.loginfo("wait point reached");
            queue_waitpoint.put({"isWaitpoint": "yes"});

    def callbackFromStats(msg):
        global x_pos, y_pos, z_pos, speed, waypoint_id, plc_error, battery;
        x_pos = msg.position.x;
        y_pos = msg.position.y;
        z_pos = msg.position.z;
        speed = msg.speed;
        plc_error = msg.plc_error;
        waypoint_id = msg.waypoint_id;
        battery = msg.battery_status;

        # send info to server.
        # Don't send on every cycle
        if counter_status[0] == (SEND_STATUS_INTERVAL*CALLBACK_FREQUENCY):
            stats_update();
            counter_status[0] = 0
        counter_status[0] += 1
    def callbackFromMagnet(msg):
        if msg.data:
            queue_magnet_drive.put({"magnet_drive":msg.data})

    def callbackFromTrafficLight(msg):
        if msg.data:
            queue_traffic_light.put({"flag":msg.data})

    def callbackFromShutter(msg):
        if msg.data == 1:
            queue_shutter_error.put({"error":"failure"})

    def callbackFromDriveMode(msg):
        if msg.data != previous_mode:
            queue_drive_mode.put({"mode_flag":msg.data})
            previous_mode = msg.data

    rospy.init_node("hmi_server", anonymous=False)

    rospy.Subscriber("tagnova_response", String, callbackFromJobHandler);

    rospy.Subscriber("server_request", String, callbackFromFreedom)
    rospy.Subscriber("waitpoint_state", Bool, callbackFromWaitpoint)
    rospy.Subscriber("tagnova_stats", Stats, callbackFromStats);
    rospy.Subscriber("shutter_error", Int16, callbackFromShutter);

    #loopback subscription to self
    rospy.Subscriber("waitpoint_clear", String, callbackFromWaitpointClear)
    rospy.Subscriber("traffic_light_flg", Int16, callbackFromTrafficLight)
    rospy.Subscriber("magnet_status", String, callbackFromMagnet)
    rospy.Subscriber("tugnova_drive_mode", Int16, callbackFromDriveMode)

    rospy.spin()

if __name__=="__main__":
    p1 = Process(target=ros_run)
    p1.start()
    print("Ros alive: %s\n"%(p1.is_alive()))
    server()
    p1.join()
    p1.terminate()
    print("Ros alive: %s\n"%(p1.is_alive()))
