#!/usr/bin/env python3
from queue import Empty
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from communication_msgs.msg import Stats, TugnovaGPS
import socketio
import eventlet
import time
import os
import logging
from eventlet import Queue as GreenQueue
from multiprocessing import Process, Queue



# Variable constants
BIND_IP = ""
PORT = 5600
TAGNOVA_ID = 1
SEND_STATUS_INTERVAL = 5 # send status update every 10 secs
CALLBACK_FREQUENCY = 1 # the publishers in other nodes are publishing at 1 hz
global CURRENT_STATUS
CURRENT_STATUS = -1

# create queue to enable data passing between two processes
queue_to_freedom = Queue()
queue_waitpoint = Queue()
queue_job_status = Queue()
queue_traffic_light = Queue()
queue_magnet_drive = Queue()
queue_drive_mode = Queue()
queue_shutter_error = Queue()
queue_gps_data = Queue()
hotspot_error_data = Queue()
counter_status=[0,"unknown"] # [counter, task_status]

# flags related to processing of Traffic Flags from Freedom
global traffic_light_processed
traffic_light_processed = True
global traffic_light_saver
traffic_light_saver = []
global magnet_mode_task_ended
magnet_mode_task_ended = True
global magnet_message_saver
magnet_message_saver = []

def server():
    global global_logger
    sio = socketio.Server()
    app = socketio.WSGIApp(sio)
    eventlet.monkey_patch()

    @sio.event
    def post_traffic_light_flg(sid, data):
        global traffic_light_processed
        global traffic_light_saver
        if not queue_traffic_light.empty():
            # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "SENDING TRAFFIC LIGHT FLAGS", logObject=global_logger)
            rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] SENDING TRAFFIC LIGHT FLAGS")
            traffic_light_processed = False
            while not traffic_light_processed:
                msg = None
                try:
                    msg = queue_traffic_light.get(timeout=2)
                except:
                    pass
                if msg is not None and msg not in traffic_light_saver:
                    traffic_light_saver.append(msg)
                if traffic_light_processed:
                    break
                for savedMsg in traffic_light_saver:
                    sio.emit("traffic_light_flag", savedMsg)
                    time.sleep(0.5)

            traffic_light_saver.clear()
            sio.emit("stop_traffic_light", {"status": "finish"})
            # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "FINISHED SENDING TRAFFIC LIGHT UPDATES", logObject=global_logger)
            rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] FINISHED SENDING TRAFFIC LIGHT UPDATES")

    @sio.event
    def traffic_light_response(sid,data):
        global traffic_light_processed
        if data["status"]:
            # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "GETTING TRAFFIC LIGHT COMPLETE ", logObject=global_logger)
            rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] GETTING TRAFFIC LIGHT COMPLETE")
            traffic_light_processed = True

    @sio.event
    def cargo_task(sid,data):
        if data["state"]:
            os.system('rostopic pub --once /cargo_task_state std_msgs/Int16 "data: {}"'.format(data["state"]))

    @sio.event
    def send_drive_mode(sid, data):
        if not queue_drive_mode.empty():
            msg = queue_drive_mode.get()
            sio.emit("tugnova_mode_change", msg)
            # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "SENDING " + str(msg), logObject=global_logger)
            rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] SENDING " + str(msg))

    @sio.event
    def magnet_drive_state(sid,data):
        global magnet_mode_task_ended
        global magnet_message_saver
        if not queue_magnet_drive.empty():
            # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "SENDING MAGNETIC UPDATE FLAGS", logObject=global_logger)
            rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] SENDING MAGNETIC UPDATE FLAGS")
            magnet_mode_task_ended = False
            while not magnet_mode_task_ended:
                msg = None
                try:
                    msg = queue_magnet_drive.get(timeout=2)
                except:
                    pass
                if magnet_mode_task_ended:
                    break
                if msg is not None and msg not in magnet_message_saver:
                    magnet_message_saver.append(msg)
                for savedMsg in magnet_message_saver:
                    sio.emit("magnet_drive_status", savedMsg)
                    time.sleep(0.5)

            magnet_message_saver.clear()
            sio.emit("stop_magnet_drive_statust", {"status": "finish"})
            # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "FINISHED SENDING MAGNET UPDATES - CLEARING QUEUE", logObject=global_logger)
            rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] FINISHED SENDING MAGNET UPDATES - CLEARING QUEUE")

    @sio.event
    def magnet_drive_response(sid,data):
        global magnet_mode_task_ended
        if data["status"]:
            # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "GETTING MAGNETIC TAPE TASK COMPLETE ", logObject=global_logger)
            rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] GETTING MAGNETIC TAPE TASK COMPLETE")
            magnet_mode_task_ended = True

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
            # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "RECEIVED WAIT CANCEL FROM FREEDOM ", logObject=global_logger)
            rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] RECEIVED WAIT CANCEL FROM FREEDOM")
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
            sio.emit("waitpoint_update", queue_waitpoint.get())

    @sio.event
    def receive_job(sid, data):
        # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "RECEIVED %s FROM CLIENT" %(data["destination"]), logObject=global_logger)
        rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] RECEIVED %s FROM CLIENT" %(data["destination"]))
        # Publish once to ros node
        cmd = 'rostopic pub --once /server_request std_msgs/String %s'%(data["destination"])
        os.system(cmd)

    @sio.event
    def send_tagnova_stats(sid, data):
        if queue_to_freedom.empty():
            pass
        else:
            stats = queue_to_freedom.get()
            send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "SENDING STATS TO FREEDOM ", logObject=global_logger)
            sio.emit("tagnova_stats", stats)

    @sio.event
    def send_gps_coordinates(sid, data):
        if not queue_gps_data.empty():
            sio.emit("gps_coordinate", queue_gps_data.get())

    @sio.event
    def shutter_error(sid,data):
        if queue_shutter_error.empty():
            pass
        else:
            # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "SHUTTER FAILURE OCURRED", logObject=global_logger)
            rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] SHUTTER FAILURE OCURRED")
            sio.emit("get_shutter_error", queue_shutter_error.get())

    @sio.event
    def send_task_update(sid, data):
        if queue_job_status.empty():
            pass
        else:
            job_status = queue_job_status.get()
            # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "PARSING: ' " + str(job_status) + " '", logObject=global_logger)
            rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] PARSING: ' " + str(job_status) + " '")
            if job_status["job"] == "Accepted":
                # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "SENDING JOB ACCEPTED TO FREEDOM ", logObject=global_logger)
                rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] SENDING JOB ACCEPTED TO FREEDOM")
                sio.emit("job_accepted",job_status)
            elif job_status["job"] == "Completed":
                # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "SENDING JOB COMPLETE TO FREEDOM ", logObject=global_logger)
                rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] SENDING JOB COMPLETE TO FREEDOM")
                sio.emit("job_completed",job_status)
            elif job_status["job"] == "LOW_BATTERY":
                # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "JOB ABORTED DUE TO LOW BATTERY ERROR", logObject=global_logger)
                rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] JOB ABORTED DUE TO LOW BATTERY ERROR")
                sio.emit("job_aborted",job_status)
            elif job_status["job"] == "Invalid_job_name":
                # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "JOB ABORTED DUE TO INVALID COMMAND FROM FREEDOM", logObject=global_logger)
                rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] JOB ABORTED DUE TO INVALID COMMAND FROM FREEDOM")
                sio.emit("job_aborted",job_status)
            else:
                pass

    @sio.event
    def tugnova_hotspot(sid, data):
        hotspot_name = data
        os.system('rostopic pub --once /hotspot_trigger std_msgs/String "data: {}"'.format(hotspot_name))

        while hotspot_error_data.empty():
            time.sleep(1)
        try:
            hotspot_result_msg = hotspot_error_data.get()
            sio.emit("hotspot_answer", hotspot_result_msg)
        except Empty:
            sio.emit("hotspot_answer", {"hotspot_result": 1})

    eventlet.wsgi.server(eventlet.listen((BIND_IP,PORT)),app, socket_timeout=None)

def ros_run():
    global global_logger
    global task
    task = ""

    def stats_update():
        global latitude, longitude, autoware_error_code

        try:
            latitude_to_send = latitude
        except NameError:
            latitude_to_send = 0

        try:
            longitude_to_send = longitude
        except NameError:
            longitude_to_send = 0

        try:
            autoware_error_code_to_send = int(autoware_error_code)
        except Exception:
            autoware_error_code_to_send = -1

        tagnova_stats = {"tagnova_id":TAGNOVA_ID,
                         "status":counter_status[1],
                         "stats":{"position":[x_pos ,y_pos, z_pos],
                                  "speed": speed,
                                  "waypoint_id": waypoint_id,
                                  "current_job":task,
                                  "plc_error":plc_error,
                                  "battery":battery,
                                  "gps_coordinates": [latitude_to_send, longitude_to_send],
                                  "autoware_error": autoware_error_code_to_send}
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

            counter_status[1] = msg.data

    def callbackFromWaitpoint(msg):
        if msg.data:
            rospy.loginfo("wait point reached")
            queue_waitpoint.put({"isWaitpoint": "yes"})

    def callbackFromStats(msg):
        global x_pos, y_pos, z_pos, speed, waypoint_id, plc_error, battery
        # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "RECEIVING STATS FROM TUGNOVA/AW " + str(msg), logObject=global_logger)
        x_pos = msg.position.x
        y_pos = msg.position.y
        z_pos = msg.position.z
        speed = msg.speed
        plc_error = msg.plc_error
        waypoint_id = msg.waypoint_id
        battery = msg.battery_status

        # send info to server.
        # Don't send on every cycle
        if counter_status[0] == (SEND_STATUS_INTERVAL*CALLBACK_FREQUENCY):
            stats_update()
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
        global CURRENT_STATUS
        if msg.data != CURRENT_STATUS:
            queue_drive_mode.put({"mode_flag":msg.data})
            CURRENT_STATUS = msg.data

    def callbackFromGps(msg):
        global latitude, longitude
        latitude = msg.latitude
        longitude = msg.longitude

    def callbackFromAutowareError(msg):
        global autoware_error_code
        autoware_error_code = msg.data

    def callbackFromHotspotResult(msg):
        hotspot_error_data.put({"hotspot_result": int(msg.data)})

    rospy.init_node("hmi_server", anonymous=False)

    rospy.Subscriber("tagnova_response", String, callbackFromJobHandler)

    rospy.Subscriber("server_request", String, callbackFromFreedom)
    rospy.Subscriber("waitpoint_state", Bool, callbackFromWaitpoint)
    rospy.Subscriber("tagnova_stats", Stats, callbackFromStats)
    rospy.Subscriber("shutter_error", Int16, callbackFromShutter)

    #loopback subscription to self
    rospy.Subscriber("waitpoint_clear", String, callbackFromWaitpointClear)
    rospy.Subscriber("traffic_light_flg", Int16, callbackFromTrafficLight)
    rospy.Subscriber("magnet_status", String, callbackFromMagnet)
    rospy.Subscriber("tugnova_drive_mode", Int16, callbackFromDriveMode)
    rospy.Subscriber("preprocessed_fix", TugnovaGPS, callbackFromGps)
    rospy.Subscriber("autoware_status_code", Int16, callbackFromAutowareError)
    rospy.Subscriber("hotspot_error", Int16, callbackFromHotspotResult)

    rospy.spin()


def initialize_logger():
    """
    """

    logger_name = "HMISERVER_TUGNOVA_" + str(TAGNOVA_ID) + ".log"

    formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
    handler = logging.handlers.RotatingFileHandler(logger_name, mode="a", maxBytes=10 * 1024 * 1024, encoding="UTF-8", backupCount=2)
    handler.setFormatter(formatter)

    logger = logging.getLogger(logger_name)
    logger.setLevel(logging.INFO)
    logger.addHandler(handler)

    return logger

def send_to_log(type, robot, context, details, printNormal=True, printLog=True, printNetLog=False, logObject=None):

    global logger
    global telnet_logger

    try:
        typeText = "[" + str(type) + "]"
        robotText = "[ROBOT " + str(robot) + "]"
        contextText = "[" + "\"" + str(context) + "\"" + "]"
        detailsText = "[" + "\"" + str(details) + "\"" + "]"
    except:
        return

    fullStr = "," + typeText + "," + robotText + "," + contextText + "," + detailsText

    if printNormal:
        print(fullStr)
    if logObject is not None:
        logObject.info(fullStr)

if __name__=="__main__":
    global global_logger
    global_logger = initialize_logger()
    p1 = Process(target=ros_run)
    p1.start()
    # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "INIT", "Ros alive: %s\n"%(p1.is_alive()), logObject=global_logger)
    rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] Ros alive: %s\n"%(p1.is_alive()))
    # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXEC", "Starting Server", logObject=global_logger)
    rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] Starting Server")
    server()
    p1.join()
    p1.terminate()
    # send_to_log("INFO", "TUGNOVA_"+str(TAGNOVA_ID), "EXIT", "Ros alive: %s\n"%(p1.is_alive()), logObject=global_logger)
    rospy.loginfo("[INFO] [TUGNOVA_"+str(TAGNOVA_ID) + "] [EXEC] Ros alive: %s\n"%(p1.is_alive()))
