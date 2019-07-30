#!/usr/bin/env python
# Test playing around with DJI M100

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from enum import Enum
from dji_sdk.srv import *

class flight_status_enum(Enum):
    M100_STATUS_ON_GROUND = 1
    M100_STATUS_TAKINGOFF = 2 
    M100_STATUS_IN_AIR = 3
    M100_STATUS_LANDING = 4 
    M100_STATUS_FINISHED_LANDING = 5

flight_status = flight_status_enum.M100_STATUS_ON_GROUND
display_mode = UInt8()
current_atti = QuaternionStamped()
current_local_pos = PointStamped()
current_gps = NavSatFix()
gps_ready = False 
command = Joy()

def obtain_control():
    rospy.wait_for_service("dji_sdk/sdk_control_authority")
    try:
        sdk_ctrl_authority_service = rospy.ServiceProxy("dji_sdk/sdk_control_authority", SDKControlAuthority)
        ret = sdk_ctrl_authority_service(1)
        if(ret.result):
            return True
        else:
            rospy.logerr("obtain control failed!") 
            return False
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False


def is_M100():
    rospy.wait_for_service("dji_sdk/query_drone_version")
    try:
        query_version_service = rospy.ServiceProxy("dji_sdk/query_drone_version", QueryDroneVersion)
        ret = query_version_service()
        # print(ret)
        if(ret.hardware=="M100"):
            return True
        else:
            rospy.logerr("Hardware not M100!") 
            return False
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False


def attitude_callback(msg):
    global current_atti
    current_atti = msg.quaternion

def local_position_callback(msg):
    global current_local_pos
    current_local_pos = msg.point

def gps_callback(msg):
    global current_gps
    global gps_ready
    current_gps = msg
    gps_ready = True 

def flight_status_callback(msg):
    global flight_status
    flight_status = flight_status_enum(msg.data)

def display_mode_callback(msg):
    global display_mode
    display_mode = msg.data

def takeoff_land(task):
    rospy.wait_for_service("dji_sdk/drone_task_control")
    try:
        drone_task_service = rospy.ServiceProxy("dji_sdk/drone_task_control", DroneTaskControl)
        ret = drone_task_service(task)
        # print(ret)
        if(ret.result):
            return True
        else:
            rospy.logerr("takeoff_land failed!") 
            return False
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

def M100monitoredTakeoff():
    global current_gps
    global start_time
    start_time = rospy.Time.now()
    home_altitude = current_gps.altitude

    if(not takeoff_land(DroneTaskControlRequest.TASK_TAKEOFF)):
        return False
    
    rospy.sleep(0.01)

    # If M100 is not in the air after 10 seconds, fail.
    while ((rospy.Time.now() - start_time) < rospy.Duration(15)):
        rospy.sleep(0.01)
        print flight_status
        print current_gps.altitude - home_altitude

    if(flight_status != flight_status_enum.M100_STATUS_IN_AIR or current_gps.altitude - home_altitude < 0.8):
        rospy.logerr("Takeoff failed.")
        return False
    else:
        start_time = rospy.Time.now()
        rospy.loginfo("Successful takeoff!")

    return True

def M100monitoredLand():
    global current_gps
    global start_time
    start_time = rospy.Time.now()

    if(not takeoff_land(DroneTaskControlRequest.TASK_LAND)):
        return False
    
    rospy.sleep(1)

    while(flight_status != flight_status_enum.M100_STATUS_FINISHED_LANDING):
        rospy.sleep(0.01)
        if((rospy.Time.now() - start_time) > rospy.Duration(30)):
            rospy.logerr("Landing failed.")
            return False
    
    rospy.loginfo("Successful landing.")
    return True    

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def talker():
    global flight_status
    rospy.init_node('dji_controller', anonymous=True)
    
    # Publisher
    # Publish the control signal
    ctrlVelYawPub = rospy.Publisher("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", Joy, queue_size=1)
    # Subscribers
    attitudeSub     = rospy.Subscriber("dji_sdk/attitude", QuaternionStamped, attitude_callback)
    gpsSub          = rospy.Subscriber("dji_sdk/gps_position", NavSatFix, gps_callback)
    flightStatusSub = rospy.Subscriber("dji_sdk/flight_status", UInt8, flight_status_callback)
    displayModeSub  = rospy.Subscriber("dji_sdk/display_mode", UInt8, display_mode_callback)
    localPosition   = rospy.Subscriber("dji_sdk/local_position", PointStamped, local_position_callback)

    # # Basic services
    # sdk_ctrl_authority_service = rospy.ServiceProxy("dji_sdk/sdk_control_authority", SDKControlAuthority)
    # drone_task_service         = rospy.ServiceProxy("dji_sdk/drone_task_control", DroneTaskControl)
    # query_version_service      = rospy.ServiceProxy("dji_sdk/query_drone_version", QueryDroneVersion)
    # set_local_pos_reference    = rospy.ServiceProxy("dji_sdk/set_local_pos_ref", SetLocalPosRef)


    rate = rospy.Rate(10) # 10hz
    while(not gps_ready):	### TODO: really check actual GPS
        rospy.sleep(0.01)
    print("STARTED")
    ret = is_M100()
    print ret
    if not ret:
    	rospy.logerr("Drone is not M100!")
    	return
    ret = obtain_control()
    print ret
    if not ret:
    	rospy.logerr("Cannot obtain control!")
    	return
    print "taking off..."
    ret = M100monitoredTakeoff()
    if not ret:
    	rospy.logerr("Cannot takeoff!")
    	return
    print "done"
    rospy.sleep(1)
    print "landing"
    ret = M100monitoredLand()
    print ret
    if not ret:
    	rospy.logerr("Cannot land!")
    	return
    print "done"


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass