#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header, Bool
from std_srvs.srv import Empty
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
import roscopter.msg
import roscopter.srv
import sys,struct,time,os
import math
import driver


##******************************************************************************
 # Name:    start_mission
 # Purpose: Callback function for "mission" Service.  Specific commands are used
 #              to control functions such as Start mission, finish mission and 
 #              failsafe.
 #              All commands may be found within the "XCEECommand" Service file.
 #              New commands should be entered there to keep uniform constants
 #              throughout calling functions
 # Params:  data: Requested command variable
#*******************************************************************************
def start_mission(req):
    print req
    driver.goto_waypoint()
    return True

def land_it(req):
    driver.land()
    return []

def adjust_throttle(req):
    driver.adjust_throttle()
    return []

def trigger_auto(req):
    driver.trigger_auto()
    return []

def clear_waypoints(req):
    driver.clear_waypoints()
    return []

def num_of_waypoints(req):
    driver.num_of_waypoints()
    return []

##******************************************************************************
# Services for APM Commands
#*******************************************************************************
# Allow for commands such as Arm, Disarm, Launch, Land, etc.
rospy.Service("mission", roscopter.srv.APMCommand, start_mission)
rospy.Service("land", Empty, land_it)
rospy.Service("adjust_throttle", Empty, adjust_throttle)
rospy.Service("trigger_auto", Empty, trigger_auto)
rospy.Service("clear_waypoints", Empty, clear_waypoints)
rospy.Service("num_of_waypoints", Empty, num_of_waypoints)


if __name__ == '__main__':
    try:
        # initially clear waypoints and start mainloop
        driver.clear_waypoints()
#        if (opts.enable_ros_failsafe):
#            rospy.Timer(rospy.Duration(1), ros_failsafe_check)        
        driver.mainloop()
    except rospy.ROSInterruptException: pass