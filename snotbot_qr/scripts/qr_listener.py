#!/usr/bin/env python
import rospy
import roslib
import time
roslib.load_manifest('roscopter')
from geometry_msgs.msg import Quaternion
import roscopter.msg
import matplotlib.pyplot as plt
import math
import gps_tools

from std_srvs.srv import *
from std_msgs.msg import String, Header, Int32
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy, Imu, NavSatStatus, NavSatFix
from datetime import datetime
from roscopter.srv import APMCommand
from roscopter.msg import VFR_HUD, State, Attitude

class MCN():
    def __init__(self):

        #########################
        # DRONE STATE VARIABLES #
        #########################

        # Twist variables
        self.axes = []
        self.buttons = []
        self.twist = [0, 0, 0, 0, 1500, 1500, 1500, 1500]
        self.x = 1500.       # Side Tilt
        self.y = 1500.       # Front Tilt
        self.z = 1500.       # Throttle
        self.yaw = 1500.     # Spin
        self.tilt = 1000.    # Camera tilt

        # Other ROS subscriber variables
        self.drone_gps = gps_tools.Point(0, 0, 0)
        self.platform_gps = gps_tools.Point(0, 0, 0)
        self.alt = 0
        self.bearing = 0
        self.climb = 0.
        self.mode = 0
        self.armed = False

        # Various failsafe variables
        self.failsafe = False
        self.arm_time = 0       # Arming
        self.althold_time = 0   # Launching
        self.land_time = 0      # Landing
        self.disarm_time = 0    # Disarming


        ######################
        # QR STATE VARIABLES #
        ######################
        
        self.qr_found = False
        self.data = [0., 0., 0., 0.]


        #########################
        # PID CONTROLLER VALUES #
        #########################

        self.current_state = {'x': 0, 'y': 0, 'z': 0, 'yaw': 0}      # Current state
        self.target_state = {'x': 0, 'y': 0, 'z': 5.0, 'yaw': 0}     # Target state
        self.error = {'x': 0, 'y': 0, 'z': 0, 'yaw': 0}              # Error state (target - current)
        self.error_prev = {'x': 0, 'y': 0, 'z': 0, 'yaw': 0}         # Previous error
        self.error_sum = {'x': 0, 'y': 0, 'z': 0, 'yaw': 0}          # Error cumulative sum
        self.t1 = {'x': 0, 'y': 0, 'z': 0, 'yaw': 0}                 # Time of previous calculation
        self.t2 = {'x': 0, 'y': 0, 'z': 0, 'yaw': 0}                 # Time of current calculation

        self.K_p = {'x': 0, 'y': 0, 'z': 0.3, 'yaw': 0.2}            # Proportional control
        self.K_i = {'x': 0, 'y': 0, 'z': 0.6, 'yaw': 0.4}            # Integral control
        self.K_d = {'x': 0, 'y': 0, 'z': 0  , 'yaw': 0  }            # Differential control


        ######################
        # ROS INITIALIZATION #
        ######################

        # Initializing ROS...
        rospy.init_node('listener', anonymous=False)

        # Ground station nodes
        self.qr_data = rospy.Subscriber('qr_data', Quaternion, self.vision_callback)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.joy_callback)

        # Landing platform nodes
        self.platform_gps = rospy.Subscriber('/gps', NavSatFix, self.platform_gps_callback)

        # Drone nodes
        self.pub_rc = rospy.Publisher('/apm/send_rc', roscopter.msg.RC)
        self.sub_attitude = rospy.Subscriber("/apm/attitude", Attitude, self.attitude_callback)
        self.sub_state = rospy.Subscriber("/apm/state", State, self.check_state)
        self.sub_height = rospy.Subscriber('/apm/vfr_hud', VFR_HUD, self.parse_action)
        self.drone_gps = rospy.Subscriber('/apm/gps', NavSatFix, self.drone_gps_callback)
        self.command_serv = rospy.ServiceProxy('/apm/command', APMCommand)

        # Flies until the drone is commanded to stop
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.fly()
            r.sleep()


    ###################
    # ROS SUBSCRIBERS #
    ###################

    # Obtains camera data from drone
    def vision_callback(self, data):
        if int(data.w) == -10:
            self.qr_found = False
        else:
            self.qr_found = True

        self.data = [data.x, data.y, data.z, data.w]

    # Enables joystick control, sends RC commands to drone
    def joy_callback(self, data):
        self.axes = data.axes
        self.buttons = data.buttons

        # Failsafe
        if self.failsafe:
            self.x = 1500 - self.axes[0] * 300     # Scales 1200-1800
            self.y = 1500 - self.axes[1] * 300     # Scales 1200-1800
            self.z = 2000 + self.axes[3] * 1000    # Scales 1000-2000
            self.yaw = 1500 - self.axes[2] * 300   # Scales 1200-1800

    # Gets GPS coordinates of drone
    def drone_gps_callback(self, data):
        (self.drone_gps.x, self.drone_gps.y, self.drone_gps.z) = (data.latitude, data.longitude, data.altitude)

    # Gets GPS coordinates of landing platform
    def platform_gps_callback(self, data):
        (self.platform_gps.x, self.platform_gps.y, self.platform_gps.z) = (data.latitude, data.longitude, data.altitude)

    # Gets certain state variables from the drone
    def check_state(self, data):
        self.armed = data.armed
        self.flight_mode = data.mode

    # Gets drone's attitude information
    def attitude_callback(self, data):
        self.current_state['yaw'] = data.yaw

    # Gets miscellaneous information from drone
    def parse_action(self, data):
        self.current_state['z'] = data.alt
        self.climb = data.climb


    ########################
    # FINITE STATE MACHINE #
    ########################

    # Mode 1 - arms the drone
    def arm(self):
        self.command_serv(roscopter.srv.APMCommandRequest.CMD_SET_LOITER)
        self.command_serv(roscopter.srv.APMCommandRequest.CMD_ARM)
        self.mode = 2
        self.arm_time = millis()

    # Mode 2 - launches drone until it reaches a certain altitude
    def launch(self):
        if millis() - self.arm_time > 5000:
            # If the drone maintains the target altitude for a few seconds, go to the next mode
            if abs(current_alt - self.target_state['z']) > 1.0:
                self.z = self.pid('z')
                self.althold_time = millis()
            else:
                if millis() - self.althold_time > 3000:
                    self.t2['z'] = 0
                    self.mode = 6
    
    # Mode 3 - move drone forwards until it sees a fiducial
    def search(self):
        # 1. Point gimbal forwards
        # 2. Use vision to track an object
        # 3. Adjust gimbal and yaw to keep object in vision

        self.x = 1500 # Temporary values
        self.y = 1500
        self.z = 1500
        self.yaw = 1500

    # Mode 4 - center drone over fiducial
    def center(self):
        self.x = 1500 # Temporary values
        self.y = 1500
        self.z = 1500
        self.yaw = 1500

    # Mode 5 - return to landing platform
    def rtl(self):
        self.target_state['yaw'] = gps_tools.bearing(self.drone_gps, self.platform_gps)
        error = self.target_state['yaw'] - self.current_state['yaw']

        # Turns the drone towards the landing platform then moves it forwards
        if abs(error) > 0.25:
            self.y = 1700
        else:
            self.y = 1500

        if gps_tools.distance(self.drone_gps, self.platform_gps) > 0.03 
            self.yaw = self.pid('yaw')
        else:
            self.t2['yaw'] = 0
            self.mode = 6

    # Mode 6 - land on platform w/ visual servoing
    def land(self):
        self.x = 1500 # Temporary values
        self.y = 1500
        self.z = 1300
        self.yaw = 1500

    # Mode 7 - disarm once landing sequence is complete
    def disarm(self):
        self.x = 1500 # Temporary values
        self.y = 1500
        self.z = 1000
        self.yaw = 1500

        if millis() - self.disarm_time < 5000:
            self.mode = 0
            self.command_serv(roscopter.srv.APMCommandRequest.CMD_DISARM)

    # Publishes RC commands to the vehicle
    def fly(self):
        if self.buttons:
            # Button 1 - enters failsafe mode (enables full control)
            if self.buttons[0]:
                self.failsafe = True

            # Button 3 - arms the drone
            if self.buttons[2]:
                self.switch_mode("LOITER")
                self.command_serv(roscopter.srv.APMCommandRequest.CMD_SET_LOITER)
                self.command_serv(roscopter.srv.APMCommandRequest.CMD_ARM)

            # Button 4 - disarms drone
            if self.buttons[3]:
                self.mode = 6
                self.disarm_time = millis()
                self.switch_mode("LOITER")
                self.command_serv(roscopter.srv.APMCommandRequest.CMD_SET_LOITER)

            # Button 5 - initiate autonomy routine
            if self.buttons[4]:
                self.mode = 1
                
            # Button 6 - end autonomy routine (RTL)
            if self.buttons[5]:
                self.mode = 4

        # Initiates finite state machines
        if not self.failsafe:
            # Arming
            if self.mode == 1:
                self.arm()

            # Launching
            if self.mode == 2:
                self.launch()

            # Searching
            elif self.mode == 3:
                self.search()

            # Tracking
            elif self.mode == 4:
                self.center()

            # Return to launch
            elif self.mode == 5:
                self.rtl()

            # Landing
            elif self.mode == 6:
                self.land()

            # Disarm
            elif self.mode == 7:
                self.disarm()

        # Publishes commands
        if self.armed:
            (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (int(self.x), int(self.y), int(self.z), int(self.yaw))
            self.twist[5] = self.tilt
            self.pub_rc.publish(self.twist)
            
    def track_object(self):
        error_x = 250 - self.data[0]
        error_y = 200 - self.data[1]
        error_z = 100 - self.data[2]
        error_yaw = -self.data[3]

        pid_x = self.pid(error_x, self.sum_x, 1400, 1600, 1, 1, 1)
        pid_y = self.pid(error_y, self.sum_y, 1400, 1600, 1, 1, 1)
        pid_z = self.pid(error_z, self.sum_z, 1400, 1600, 1, 1, 1)
        pid_yaw = self.pid(error_yaw, self.sum_yaw, 1400, 1600, 1, 1, 1)

        return (pid_x, pid_y, pid_z, pid_yaw)

    def pid(self, variable_name):
        # If pid is running for the first time, start with tiny dt
        if self.t2[variable_name] == 0:
            self.t1[variable_name] = millis()
            self.t2[variable_name] = millis() + 0.0001 # Prevent division by zero
        else:
            self.t1[variable_name] = t2[variable_name]
            self.t2[variable_name] = millis()
        dt = (self.t2[variable_name] - self.t1[variable_name]) / 1000

        # Calculates the error, error cumulative sum, and error difference
        self.error_prev [variable_name] =  self.error [variable_name]
        self.error      [variable_name] =  self.target_state [variable_name] - self.current_state [variable_name]
        self.error_sum  [variable_name] += self.error [variable_name] * dt
        self.error_dif  [variable_name] =  self.error [variable_name] - self.error_prev [variable_name]

        # Calculates PID
        P = K_p [variable_name] * self.error     [variable_name]
        I = K_i [variable_name] * self.error_sum [variable_name]
        D = K_d [variable_name] * self.error_dif [variable_name] / dt
        PID = P + I + D

        # Returns the PID result, scaled to the correct control range
        return PID * 400 + 1500


def millis():
    return int(round(time.time() * 1000))

if __name__ == '__main__':
    print 'process started at ' + str(datetime.now())
    try:
        var = MCN()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass