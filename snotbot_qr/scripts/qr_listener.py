#!/usr/bin/env python
import rospy
import roslib
import time
roslib.load_manifest('roscopter')
from geometry_msgs.msg import Quaternion
import roscopter.msg
import matplotlib.pyplot as plt

from std_srvs.srv import *
from std_msgs.msg import String, Header, Int32
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy, Imu, NavSatStatus, NavSatFix
from datetime import datetime
from roscopter.srv import APMCommand
from roscopter.msg import VFR_HUD, State

class MCN():
    def __init__(self):
        rospy.init_node('listener', anonymous=False)
        self.axes = []
        self.buttons = []
        self.twist = [0, 0, 0, 0, 1500, 1500, 1500, 1500]
        self.x = 1500.    # Side Tilt
        self.y = 1500.    # Front Tilt
        self.z = 1500.    # Throttle
        self.yaw = 1500.  # Spin
        self.mode = 0

        self.armed = False
        self.failsafe = False
        self.disarm_time = 0.
        self.disarming = False
        self.gps_data = [0., 0., 0.]
        					    # Finite State Machine
        self.launching = True   # 1. Launches to a certain height
        self.searching = False  # 2. Moves forward and searches for fiducial
        self.tracking = False   # 3. Centers itself over fiducial
        
        # Tracks data from the drone
        self.qr_found = False
        self.data = [0., 0., 0., 0.]
        self.old_data = self.data
        self.counter = 0
        self.alt = 0.0
        self.target_alt = 2

        # PID controller - dt = t2 - t1
        self.t1_alt = int(round(time.time() * 1000))
        self.t1_x = int(round(time.time() * 1000))
        self.t1_y = int(round(time.time() * 1000))
        self.t1_z = int(round(time.time() * 1000))
        self.t1_yaw = int(round(time.time() * 1000))

        self.t2_alt = int(round(time.time() * 1000))
        self.t2_x = int(round(time.time() * 1000))
        self.t2_y = int(round(time.time() * 1000))
        self.t2_z = int(round(time.time() * 1000))
        self.t2_yaw = int(round(time.time() * 1000))

        # PID controller - current error values
        self.error_alt = 0.0
        self.error_x = 0.0
        self.error_y = 0.0
        self.error_z = 0.0
        self.error_yaw = 0.0

        # PID controller - cumulative sum values
        self.sum_alt = 0.0
        self.sum_x = 0.0
        self.sum_y = 0.0
        self.sum_z = 0.0
        self.sum_yaw = 0.0

        # ROS publishers and subscribers
        self.qr_data = rospy.Subscriber('qr_data', Quaternion, self.vision_callback)
        self.pub_rc = rospy.Publisher('/apm/send_rc', roscopter.msg.RC)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.sub_state = rospy.Subscriber("/apm/state", State, self.check_state)
        self.sub_height = rospy.Subscriber('/apm/vfr_hud', VFR_HUD, self.parse_action)
        self.command_serv = rospy.ServiceProxy('/apm/command', APMCommand)
        self.platform_gps = rospy.Subscriber('/gps', NavSatFix, self.gps_callback)

        # Flies until the drone is commanded to stop
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.fly()
            r.sleep()

    def check_state(self, data):
        self.armed = data.armed

    def parse_action(self, data):
        self.alt = data.alt

    def gps_callback(self, data):
        self.gps_data = {data.latitude, data.longitude, data.altitude}

    def vision_callback(self, data):
        if int(data.w) == -10:
            self.qr_found = False
        else:
            self.qr_found = True

        self.old_data = self.data
        self.data = [data.x, data.y, data.z, data.w]

    def joy_callback(self, data):
        self.axes = data.axes
        self.buttons = data.buttons

        # Failsafe
        if self.failsafe:
            self.x = 1500 - self.axes[0] * 300     # Scales 1200-1800
            self.y = 1500 - self.axes[1] * 300     # Scales 1200-1800
            self.z = 2000 + self.axes[3] * 1000    # Scales 1000-2000
            self.yaw = 1500 - self.axes[2] * 300   # Scales 1200-1800
        else:
            # Launching
            if self.mode == 1:
                self.launch()

            # Searching
            elif self.mode == 2:
                self.search()

            # Tracking
            elif self.mode == 3:
                self.track()

            # Return to launch pad
            elif self.mode == 4:
                self.rtl()

            # Landing
            elif self.mode == 5:
                self.land()

            # Disarm
            elif self.mode == 6:
                self.disarm()

    def launch(self):
        self.t1_alt = self.t2_alt
        self.t2_alt = int(round(time.time() * 1000))
        dt_alt = (self.t2_alt - self.t1_alt) / 1000.  # Roughly calculates dt

        current_alt = self.alt
        diff_alt = (self.target_alt - current_alt) - self.error_alt
        self.error_alt = self.target_alt - current_alt
        self.sum_alt += self.error_alt * dt_alt
        
        self.z = self.pid(self.error_alt, diff_alt, self.sum_alt, dt_alt, 400, 1500, 0.3, 0.3, 0)
    
    def search(self):
        self.x = 1500 # Temporary values
        self.y = 1600
        self.z = 1500
        self.yaw = 1500

    def track(self):
        self.x = 1500 # Temporary values
        self.y = 1500
        self.z = 1500
        self.yaw = 1500

    def rtl(self):
        self.x = 1500 # Temporary values
        self.y = 1500
        self.z = 1500
        self.yaw = 1500

    def land(self):
        self.x = 1500 # Temporary values
        self.y = 1500
        self.z = 1300
        self.yaw = 1500

    def disarm(self):
        self.x = 1500 # Temporary values
        self.y = 1500
        self.z = 1000
        self.yaw = 1500

    def fly(self):
        if self.buttons:
            # Button 1 - enters failsafe mode (enables full control)
            if self.buttons[0]:
                self.failsafe = True

            # Button 3 - arms the drone
            if self.buttons[2]:
                self.command_serv(roscopter.srv.APMCommandRequest.CMD_SET_ALT_HOLD)
                self.command_serv(roscopter.srv.APMCommandRequest.CMD_ARM)

            # Button 4 - disarms drone
            if self.buttons[3]:
                self.command_serv(roscopter.srv.APMCommandRequest.CMD_DISARM)

            # Button 5 - initiate autonomy routine
            if self.buttons[4]:
                self.mode = 1

            # Button 6 - end autonomy routine (RTL)
            if self.buttons[5]:
                self.mode = 4

        if self.armed:
            (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (int(self.x), int(self.y), int(self.z), int(self.yaw))
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

    def pid(self, error, error_sum, error_dif, dt, control_range, control_mid, K_p, K_i, K_d):
        P = K_p * error
        I = K_i * error_sum
        D = K_d * error_dif / dt

        return (P + I + D) * control_range + control_mid

if __name__ == '__main__':
    print 'process started at ' + str(datetime.now())
    try:
        var = MCN()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass