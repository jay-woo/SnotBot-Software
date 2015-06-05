#!/usr/bin/env python

import rospy

from quadcopter import Quadcopter
from quadcopter_brain import QuadcopterBrain
from rc_command import RCCommand


def backward(q):
    rc_command = RCCommand({'pitch': 0.9})
    q.send_rc_command(rc_command)


def forward(q):
    rc_command = RCCommand({'pitch': 0.1})
    q.send_rc_command(rc_command)


def right(q):
    rc_command = RCCommand({'roll': 0.9})
    q.send_rc_command(rc_command)


def left(q):
    rc_command = RCCommand({'roll': 0.1})
    q.send_rc_command(rc_command)


def still(q):
    rc_command = RCCommand()
    q.send_rc_command(rc_command)


def throttle_up(q):
    rc_command = RCCommand({"throttle": 0.8})
    q.send_rc_command(rc_command)


def throttle_down(q):
    rc_command = RCCommand({"throttle": 0.2})
    q.send_rc_command(rc_command)


def rc_square_dance():
    import time

    q = Quadcopter()
    outside = rospy.get_param("Quadcopter/outside", False)
    if outside:
        q.arm()
    q.launch()

    print "forward"
    forward(q)
    time.sleep(2)
    print "still"
    still(q)
    time.sleep(2)

    print "right"
    right(q)
    time.sleep(2)
    print "still"
    still(q)
    time.sleep(2)

    print "backward"
    backward(q)
    time.sleep(2)
    print "still"
    still(q)
    time.sleep(2)

    print "left"
    left(q)
    time.sleep(2)
    print "still"
    still(q)
    time.sleep(2)

    throttle_down(q)
    time.sleep(20)


def print_position_data(quadcopter):
    rospy.loginfo("Position data:")
    rospy.loginfo("\tLatitude: %.8f" % quadcopter.current_lat)
    rospy.loginfo("\tLongitude: %.8f" % quadcopter.current_long)
    rospy.loginfo("\tRelative Altitude: %.2f" % quadcopter.current_rel_alt)
    rospy.loginfo("\tAltitude: %.2f" % quadcopter.current_alt)
    rospy.loginfo("\tHeading: %.2f" % quadcopter.heading)


def main():
    frizzle = QuadcopterBrain()

    # Quadcopter node (carl) must be initialized before get_param will work
    outside = rospy.get_param("Quadcopter/outside", False)
    rospy.loginfo("In outside mode: %s.", outside)
    rospy.loginfo("If incorrect, add _outside:=True to the rosrun call")

    frizzle.quadcopter.clear_waypoints()
    rospy.loginfo("Sleeping for 3 seconds...")
    rospy.sleep(3)

    #To change waypoints, change the file shown below
    mission_waypoints = WaypointTools.open_waypoint_file(
        "oval_waypoints.json")

    if outside:
        frizzle.quadcopter.arm()
    
    #If mission file has all of the points of interest, use:
    #for point in mission_waypoints:
    #   mission.append(point)
    #frizzle.fly_path(mission)
    frizzle.fly_path([mission_waypoints["A"],
                      mission_waypoints["B"]])


if __name__ == '__main__':
    main()
    #For debugging, use the following: 
    # rc_square_dance()