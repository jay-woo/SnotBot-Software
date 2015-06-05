#Ocean Alliance and Olin College SnotBot

Adapted from the repository done in conjunction with Army Research Labs for the 2014-2015 Olin SCOPE project with liaisons Brendan Byrne and Jason Gregory and team members Kyle McConnaughay, Charles Goddard, Heather Boortz, Elizabeth Duncan, Eric Schneider,and Kaitlin Gallagher.

This adaptation is created by the SnotBot Research Team at Olin College.

Advisor: Andrew Bennett
Team Members: Victoria Preston, Jay Woo, Riley Chapman, James Wang, Devynn Diggins, and Shivali Chandra.

SnotBot is an autonomous drone to be used by marine scientists during research voyages as a data collection aide.  The drone can be given missions by a human researcher, can perform autonomous missions, can be controlled by a human operator, and collect whale breath condensate.


# Launching the Repo
In order to run this repo, you will need to download the following packages
		- ROS from Willow Garage for the distribution of Ubuntu you are running
		- A workspace for ROS (like catkin)
		- OpenCV
		- Roscopter from https://github.com/vpreston/roscopter (tutorial here:https://github.com/epsilonorion/roscopter/wiki)
		- Joystick drivers for your ROS/Ubuntu distribution (these links may be a useful resource: http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick and http://linuxcommand.org/lts0070.php)
		- geodesy (sudo apt-get install ros-hydro-geodesy for your version of ROS)

# Running Missions
In order to run the brain structure, launch roscore, launch roscopter, and rosrun quadcopter_brain current_mission.py _outside:=False (or True depending on your actual state of outside-ness).
