#!/usr/bin/env python

# Load system modules
from __future__ import print_function
import sys, math, threading, time, signal
import numpy as np

# Load ROS modules
import roslib, rospy
roslib.load_manifest('navigator')
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from apriltags_ros.msg import AprilTagDetection, AprilTagDetectionArray

# Load custom modules
from planner import RosPlanner as Planner
from drone import BebopController as Drone

# Main function
def main(args):
	# Intiate the ROS node
	rospy.init_node('navigator', anonymous=True)
	
	# Drone control topics
	vel_cmd_pub = rospy.Publisher('/vservo/cmd_vel', Twist, queue_size=3)
	takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=3)
	land = rospy.Publisher('/bebop/land', Empty, queue_size=3)
	
	# Load drone model
	drone = Drone(vel_cmd_pub)

	# Planner parameters (All in meters)
	goal = np.array([10.0, 0.0])
	planner = Planner(safe_dist=6.0, goal=goal, drone=drone)

	# Subscribe to required topics
	tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, planner.obstacle_tracker)
	odometry_sub = rospy.Subscriber('/bebop/odom', Odometry, planner.odometry)

	# Thread to run the planner
	print('Starting planner in 5 seconds...')
	time.sleep(5)
	planner_thread = threading.Thread(target=planner.run)
	planner_thread.start()

	def signal_handler(sig, frame):
		planner.stop = True
		print('You pressed Ctrl+C! Stopping planner')
		sys.exit(0)

	signal.signal(signal.SIGINT, signal_handler)
	
	try:
		rospy.spin()
	except Exception as e:
		print('Shutting Down!')

if __name__ == '__main__':
	main(sys.argv)