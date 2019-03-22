#!/usr/bin/env python

# Load python Libraries 
from __future__ import print_function
import time, threading, random
import math
import numpy as np

# Load ROS Libraries
import rospy
import roslib
import message_filters
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

# Load ROS Messages
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from apriltags_ros.msg import AprilTagDetection, AprilTagDetectionArray
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState

# Load custom modules
from dub_time import planDubinsPath
from planner import Planner, Grid
# from drone_velCon import Drone 	# Use this for emulated VELOCITY CONTROL (Ayush Gaud Matlab converted code)
from drone import Drone 		# Use this for Position control with Lee Trajecotory Controller

# Main function
def main():
	# Intiate the ROS node
	rospy.init_node('execute_motion', anonymous=False)
	
	# Drone control topics
	trajectory_handle = rospy.Publisher('/bebop2/command/trajectory', MultiDOFJointTrajectory, queue_size=10)

	# Path publish topics
	path_handle = rospy.Publisher('/path', Path, queue_size = 2)

	# Velocity publish topics
	velocity_pub = rospy.Publisher('/bebop2/cmd_vel', Twist, queue_size = 5)

	# Gazebo publish handle definition
	gaz = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)
	
	# Load drone model
	Bebop = Drone(
				  forward_velocity = 1.0,
				  angular_velocity = 0.8,
				  trajectory_handle = trajectory_handle,
				  path_handle = path_handle,
				  velocity_handle = velocity_pub
				  )

	# Planner parameters (All in meters)
	planner = Planner(
					  min_safe_distance = 5,
					  window_size = (0.5, 0.5),     # (x, y)
					  grid_size = (10, 10), 		# (x, y)
					  obstacle_radius = 2,
					  goal = (20, 0, math.radians(0)), # (x, y, heading)
					  alternategoal = [(15, 5, math.radians(0)), (20, 5, math.radians(90))],
					  error_margin = 0.1,
					  drone = Bebop,
					  gaz_handle = gaz
					  )

	# Subscribe to required topics
	tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, planner.detectAprilTags)
	odometry_sub = rospy.Subscriber('/bebop2/odometry_sensor1/odometry', Odometry, Bebop.updateOdometry)
	dynamic_obs = rospy.Subscriber('/safe_velocities', Twist, planner.dynamicObstacle)
	
	# Thread to run the planner
	print('Starting planner in 2 seconds...')
	time.sleep(2)
	planner_thread = threading.Thread(target = planner.run)
	planner_thread.start()

	try:
		rospy.spin()
	except Exception as e:
		print('Shutting Down!')

if __name__ == '__main__':
	main()
