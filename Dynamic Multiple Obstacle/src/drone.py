#!/usr/bin/env python
from __future__ import print_function

#import python libraries
import sys
import math

#import ros libraries
import roslib
import rospy
import time

#import ros messages and functions
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry


class Drone(object):
	"""docstring for [object Object]."""
	def __init__(self,
				 forward_velocity,
				 angular_velocity,
				 trajectory_handle,
				 velocity_handle,
				 path_handle
				 ):
		self.forward_velocity = forward_velocity
		self.angular_velocity = angular_velocity
		self.trajectory_handle = trajectory_handle
		self.path_handle = path_handle
		self.x = 0
		self.y = 0
		self.yaw = 0
		self.velocity = None
		self.done = False
		self.odom = None
		self.original_forward_velocity = forward_velocity
		self.original_angular_velocity = angular_velocity

	def generateTimestamp(self, px, py, pyaw, v, w):
		timestamp = []
		timestamp.append(0)
		for i in range(1,len(px)):
			if pyaw[i] < 0 and 4.6 < pyaw[i-1] < 4.8 :		#entering negative yaw from +3pi/2 = 4.7099
				dtheta = pyaw[i] + 2*math.pi - pyaw[i-1]
			elif pyaw[i] > 0 and pyaw[i-1] < -1.5 :			#exiting negative yaw from -pi/2 = -1.57
				dtheta = -pyaw[i] + 2*math.pi + pyaw[i-1]
			else :
				dtheta = pyaw[i] - pyaw[i-1]

			if(dtheta):
				dt = abs(dtheta)/w
			else:
				dx = math.sqrt((px[i] - px[i-1])**2 + (py[i] - py[i-1])**2)
				dt = dx/v 
			timestamp.append(timestamp[i-1]+dt)
		return timestamp

	#generate ros trajectory message from data
	def generateTrajectoryMessage(self, px, py, pyaw, time_traj):
		traj = MultiDOFJointTrajectory()
		path = Path()

		temp_poses = []
		temp_points = []

		# path.header.stamp = rospy.Time.now() 
		path.header.frame_id = "world"

		traj.header.stamp = rospy.Time.now()
		traj.header.frame_id = "world"
		traj.joint_names.append("drone")

		for i in range(len(px)):
			point = MultiDOFJointTrajectoryPoint()

			tf_msg = Transform()
			tf_msg.translation.x = px[i]
			tf_msg.translation.y = py[i]
			tf_msg.translation.z = 1 		#altitude of Flight

			quaternion = quaternion_from_euler(0, 0, pyaw[i])
			tf_msg.rotation.x = quaternion[0]
			tf_msg.rotation.y = quaternion[1]
			tf_msg.rotation.z = quaternion[2]
			tf_msg.rotation.w = quaternion[3]

			pose = PoseStamped()
			pose.header.stamp = rospy.Time.now()
			pose.header.frame_id = "world"
			pose.pose.position.x = px[i]
			pose.pose.position.y = py[i]
			pose.pose.position.z = 1
			pose.pose.orientation.x = quaternion[0]
			pose.pose.orientation.y = quaternion[1]
			pose.pose.orientation.z = quaternion[2]
			pose.pose.orientation.w = quaternion[3]

			
			# vel_msg = Twist()
			# vel_msg = self.velocity
			# point.velocities.append(vel_msg)

			# if pyaw[i] == pyaw[i-1] :
			# 	vel_msg.angular.z = 0
			# elif pyaw[i] > pyaw[i-1] :
			# 	vel_msg.angular.z = self.angular_velocity
			# elif pyaw[i] < pyaw[i-1] :
			# 	vel_msg.angular.z = -self.angular_velocity

			# acc_msg = Twist()
			# acc_msg.linear.x = 0

			point.transforms.append(tf_msg)			
			# point.accelerations.append(acc_msg)
			
			point.time_from_start = rospy.Duration.from_sec(time_traj[i]) #+ rospy.Duration(5)

			temp_points.append(point)
			temp_poses.append(pose)

		traj.points = temp_points[70:]
		path.poses = temp_poses[70:]
		return traj, path
	
	def dubinsMoveDrone(self, px, py, pyaw):

		#print("odom and time before publish :", self.x, self.y, self.yaw, rospy.Time.now())
		
		t = self.generateTimestamp(px, py, pyaw, self.forward_velocity, self.angular_velocity)

		## resetting forward and angular velocity
		self.forward_velocity = self.original_forward_velocity
		self.angular_velocity = self.original_angular_velocity

		trajectory, path = self.generateTrajectoryMessage(px, py, pyaw, t)

		self.trajectory_handle.publish(trajectory)

		self.path_handle.publish(path)
		# time.sleep(3)
		#print("Trajectory generated and published")
		
	def updateOdometry(self, odom):

		self.x = odom.pose.pose.position.x
		self.y = odom.pose.pose.position.y
		quaternion = (
					  odom.pose.pose.orientation.x,
					  odom.pose.pose.orientation.y,
					  odom.pose.pose.orientation.z,
					  odom.pose.pose.orientation.w
					 )
		euler = euler_from_quaternion(quaternion)
		self.yaw = euler[2]
		self.velocity = odom.twist.twist
		self.odom = odom
