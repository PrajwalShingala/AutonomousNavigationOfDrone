#!/usr/bin/env python

#include python libs
import time
import numpy as np
#include custom libs
from dub_time import *
#include ros libs
import rospy
#include rosmsgs
from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

#generate ros trajectory message from data
def generateTrajectoryMessage(px, py, pyaw, time_traj):
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

		traj.points = temp_points[80:]
		path.poses = temp_poses[80:]
		return traj, path
	
def init_node():

	pub = rospy.Publisher('/bebop2/command/trajectory', MultiDOFJointTrajectory, queue_size=10)

	path_handle = rospy.Publisher('/path', Path, queue_size = 2)

	file = open("log_path.txt", "w")

	rospy.init_node('trajectory_test_dubin', anonymous=False)

	print "Initializing node ..."

	time.sleep(0.5)

	px, py, pyaw = planDubinsPath(3.217,-1.13,2.011,5,0,math.radians(0),0.8)  #(sy, sx, syaw, ey, ex, eyaw, c)

	print len(px)

	t = generateTimestamp(px, py, pyaw, 1, 1)

	# for i in range(len(pyaw)):
	# 	print("px, py ", px[i], py[i], "yaw: ", pyaw[i], "	timestamp: ", t[i])
	for i in range(len(pyaw)):
		file.write("x: %f y: %f yaw: %f time: %f \n" %(px[i], py[i], pyaw[i],t[i]))

	trajectory, path = generateTrajectoryMessage(px, py, pyaw, t)

	# 	pub.publish(trajectory)

	path_handle.publish(path)

	file.close()

if __name__ == '__main__':
	try:
		init_node()
	except rospy.ROSInterruptException:
		pass 