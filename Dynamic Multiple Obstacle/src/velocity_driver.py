#!/usr/bin/env python
import numpy as np
from transforms3d import quaternions

import rospy
import message_filters

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

pub = rospy.Publisher("/bebop2/command/trajectory", MultiDOFJointTrajectory, queue_size = 5)

error_prev = 0
time_previous = 0
integralError = 0
controllerInitialised = False

def getReferenceVelocityFromPID(setpoint, fb_vel, kp = 4, ki = 2, kd = -0.5):
	# Declare global variables
	global error_prev
	global time_previous
	global integralError
	global controllerInitialised
	
	error_cur = setpoint - fb_vel
	delE = (error_cur - error_prev)
	
	if not controllerInitialised :
		controllerInitialised = True
		time_previous = rospy.get_time()
		return 0
	else:
		dt = rospy.get_time() - time_previous
		if not dt :			# To prevent division by zero
			print("no velocity as dT = zero")
			return 0
		if error_cur*error_prev < 0 :		# When error changes direction
			pass
		else :
			integralError = integralError + error_cur*dt
		print ("Integral Error: ", integralError)
		# Update prev_time and prev_error
		error_prev = error_cur
		time_previous = rospy.get_time()

		# print ("prop_Vel", kp*error_cur)
		# print ("diff_vel", kd*delE/dt)
		# print ("integral_vel", ki*integralError)
		print ("setPoint, feedback velocity", setpoint, fb_vel)
		print ("reference velocity", kp*error_cur + ki*integralError+ kd*delE/dt)
		return kp*error_cur + ki*integralError + kd*delE/dt
	

def callback(odom, vel):
	# vel.linear.x = getReferenceVelocityFromPID(vel.linear.x, odom.twist.twist.linear.x, 4.5, 2, -0.01)
	# vel.angular.z = getReferenceVelocityFromPID(vel.angular.z, odom.twist.twist.angular.z)

	dt = 0.6
	commandPose = MultiDOFJointTrajectory()
	commandPose.header.stamp = rospy.Time.now()
	commandPose.header.frame_id = "world"
	commandPose.joint_names.append("drone")

	point = MultiDOFJointTrajectoryPoint()

	tf_msg = Transform()

	quat = (
	odom.pose.pose.orientation.w,
	odom.pose.pose.orientation.x,
	odom.pose.pose.orientation.y,
	odom.pose.pose.orientation.z)

	# Rotation matrix from quaternions of Drone Odom
	rotationMatrix = quaternions.quat2mat(quat)
	# print rotationMatrix
	# Rotation Translation Matrix for the drone from Odometry
	RTM = np.zeros((4,4))										
	RTM[0:3,0:3] = rotationMatrix
	RTM[ : ,3] = (odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, 1)
	# print RTM
	# print("Initial Pose: ", odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z)
	# Pose change w.r.t local drone frame
	delPose  = (vel.linear.x*dt, vel.linear.y*dt, vel.linear.z*dt, 1)
	# print "local pose: ", delPose

	# Transform to get global command pose (x, y, z)
	globalCommandPose = (np.matmul(RTM, delPose)).tolist()
	# print "Global Target Pose: ", globalCommandPose
	tf_msg.translation.x = globalCommandPose[0]
	tf_msg.translation.y = globalCommandPose[1]
	tf_msg.translation.z = 1 #globalCommandPose[2]

	RPY = list(euler_from_quaternion((odom.pose.pose.orientation.x,
							    odom.pose.pose.orientation.y,
							    odom.pose.pose.orientation.z,
							    odom.pose.pose.orientation.w)))
	# print RPY

	RPY[2] = RPY[2] + vel.angular.z*dt
	quat = quaternion_from_euler(RPY[0], RPY[1], RPY[2])
	# print quat
	tf_msg.rotation.x = quat[0]
	tf_msg.rotation.y = quat[1]
	tf_msg.rotation.z = quat[2]
	tf_msg.rotation.w = quat[3]

	point.transforms.append(tf_msg)

	commandPose.points.append(point)
	pub.publish(commandPose)

def main():

	rospy.init_node('velocity_driver', anonymous = False)

	odom_sub = message_filters.Subscriber("/bebop2/odometry_sensor1/odometry", Odometry)

	cmd_vel_sub = message_filters.Subscriber("/bebop2/cmd_vel", Twist)

	ts = message_filters.ApproximateTimeSynchronizer([odom_sub, cmd_vel_sub], 10, 0.1, allow_headerless=True)
	
	ts.registerCallback(callback)

	rospy.spin()

if __name__ == '__main__':
	main()