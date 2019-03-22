import numpy as np
import math
from tf.transformations import euler_from_quaternion
from dub_time import planDubinsPath
import dubins

def velocityObstacle(drone, obs, agent_velocity = 0, obstacle_velocity = 0):
# Agent and Obstacle defined as (x, y, z, radius_of_agent/obstacle)
# velocities defined as (x_vel, y_vel, z_vel)

	agent = np.array([drone[0], drone[1], drone[2]])
	obstacle = np.array([obs[0], obs[1], obs[2]])
	r = drone[3] + obs[3]
	
	# Calculate Tangent Points
	separation = np.linalg.norm(agent - obstacle) 
	if r > separation :
		r = separation
	alpha = math.asin(r/separation)
	gamma = math.pi/2 - alpha
	if(not obstacle[0] - agent[0]):		# To prevent division by zero
		offset = math.pi/2 if (obstacle[1] - agent[1]) > 0 else -math.pi/2
	else:
		offset = math.atan((obstacle[1] - agent[1])/(obstacle[0] - agent[0]))
	theta1 = math.pi - (gamma + offset)
	theta2 = -math.pi + (gamma - offset)
	tp1 = (obstacle[0] + r*math.cos(theta1), obstacle[1] + r*math.sin(theta1))
	tp2 = (obstacle[0] + r*math.cos(theta2), obstacle[1] + r*math.sin(theta2))

	print tp1, tp2


def conversion(x, y, z , drone_odom):
	position_array = np.array([[x, y, z]])

	quaternion = (drone_odom.pose.pose.orientation.x, drone_odom.pose.pose.orientation.y, drone_odom.pose.pose.orientation.z, drone_odom.pose.pose.orientation.w)

	euler = euler_from_quaternion(quaternion)

	alpha = euler[2]

	position_drone = np.array([[drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z]])

	transformation_matrix = np.array([[math.cos(alpha), -1* math.sin(alpha), 0], [math.sin(alpha), math.cos(alpha), 0], [0, 0, 1]])

	new_position = np.matmul(transformation_matrix, np.transpose(position_array)) + np.transpose(position_drone) 
	
	return new_position[0], new_position[1], new_position[2]


# def checkCollision(obstacle, drone_pos, vel_agent, vel_obstacle, timestep, filehandle, drone_odom):
# # obstacle (x, y) is position of obstacle in global frame
# # drone_pos () is position of drone in global frame
# # vel_agent (vx, vy, omega) is X and Y velocity of drone in global frame and omega is about axis of drone
# # vel_obstacle (vx, vy) is vel. of obstacle in global frame frame
# # timestep is dt for projection time
# # drone_odom is odometry of drone in global frame
	
# 	obs_t = [0.0] * len(obstacle[0])
# 	d_min = [100] * len(obs_t)
# 	omega = vel_agent[2]
# 	forward_velocity = math.sqrt(vel_agent[0]**2 + vel_agent[1]**2)
# 	separation = [0.0] * len(obs_t)
# 	for i in range(100):
# 		t = timestep*i
# 		for j in range(len(obs_t)):
# 			obs_t[j] = (obstacle[0][j] + vel_obstacle[0][j]*t, obstacle[1][j] + vel_obstacle[1][j]*t)
		
# 		# Uncomment to log obs(t)
# 		# filehandle.write("obs_x: %f obs_y: %f \t" %(obs_t[0], obs_t[1]))
		
# 		if abs(omega) > 0.2:		# If in circular motion
# 			r = forward_velocity/omega
# 			if vel_agent[1] > 0 :	# Anticlockwise rotation for drone
# 				agent_t = (r*math.sin(omega*t), r*(1-math.cos(omega*t)))
# 			else :
# 				agent_t = (-r*math.sin(omega*t), r*(-1+math.cos(omega*t)))
# 			# Convert projected circle from drone frame into global frame
# 			agent_t = conversion(agent_t[0], agent_t[1], 0, drone_odom)
# 		else :
# 			# Project drone in global frame if moving with only linear vel
# 			agent_t = (vel_agent[0]*t + drone_pos[0], vel_agent[1] * t + drone_pos[1])
# 		for j in range(len(obs_t)):

# 			separation[j] = math.sqrt((agent_t[0] - obs_t[j][0])**2 + (agent_t[1] - obs_t[j][1])**2)
		
# 		# Uncomment to log agent(t) and separation(t)
# 		# filehandle.write("agent_x: %f agent_y: %f \t" %(agent_t[0], agent_t[1]))
# 		# filehandle.write("separation: %f \n" %(separation))
# 		#flag = False
# 		for j in range(len(separation)):
# 			if separation[j] < d_min[j] :
# 				if separation[j] < 1 : # rad_agent + rad_obstacle assumed to be 1m
# 					print "Collision imminent at %f seconds from now" % t
# 					return True
# 					# print "end of projection as collision detected"
# 					# filehandle.write("Collision occurs in projection \n\n\n")
# 					#flag = flag or True
# 					#return True
# 				#if  not flag:
# 				d_min[j] = separation[j]
# 				# print separation[i]

# 	# filehandle.write("end of projection with no collision :D :D :D \n\n\n\n")
# 	#if flag:
# 		#return flag
# 	print "No collision in projected time"
# 	return False



def checkCollision(obstacle, drone_pos, vel_agent, vel_obstacle, timestep, filehandle, drone_odom, curr_vel_flag):
	#time_threshold = 6.0
	print 'Number of obstacles are: ', len(obstacle[0])
	print 'Size of obstacle position: ', np.array(obstacle).shape
	print 'Size of drone_pos: ', np.array(drone_pos).shape
	print 'Size of vel_agent: ', np.array(vel_agent).shape
	print 'Size of vel_obstacle: ', np.array(vel_obstacle).shape
	obs_t = [0.0] * len(obstacle[0])
	d_min = [100] * len(obs_t)
	angular = vel_agent[2]
	forward_velocity = math.sqrt(vel_agent[0]**2 + vel_agent[1]**2)
	separation = [0.0] * len(obs_t)
	#r = abs(forward_velocity/angular)

	## If collision is being checked for potential future velocities
	## If curr_vel_flag is True, that means that collision is being checked for current velocity and not sampled velocity
	if not curr_vel_flag:
		if abs(angular) > 0.08:
			r = abs(forward_velocity/angular)
			if angular < -0.08:
				target = [2 * r, 2 * r + 5.0]
			if angular > 0.08:
				target = [2 * r, -2 * r - 5.0]

			target = conversion(target[0], target[1], 0, drone_odom)
			quaternion = (drone_odom.pose.pose.orientation.x, drone_odom.pose.pose.orientation.y, drone_odom.pose.pose.orientation.z, drone_odom.pose.pose.orientation.w)
			euler = euler_from_quaternion(quaternion)
			#path = planDubinsPath(start_x = drone_pos[1], start_y = drone_pos[0], start_yaw = euler[2], end_x = target[1], end_y = target[0], end_yaw = euler[2], curvature = 1/r)
			path = dubins.shortest_path([-drone_pos[1], drone_pos[0], math.pi/2 + euler[2]], [-target[1], target[0], math.pi/2 + euler[2]], r)
			step_size = 0.01
			config, _ = path.sample_many(step_size)
			path_length = path.path_length()
			seg_length = path.segment_length(0)
			print path_length
			no_iterations = int(path_length/step_size)

			for i in range(no_iterations):
				t = step_size * i
				for j in range(len(obs_t)):
					obs_t[j] = (obstacle[0][j] + vel_obstacle[0][j]*t, obstacle[1][j] + vel_obstacle[1][j]*t)
					separation[j] = math.sqrt((config[i][1] - obs_t[j][0])**2 + (-config[i][0] - obs_t[j][1])**2)
					#print 'Separation[j]: ', separation[j], ' at time t: ', t
				#for j in range(len(obs_t)):
				#print 'Minimum separation: ', min(separation)

				for j in range(len(separation)):
					if separation[j] < d_min[j] :
						
						if separation[j] < 2.0 : # rad_agent + rad_obstacle assumed to be 1m
							print 'Separation[j]: ', separation[j], ' at time t: ', t, ' with omega: ', angular
							print "Collision imminent at %f seconds from now" % t
							return True
							# print "end of projection as collision detected"
							# filehandle.write("Collision occurs in projection \n\n\n")
							#flag = flag or True
							#return True
						#if  not flag:
						d_min[j] = separation[j]
						# print separation[i]

		else:
			for i in range(100):

				t = timestep*i

				for j in range(len(obs_t)):

					obs_t[j] = (obstacle[0][j] + vel_obstacle[0][j]*t, obstacle[1][j] + vel_obstacle[1][j]*t)
					print "Obstacle location at time t = ", t, " is " , obs_t[j]

				agent_t = (vel_agent[0]*t + drone_pos[0], vel_agent[1] * t + drone_pos[1])
				#agent_t = conversion(agent_t[0], agent_t[1], 0, drone_odom)
				print "Drone global position at time t = ", t, " is ", agent_t	

				for j in range(len(obs_t)):

					separation[j] = math.sqrt((agent_t[0] - obs_t[j][0])**2 + (agent_t[1] - obs_t[j][1])**2)

				for j in range(len(separation)):

					if separation[j] < d_min[j] :

						if separation[j] < 2.0 : # rad_agent + rad_obstacle assumed to be 1m
							print 'Separation[j]: ', separation[j], ' at time t: ', t
							print "Collision imminent at %f seconds from now for simulated velocity: " % t

							return True

						d_min[j] = separation[j]


		print "No collision in projected time for omega = ", angular
		return False

	# If collision is being checked for current velocity
	else:
			
			obs_t = [0.0] * len(obstacle[0])
			d_min = [100] * len(obs_t)
			omega = vel_agent[2]
			forward_velocity = math.sqrt(vel_agent[0]**2 + vel_agent[1]**2)
			separation = [0.0] * len(obs_t)
			for i in range(100):
				t = timestep*i
				for j in range(len(obs_t)):
					obs_t[j] = (obstacle[0][j] + vel_obstacle[0][j]*t, obstacle[1][j] + vel_obstacle[1][j]*t)
				
				# Uncomment to log obs(t)
				# filehandle.write("obs_x: %f obs_y: %f \t" %(obs_t[0], obs_t[1]))
				
				if abs(omega) > 0.08:		# If in circular motion
					r = abs(forward_velocity/omega)
					if vel_agent[1] > 0 :	# Anticlockwise rotation for drone
						agent_t = (r*math.sin(omega*t), r*(1-math.cos(omega*t)))
					else:
						agent_t = (-r*math.sin(omega*t), r*(-1+math.cos(omega*t)))
					# Convert projected circle from drone frame into global frame
					agent_t = conversion(agent_t[0], agent_t[1], 0, drone_odom)
				else:
					# Project drone in global frame if moving with only linear vel
					agent_t = (vel_agent[0]*t + drone_pos[0], vel_agent[1] * t + drone_pos[1])
				for j in range(len(obs_t)):

					separation[j] = math.sqrt((agent_t[0] - obs_t[j][0])**2 + (agent_t[1] - obs_t[j][1])**2)
				
				# Uncomment to log agent(t) and separation(t)
				# filehandle.write("agent_x: %f agent_y: %f \t" %(agent_t[0], agent_t[1]))
				# filehandle.write("separation: %f \n" %(separation))
				#flag = False
				for j in range(len(separation)):
					if separation[j] < d_min[j] :
						if separation[j] < 1 : # rad_agent + rad_obstacle assumed to be 1m
							print "Collision imminent at %f seconds from now for current velocity: " % t
							return True
							# print "end of projection as collision detected"
							# filehandle.write("Collision occurs in projection \n\n\n")
							#flag = flag or True
							#return True
						#if  not flag:
						d_min[j] = separation[j]
						# print separation[i]

			# filehandle.write("end of projection with no collision :D :D :D \n\n\n\n")
			#if flag:
				#return flag
			print "No collision in projected time"
			return False

if __name__ == '__main__':
	# velocityObstacle((0,0,0,1), (0,5,0,1))
	checkCollision((4,1), (1,1), (-1,0), 0.1)
