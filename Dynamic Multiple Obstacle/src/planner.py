#!/usr/bin/env python

# Import system modules
import time, math, threading, random
import dubins

# Import python modules
import numpy as np
from matplotlib import pyplot as plt

# Import custom modules
from dub_time import planDubinsPath
from drone import Drone
from tf.transformations import euler_from_quaternion
import math

# Msg to trigger moving obstacle
from gazebo_msgs.msg import ModelState
msg = ModelState()
msg.model_name = 'id37'
msg.pose.position.x = 30 #20 # 15	
msg.pose.position.y = 0
msg.pose.position.z = 0.5
msg.pose.orientation.z = 0
msg.pose.orientation.w = 1
msg.twist.linear.y = 0
msg.twist.linear.x = -1.0

msg_1 = ModelState()
msg_1.model_name = 'id38'
msg_1.pose.position.x = 22.5 #10 # 15
msg_1.pose.position.y = -2 #10 
msg_1.pose.position.z = 500
msg_1.pose.orientation.z = 0
msg_1.pose.orientation.w = 1
msg_1.twist.linear.x = -1.0#-0.5
msg_1.twist.linear.y = 0.0 #-1.0


msg_2 = ModelState()
msg_2.model_name = 'id39'
msg_2.pose.position.x = 22.5 #10
msg_2.pose.position.y = 2	#-10	
msg_2.pose.position.z = 500
msg_2.pose.orientation.z = 0
msg_2.pose.orientation.w = 1
msg_2.twist.linear.x = -1.0
msg_2.twist.linear.y = 0.0 #1.0

msg_3 = ModelState()
msg_3.model_name = 'id35'
msg_3.pose.position.x = 15 #10
msg_3.pose.position.y = 15	#-10	
msg_3.pose.position.z = 500
msg_3.pose.orientation.z = 0
msg_3.pose.orientation.w = 1
msg_3.twist.linear.x = 0.0
msg_3.twist.linear.y = -1.0 #1.0

msg_4 = ModelState()
msg_4.model_name = 'id36'
msg_4.pose.position.x = 15 #10
msg_4.pose.position.y = -15	#-10	
msg_4.pose.position.z = 500
msg_4.pose.orientation.z = 0
msg_4.pose.orientation.w = 1
msg_4.twist.linear.x = 0.0
msg_4.twist.linear.y = 1.0 #1.0


class Planner():
	"""
		- Generate a grid with all detected tags on it.
		- Detect free space on the grid and get waypoint in x wrt to drone.
		- If waypoint is 0 continue.
		- Else replan.
	"""
	def __init__(self, 
				min_safe_distance,
				window_size,
				grid_size,
				obstacle_radius,
				goal,
				alternategoal,
				error_margin,
				drone,
				gaz_handle
				):
		self.min_safe_distance = min_safe_distance
		self.goal = goal
		self.alternategoal = alternategoal
		self.error_margin = error_margin
		self.drone = drone
		self.window_size = window_size
		self.grid = Grid(np.array(grid_size), obstacle_radius, min_safe_distance)
		self.control_point = 0
		self.next_waypoint = None
		self.move = False
		self.initialized = False
		self.tag_detections = None
		self.count = 0
		self.plan_end_x = None
		self.plan_end_y = None
		self.plan_end_yaw = None
		self.gaz_handle = gaz_handle
		self.safe_vel = None

	def resetSearch(self):
		self.control_point = 0

	def conversion(self, x, y, z , drone_odom):
		position_array = np.array([[x, y, z]])

		quaternion = (drone_odom.pose.pose.orientation.x, drone_odom.pose.pose.orientation.y, drone_odom.pose.pose.orientation.z, drone_odom.pose.pose.orientation.w)

		euler = euler_from_quaternion(quaternion)

		alpha = euler[2]

		position_drone = np.array([[drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z]])

		transformation_matrix = np.array([[math.cos(alpha), -1* math.sin(alpha), 0], [math.sin(alpha), math.cos(alpha), 0], [0, 0, 1]])

		new_position = np.matmul(transformation_matrix, np.transpose(position_array)) + np.transpose(position_drone) 
		
		return new_position[0], new_position[1]

	def maneuver_dubins(self, init_point, new_point, turning_radius, drone_pos, drone_new_pos, vx_new, vy_new, new_heading):

		length_ratio = 1000;

		while True:
			## normal distance
			drone_dis = np.linalg.norm(np.array(drone_new_pos) - np.array(drone_pos))
			
			## Forming dubing path
			path = dubins.shortest_path(init_point, new_point, turning_radius)
		    
			## Dubins path length
			dubins_length = path.path_length()
		    
			length_ratio = dubins_length/drone_dis
			#print(length_ratio)
		    
			if length_ratio > 1.2:
				drone_new_pos = [drone_new_pos[0] + 0.01*vy_new, drone_new_pos[1] + 0.01*vy_new]
				new_point = [drone_new_pos[0], drone_new_pos[1], new_heading]
			else:
				return new_point, path;

	# Plan dubins curve and move drone along it
	def run(self):
		if not self.initialized:
			# Plan initial trajectory
			px, py, pyaw, seglens, mode = planDubinsPath(
										  start_y = 0,
										  start_x = 0,
										  start_yaw = math.radians(0),
										  end_y = self.goal[0],
										  end_x = self.goal[1],
										  end_yaw = self.goal[2],
										  curvature = self.drone.angular_velocity/self.drone.forward_velocity
										 )
			self.last_plan = time.time()
			self.move = True
			self.initialized = True
			self.plan_end_x = px[-1]
			self.plan_end_y = py[-1]
			self.plan_end_yaw = pyaw[-1]
			#print('Initialized with Global Plan', mode)

			# Uncomment to use Position Control
			self.gaz_handle.publish(msg)
			time.sleep(0.005)
			self.gaz_handle.publish(msg_1)
			time.sleep(0.005)
			self.gaz_handle.publish(msg_2)
			time.sleep(0.005)
			self.gaz_handle.publish(msg_3)
			time.sleep(0.005)
			self.gaz_handle.publish(msg_4)			# time.sleep(3)
			self.drone.dubinsMoveDrone(px, py, pyaw)

			# time.sleep(0.005)
			# self.gaz_handle.publish(msg_3)
			# time.sleep(0.005)
			# self.gaz_handle.publish(msg_4)
			# Uncomment to use emulated velocity control
			# self.drone.thread = threading.Thread(target = self.drone.dubinsMoveDrone, args = [mode, seglens, px, py, pyaw])
			# self.drone.thread.start()

		while self.move:
			# print "Inside while loop!"
			if self.planExecuted(self.plan_end_x, self.plan_end_y, self.plan_end_yaw):
				#print ("In region of End of plan")
				if (abs(self.plan_end_x - self.goal[0]) < 0.05 and abs(self.plan_end_y - self.goal[1]) < 0.05 and abs(self.plan_end_yaw - self.goal[2]) < 0.05) :
					self.move = False
					self.drone.kill_thread = True
					break
				self.drone.done = True

			# if self.safe_vel:   

   #              		linear = self.drone.forward_velocity
   #             			angular = self.safe_vel.angular.z
   #              		self.safe_vel = None

   #              		target = self.conversion(0, 2*linear/angular, 0, self.drone.odom)   #

   #              		print "Planning with safe velocity"
   #              		px, py, pyaw, seglens, mode = planDubinsPath(
   #                                            start_y = self.drone.x,
   #                                            start_x = self.drone.y,
   #                                            start_yaw = self.drone.yaw,
   #                                            end_y = target[0],
   #                                            end_x = target[1],
   #                                            end_yaw = self.drone.yaw + math.pi,
   #                                            curvature = self.drone.angular_velocity/self.drone.forward_velocity
   #                                           )
   #              # To simulate fixed time execution of evasiveManeuver
   #              		print len(px)
   #              		px = px[0:170]
   #             		        py = py[0:170]
   #             			pyaw = pyaw[0:170]

   #             			self.plan_end_x = px[-1]
   #              		self.plan_end_y = py[-1]
   #              		self.plan_end_yaw = pyaw[-1]
   #              		self.drone.dubinsMoveDrone(px, py, pyaw)


################################ Uncomment this if not running cone thing! #########################################
		  #   	if self.safe_vel:
			 #    	linear = self.drone.forward_velocity
			 #    	angular = self.safe_vel.angular.z
			 #    	self.safe_vel = None

			 #    	x = 2 * abs(linear/angular)
			 #    	print angular
			 #    	if angular < 0:
			 #    		y = abs(2 * linear/angular) + 5
			 #    	else:
			 #    		y = -1*(2 * linear/angular + 5)

				# 	target = self.conversion(x, y, 0, self.drone.odom)   #
				# 	print(target)
					
				# 	print "Planning with safe velocity"
				# 	px, py, pyaw, seglens, mode = planDubinsPath(
				# 		              start_y = self.drone.x,
				# 		              start_x = self.drone.y,
				# 		              start_yaw = self.drone.yaw,
				# 		              end_y = target[0],
				# 		              end_x = target[1],
				# 		              end_yaw = self.drone.yaw,
				# 		              curvature = self.drone.angular_velocity/self.drone.forward_velocity)
				# # To simulate fixed time execution of evasiveManeuver
				# 		#print len(px)
				# 		#px = px[0:170]
			 #       		     #   py = py[0:170]
			 #       			#pyaw = pyaw[0:170]
				# 	self.plan_end_x = px[-1]
				# 	self.plan_end_y = py[-1]
				# 	self.plan_end_yaw = pyaw[-1]
				# 	self.drone.dubinsMoveDrone(px, py, pyaw)
#######################################################################################################################

			if self.safe_vel:
				# print "safe vel published"
				vx_old = self.safe_vel.linear.x
				vy_old = self.safe_vel.linear.y
				vx_new = self.safe_vel.angular.x
				vy_new = self.safe_vel.angular.y;
				vel_flag = self.safe_vel.angular.z;
				# print 'Vel flag: ',vel_flag
				if vel_flag == -5 and (abs(vx_new) > 0.1 or abs(vy_new) > 0.1):
					print "Vel flag 0. Safe to publish."
					self.safe_vel = None
					## let's get headings
					
					## Old heading
					old_heading = self.drone.yaw

					## getting new heading
					if vx_new == 0:
						new_heading = math.pi/2
					else:
						new_heading = np.arctan2(vy_new,vx_new)

					## Defining dubins configuration
					# print "Difference between new heading and old heading is: ", new_heading - old_heading
					print "Old heading, new_heading, vx_old, vy_old, vx_new, vy_new: ", old_heading, new_heading, vx_old, vy_old, vx_new, vy_new
					drone_pos = [self.drone.x, self.drone.y]
					drone_new_pos = [self.drone.x + 5.0*vx_new, self.drone.y + 5.0*vy_new]
					turning_radius = 1.0
					init_point = [drone_pos[0], drone_pos[1], old_heading]
					new_point1 = [drone_new_pos[0], drone_new_pos[1], new_heading]

					new_point2, _ = self.maneuver_dubins(init_point, new_point1, turning_radius, drone_pos, drone_new_pos, vx_new, vy_new, new_heading) 

					drone_dis1 = np.linalg.norm(np.array([new_point1[0], new_point1[1]]) - np.array(drone_pos))
					drone_dis2 = np.linalg.norm(np.array([new_point2[0], new_point2[1]]) - np.array(drone_pos))

					if drone_dis1 > drone_dis2:
						new_point = new_point1
						print "new point 1 selected"
					else:
						new_point = new_point2
						print "new point 2 selected"

					print "Current, New points and new velocities are: ", init_point, new_point, vx_new, vy_new
					px, py, pyaw, seglens, mode = planDubinsPath(start_y = self.drone.x,
							start_x = self.drone.y,
							start_yaw = self.drone.yaw,
							end_y = new_point[0],
							end_x = new_point[1],
							end_yaw = new_point[2],
							curvature = 1/turning_radius)

					self.plan_end_x = px[-1]
					self.plan_end_y = py[-1]
					self.plan_end_yaw = pyaw[-1]
					self.drone.forward_velocity = math.sqrt(vx_new**2 + vy_new**2)
					self.drone.angular_velocity = self.drone.forward_velocity/turning_radius
					self.drone.dubinsMoveDrone(px, py, pyaw)

				else:
					self.done = True


			'''if self.safe_vel:	

				linear = self.drone.forward_velocity
				angular = self.safe_vel.angular.z
				self.safe_vel = None

				direction = 1 if angular > 0 else -1
				target = self.conversion(0, 2*linear*direction/angular, 0, self.drone.odom)

				print "Planning with safe velocity"
				px, py, pyaw, seglens, mode = planDubinsPath(
											  start_y = self.drone.x,
											  start_x = self.drone.y,
											  start_yaw = self.drone.yaw,
											  end_y = target[0],
											  end_x = target[1],
											  end_yaw = self.drone.yaw + math.pi,
											  curvature = self.drone.angular_velocity/self.drone.forward_velocity
											 )

				self.plan_end_x = px[-1]
				self.plan_end_y = py[-1]
				self.plan_end_yaw = pyaw[-1]'''

				# Uncomment to Execute Planned path using Position Controller and Trajectory Message
				

			# if self.next_waypoint and not self.safe_vel:
				
			# 	# print('Replanning to next waypoint as obstacle detected')
				
			# 	# Finding Next Global Waypoint generated due to presence of obstacle
			# 	nextway_x = math.cos(self.drone.yaw)*self.min_safe_distance - math.sin(self.drone.yaw)*(self.next_waypoint) + self.drone.x
			# 	nextway_y = math.sin(self.drone.yaw)*self.min_safe_distance + math.cos(self.drone.yaw)*(self.next_waypoint) + self.drone.y
			# 	# print("Next Waypoint due to Obstacle (Global):", nextway_x, nextway_y)

			# 	# Obstacle at goal Detouring
			# 	# for tag in self.tag_detections:
			# 	# 	if tag.pose.pose.position.z < self.min_safe_distance :
			# 	# 		obs_globalx = math.cos(self.drone.yaw)*tag.pose.pose.position.z + self.drone.x
			# 	# 		obs_globaly = math.sin(self.drone.yaw)*tag.pose.pose.position.z + self.drone.y

			# 	# 		if abs(obs_globalx - self.goal[0]) < 1 and abs(obs_globaly - self.goal[1]) < 1 :
			# 	# 			self.count+=1

			# 	# 			if self.count>1 :
			# 	# 				self.count = 0
			# 	# 				self.goal = self.alternategoal.pop(0)
			# 	# 				break

			# 	print("About to Replan with initial pose :", self.drone.x, self.drone.y, self.drone.yaw)
			# 	px, py, pyaw, seglens, mode = planDubinsPath(
			# 								  start_y = self.drone.x,
			# 								  start_x = self.drone.y,
			# 								  start_yaw = self.drone.yaw,
			# 								  end_y = nextway_x,
			# 								  end_x = nextway_y,
			# 								  end_yaw = self.drone.yaw,
			# 								  curvature = self.drone.angular_velocity/self.drone.forward_velocity
			# 								 )

			# 	self.plan_end_x = px[-1]
			# 	self.plan_end_y = py[-1]
			# 	self.plan_end_yaw = pyaw[-1]

			# 	# self.last_plan = time.time() + 0.5  # To prevent moving into replanning

			# 	self.next_waypoint = None
				
			# 	# Uncomment to Execute Planned path using Position Controller and Trajectory Message
			# 	self.drone.dubinsMoveDrone(px, py, pyaw)

			# # 	# Uncomment to use emulated velocity controller
			# # 	# self.drone.kill_thread = True	#Kill previous ModeDrone Thread

			# # 	# self.drone.thread.join()		# Start a new thread for moveDrone (use while self.kill)
			# # 	# self.drone.done = False			# To prevent going into replanning as drone.done became true at end of function
			# # 	# self.drone.kill_thread = False
			# # 	# self.drone.thread = threading.Thread(target = self.drone.dubinsMoveDrone, args = [mode, seglens, px, py, pyaw])
			# # 	# self.drone.thread.start()

			# # 	time.sleep(1.0)	#blindTime to prevent seeing the obstacle

			# Replan to goal if waypoint or replan time reached
			if self.drone.done or (time.time() - self.last_plan) > 100:
				
				self.drone.done = False				

				# print('Replanning as end of plan or Replan time reached')
				# print("About to Replan to goal with initial pose :", self.drone.x, self.drone.y, self.drone.yaw)
				px, py, pyaw, seglens, mode = planDubinsPath(
											  start_y = self.drone.x,
											  start_x = self.drone.y,
											  start_yaw = self.drone.yaw,
											  end_y = self.goal[0], #+  random.uniform(-0.2, 0.2), 	# Spoofing Goal
											  end_x = self.goal[1], #+ random.uniform(-0.2, 0.2),
											  end_yaw = self.goal[2], #+ math.radians(random.randint(-2,2)),
											  curvature = self.drone.angular_velocity/self.drone.forward_velocity
											 )

				self.plan_end_x = px[-1]
				self.plan_end_y = py[-1]
				self.plan_end_yaw = pyaw[-1]
	
				self.last_plan = time.time()

				self.next_waypoint = None

				# Uncomment to Execute Replanned Motion using Position Controller 
				# Do NOT REPLAN if using POSITION/TRAJECTORY CONTROL
				self.drone.dubinsMoveDrone(px, py, pyaw)

				# Uncomment to use emulated Velocity Controller
				# self.drone.kill_thread = True	#Kill previous ModeDrone Thread

				# self.drone.thread.join()		# Start a new thread for moveDrone (use while self.kill)
				# self.drone.done = False			# To prevent going into replanning as drone.done became true at end of function
				# self.drone.kill_thread = False
				# self.drone.thread = threading.Thread(target = self.drone.dubinsMoveDrone, args = [mode, seglens, px, py, pyaw])
				# self.drone.thread.start()

			# Limit the loop rate
			time.sleep(1/10)

		print('Reached Goal!')
		time.sleep(1)

	def planExecuted(self, x, y, yaw):
		margin_x = 0.2 #self.goal[0] * self.error_margin
		margin_y = 0.2 #self.goal[1] * self.error_margin
		reached_x = abs(x - self.drone.x) < margin_x
		reached_y = abs(y - self.drone.y) < margin_y
		reached_heading = abs(yaw - self.drone.yaw) < math.radians(10)	# 10 degree goal tolerance
		if reached_x and reached_y and reached_heading:
			# print(self.drone.x, self.drone.y, self.drone.yaw)
			return True

	# Callback for apriltags subscriber
	def detectAprilTags(self, tags):
		# Put all tags on a grid
		self.grid.populate(tags.detections)
		#update tag_detections for transforming obstacle pose to global frame
		self.tag_detections = tags.detections

		try:
			self.next_waypoint = self.getWaypoint()
			if self.waypoint:
				pass# print(self.next_waypoint)
		except:
			pass
		# Reset the grid
		self.grid.reset()

	def dynamicObstacle(self, vel_msg):
		print "callback for safe velocity succesful"
		self.safe_vel = vel_msg
		time.sleep(0.3)

	# Helper function to calculate waypoint
	def getWaypoint(self):
		# Slide window and check for freespace
		freespace = False
		self.resetSearch()
		while not freespace and self.control_point < self.grid.size[0]*5:
			# Select the window region
			sub_space_left, sub_space_right = self.crop()
			# Check for obstacles
			obstacles_left = np.count_nonzero(sub_space_left)
			obstacles_right = np.count_nonzero(sub_space_right)
			# Move window if there are obstacles
			if obstacles_left > 0 and obstacles_right > 0:
				self.control_point += 1
			# Return None if window was not moved, else distance of waypoint in metres.
			else:
				self.control_point *= -1 if obstacles_left > 0 else 1
				if self.control_point:
					return self.control_point/10
				else:
					return None

		# Halt if no free space is detected
		self.move = False
		print('No free space detected. Halting!')

	def crop(self):
		y_top = int(self.grid.origin['y'] - self.window_size[1] * 10)
		y_bottom = int(self.grid.origin['y'] + self.window_size[1] * 10)
		wleft_x_left = int(self.grid.origin['x'] - self.window_size[0] * 10 - self.control_point)
		wleft_x_right = int(self.grid.origin['x'] + self.window_size[0] * 10 - self.control_point)
		wright_x_left = int(self.grid.origin['x'] - self.window_size[0] * 10 + self.control_point)
		wright_x_right = int(self.grid.origin['x'] + self.window_size[0] * 10 + self.control_point)

		left_window = self.grid.grid[y_top:y_bottom, wleft_x_left:wleft_x_right]
		right_window = self.grid.grid[y_top:y_bottom, wright_x_left:wright_x_right]

		return left_window, right_window


class Grid(object):
	def __init__(self, grid_size, obstacle_radius, min_safe_distance):
		self.obstacle_radius = obstacle_radius
		self.size = grid_size
		self.min_safe_distance = min_safe_distance
		self.origin = {
			'x': round(grid_size[0]*10/2),
			'y': round(grid_size[1]*10/2)
		}
		# print(10*grid_size)
		self.grid = np.zeros(10 * grid_size)
	
	def reset(self):
		self.grid = np.zeros(10*self.size)

	def populate(self, tags):
		for tag in tags:
			if tag.pose.pose.position.x < self.min_safe_distance :
				
				x = round(-tag.pose.pose.position.y*10) + self.origin['x']
				y = round(-tag.pose.pose.position.z*10) + self.origin['y']
				
				try:
					self.grid[int(y-self.obstacle_radius*10):int(y+self.obstacle_radius*10), int(x-self.obstacle_radius*10):int(x+self.obstacle_radius*10)] = 1
				except Exception as e:
					print('Obstacle omitted dues to padding.')
