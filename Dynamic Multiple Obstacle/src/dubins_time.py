import dubins
import math

def generateTimestamp(px, py, pyaw, v, w):
	timestamp = []
	compensation_constant = 0
	timestamp.append(0)
	for i in range(1,len(px)):
		if pyaw[i] < 0 and 4.6 < pyaw[i-1] < 4.8 :		#entering negative yaw from +3pi/2 = 4.7099
			dtheta = pyaw[i] + 2*math.pi - pyaw[i-1]
		elif pyaw[i] > 0 and pyaw[i-1] < -1.5 :		#exiting negative yaw from -pi/2 = -1.57
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

def planDubinsPath(start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature):

	start_x *= -1
	end_x *= -1
	start_yaw += math.pi/2
	end_yaw += math.pi/2

	start = (start_x, start_y, start_yaw)
	end = (end_x, end_y, end_yaw)
	radius = 1/curvature
	step_size = 0.01

	path = dubins.shortest_path(start, end, radius)
	configurations, _ = path.sample_many(step_size)
	
	seglens = [path.segment_length(0), path.segment_length(1), path.segment_length(2)]
	
	modes = {
			 0:"LSL",
			 1:"LSR",
			 2:"RSL",
			 3:"RSR",
			 4:"RLR",
			 5:"LRL",
			}

	mode = modes[path.path_type()]

	px = [i[1] for i in configurations]
	py = [-i[0] for i in configurations]
	pyaw = [i[2]-(math.pi/2) for i in configurations]

	return px, py, pyaw, seglens, mode

# def main():
# 	px, py, pyaw = planDubinsPath(0,0,1.57,-5,0,1.57,0.8)
# 	t = generateTimestamp(px, py, pyaw, 1, 1)
# 	print t

# if __name__ == '__main__':
# 	main()
