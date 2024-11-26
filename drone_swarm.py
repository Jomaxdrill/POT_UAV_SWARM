
from itertools import combinations
from utilities import get_vector,distance, rotation_vector_by, interpolate_line
from hybrid import hybrid_algorithm
from plotting import plot_trajectories, plot_pos_vels
import numpy as np
import math
#*threshold for actions and general dimensions
MAX_ITER = 20
#TODO: tune these values
STEP_TIME = 0.05 #seconds
INTER_POINTS = 50 #number of previous points to consider in a new position
#*The control law a high speed vector could be potentially generated which could be realistic
#*impossible to achieve for a drone
#
MIN_VEL = 400
MAX_VEL = 1000
RADIUS_FORMATION = 200#mm
DELTA_TOLERANCE = 5 #* a margin for the relative horizontal distance expected
WIDTH_SPACE = 1000  #mm Horizontal dimension of space
HEIGHT_SPACE = 1000  #mm Vertical dimension of space
N_UAVS = 3 # Number of Drones/Agents in the process, MUST BE GREATER THAN 2
BORDER_UAV = 30 # radius of the area around the drone where repulse components are activated (the danger zone)
#INITIAL POSITIONS ELEMENTS MUST MATCH THE N_UAVS
#TODO: MAKE THE USER INPUT THE ENTRIES ?
INITIAL_POSITIONS = np.array([
	[150, 150],
	[500, 400],
	[500, 500]], dtype=float
)
# INITIAL_POSITIONS = np.array([
# 	[40, 200],
# 	[900, 500],
# 	[900, 300],
# 	[7000, 100],
# 	[300, 150],
# 	[150, 600]
# ], dtype=float
# )
states_drones = {}
final_formation = {}
geometries = {
	0: 'Circle',
	1: 'Rectangle',
	2: 'Polygon'
}
obstacles = {
	1: {
	'center': np.array([250, 250]),
	'radius': 100, #radius to the further point of the obstacle
	'geometry': 0
	}
}
algorithms = ['potential']
states_drones = {}
##* Positions/states will be created followed a geometrical rule.
# If 0 OR 1 -ERROR no drones If 1 center of the formation
#If 2 or greater define the positions based on the positions by 360/uavs in a given radius beggining from angle 0 in circunference
#eg: if uav== 2 then one uav will be every 180 degrees from 0 to 360.
#if uav==3 then each uav will be every 120 degrees
def final_positions():
	MIN_ANGLE = 360/ N_UAVS
	MATRIX_ROT = rotation_vector_by(MIN_ANGLE)
	#Homogeneous transformation matrix from the center of the formation to the base coordinate system
	# HOM_TRANS = np.array([[1,0,0,center[0]],
	#                     [0,1,0,center[1]],
	#                     [0,0,1,0],
	#                     [0,0,0,1]]
	#                     )
	vector_init = [RADIUS_FORMATION, 0] #initial vector in center of formation frame
	#Rotate this vector for every i_uavs by the respective angle
	for drone in range(0, N_UAVS):
		rotated_vector = MATRIX_ROT @ vector_init
		# final_position = HOM_TRANS @ [*rotated_vector,1]
		# final_formation[drone] = final_position[:2]
		final_formation[drone] = rotated_vector
		vector_init = final_formation[drone]

#get the distances we need
def delta_formations(curr_drone):
	delta_positions = {}
	for next_drone, next_pos in final_formation.items():
		if next_drone is curr_drone:
			continue
		delta_positions[next_drone] = get_vector(final_formation[curr_drone], next_pos)
	return delta_positions

def create_state_formation():
	#*The drones will have random positions at first
	drone = 0
	final_positions()
	while drone < N_UAVS:
		#TODO: SET PROPER INITIAL
		# init_pos =  np.array((np.random.rand()* WIDTH_SPACE, np.random.rand()* HEIGHT_SPACE))
		init_pos = INITIAL_POSITIONS[drone]
		#*if the initial position is inside an obstacle, re-generate a new one
		while check_in_obstacle(init_pos):
			init_pos = np.array((np.random.rand()* WIDTH_SPACE, np.random.rand()* HEIGHT_SPACE))
		states_drones[drone] = {}
		states_drones[drone]['position'] = init_pos
		states_drones[drone]['direction'] = np.array([0, 0]) #is the direction of velocity vector
		states_drones[drone]['path'] = [init_pos]
		states_drones[drone]['velocities'] = []
		#get the desired formation horizontally
		states_drones[drone]['deltas'] = delta_formations(drone)
		drone += 1
	#generate a random vector between the current
	#check it's not inside an obstacle


def check_in_obstacle(state):
	"""
	This function checks if a given state is within the obstacle space.

	Args:
		state (tuple): The horizontal and vertical coordinates of the state.
		border (int): The clearance of the obstacles.

	Returns:
		bool: True if the state is within the obstacle space, False otherwise.

	"""
	tl = BORDER_UAV
	x_pos, y_pos = state
	# #outside of space
	if x_pos < 0 or y_pos < 0:
		#print(f'outside of space')
		return True
	if x_pos > WIDTH_SPACE or y_pos > HEIGHT_SPACE:
		#print(f'outside of space')
		return True

	for obs_info in obstacles.values():
		#obstacle square
		#in_obstacle_0 = (x_pos >= 1500 - tl) and (x_pos <= 1750 + tl) and (y_pos >= 1000 - tl) and (y_pos <= HEIGHT_SPACE)
		# obstacle circle
		in_obstacle = np.power(x_pos - obs_info['center'][0], 2) + \
            np.power(y_pos - obs_info['center'][1], 2) <= \
            np.power(obs_info['radius'] + tl, 2)
		if in_obstacle:
			return True
	return False


def check_nearest_obstacle(pos):
	nearest_obs = 0
	curr_distance = np.inf
	for key, obs in obstacles.items():
		diff_distance = distance(pos, obs['center'])
		if diff_distance < curr_distance:
			nearest_obs = key
			curr_distance = diff_distance
	return nearest_obs

def verify_end_formation():
	flag_complete_formation = True
	deltas_drone_0 = states_drones[0]['deltas']
	drone_0_pos = states_drones[0]['position']
	for drone in range(1,N_UAVS):
		next_drone_pos = states_drones[drone]['position']
		#get the vector between them and compare components with expected delta
		current_delta = get_vector(drone_0_pos, next_drone_pos)
		#Verify if the current distance x,y between drone 0 and drone i is equal to the expected delta
		flag_complete_formation &= np.all(np.abs(get_vector(current_delta,deltas_drone_0[drone])) <= DELTA_TOLERANCE)
	if flag_complete_formation:
		print(f"Drone {drone} has achieved all the desired formations.")
		return True
	return False


#TODO: Consider do a position controller for each drone, eg a PD controller

def control_law(algorithm, uavs):
	formation_completed = False
	iterations = 0
	while not formation_completed and iterations <= MAX_ITER:
		print(f'iterations: {iterations}',end="\r")
		for curr_uav, drone_info in uavs.items():
			#TODO: Maybe i need a function to detect the nearest obstacle
			nearest_obs = check_nearest_obstacle(drone_info['position'])
			new_vel_vector = algorithm(curr_uav, uavs, obstacles[nearest_obs], BORDER_UAV)
			mag_new_vel_vector = np.linalg.norm(new_vel_vector)
			print(f'mag vel is {mag_new_vel_vector}')
			#Consider a saturation condition especially for cases where jamming might be present when reaching obstacle
			if mag_new_vel_vector <= 0:
				print(f'zero speed for {curr_uav}')
				continue
			if mag_new_vel_vector <= MIN_VEL:
				print(f'not so much speed for {curr_uav}')
				new_vel_vector = new_vel_vector * (MIN_VEL / mag_new_vel_vector)
			if mag_new_vel_vector >= MAX_VEL :
				print(f'not so fast {curr_uav}')
				new_vel_vector = new_vel_vector * (MAX_VEL / mag_new_vel_vector)
			#print(f'for drone {curr_uav} is {new_vel_vector}')
			#get new position
			new_pos = drone_info['position'] + (new_vel_vector * STEP_TIME)
			#check if new position and the path is in obstacle space
			# if collision(drone_info['position'], new_pos, INTER_POINTS):
			# 	continue
			# if check_in_obstacle(new_pos):
			# 	continue
			states_drones[curr_uav]['position'] = new_pos
			states_drones[curr_uav]['direction'] = new_vel_vector
			states_drones[curr_uav]['path'].append(new_pos)
			states_drones[curr_uav]['velocities'].append(new_vel_vector)
			formation_completed = verify_end_formation()
			if formation_completed:
				print(f'formation took seconds to complete')
				return True
		iterations += 1

def generate_unique_pairs(uavs):
	return list(combinations(uavs, 2))

def collision(node, new_node, inter_points):
	"""
	This function checks if a collision occurs between a line segment defined by two nodes and obstacles.

	Parameters:
	node (tuple): The coordinates of the starting point of the line segment.
	new_node (tuple): The coordinates of the ending point of the line segment.
	option (int): The option to determine the obstacle space.
	constraints (tuple): The constraints for the obstacle space.
	inter_points (int): The number of intermediate points to interpolate between the start and end points.
	matrix (numpy.ndarray, optional): The matrix representation of the maze for image-based obstacle detection.

	Returns:
	bool: True if a collision occurs, False otherwise.

	"""
	hit = False
	line = interpolate_line(node, new_node, inter_points)
	for point in line:
		if check_in_obstacle(point):
			hit = True
			break
	return hit


if __name__ == "__main__":
	create_state_formation()
	control_law(hybrid_algorithm, states_drones)
	for drone in states_drones:
		print(f"\nDrone {drone}: Position: {states_drones[drone]['position']},\
		Direction: {states_drones[drone]['direction']}\n\
		")
	plot_trajectories(states_drones, obstacles, final_formation)
	#plot_pos_vels(states_drones)
