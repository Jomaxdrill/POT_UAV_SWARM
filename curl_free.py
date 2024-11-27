import numpy as np
from potential_field import gradient_formation
from utilities import distance,get_vector,get_angle
KC = 20 #curl-free gain coefficient
DO = 15 #limit distance of the potential field influence
hor_vector = [1,0] #unitary vector horizon direction
def curl_free_vector_field(pos_uav,obs):
	dist_to_obstacle = distance(pos_uav,obs)
	if dist_to_obstacle < DO and dist_to_obstacle > 0:
		return KC *((1/dist_to_obstacle)-(1/DO))*(1/dist_to_obstacle**3)*((rot_matrix_field(pos_uav,obs) @ get_vector(pos_uav, obs)))
	return 0

def rot_matrix_field(vel_vector,pos_vector, obs_pos):
	#TODO: GET THE CORRECT VECTORS TO EVALUATE
	angle_path = get_angle(vel_vector, hor_vector) #path angle of UAV #? assume the drone has a head ? can be previous vector
	angle_obs = get_angle(get_vector(pos_vector, obs_pos),hor_vector) #angle of relative position vector from UAV and obstacle
	angle_diff = angle_path - angle_obs
	if angle_diff >= 0:
		return np.array([[0, 1],[-1, 0]]) #clockwise direction
	return np.array([[0, -1],[1, 0]]) #counter clockwise direction

