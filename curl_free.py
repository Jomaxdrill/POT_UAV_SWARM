import numpy as np
from utilities import distance,get_vector,get_angle
KA = 1.05 # attractive formation gain coefficient
KC = 2e13#curl-free gain coefficient
hor_vector = [1,0] #unitary vector horizon direction
def curl_free_field_force(curr_uav,obs,shor_dist, total_DO):
	return KC *((1/shor_dist)-(1/total_DO))*(1/shor_dist**3)*\
	(rot_matrix_field(curr_uav['direction'],curr_uav['position'],

        obs['center']) @ get_vector(curr_uav['position'], obs['center']))

def rot_matrix_field(vel_vector,pos_vector, obs_pos):
	angle_path = get_angle(vel_vector, hor_vector) #path angle of UAV #? assume the drone has a head ? can be previous vector
	angle_obs = get_angle(get_vector(pos_vector, obs_pos),hor_vector) #angle of relative position vector from UAV and obstacle
	angle_diff = angle_path - angle_obs
	if angle_diff >= 0:
		return np.array([[0, 1],[-1, 0]]) #clockwise direction
	return np.array([[0, -1],[1, 0]]) #counter clockwise direction

def gradient_formation(curr_uav, uavs):
	vector_sum = np.zeros(2)
	for drone in uavs:
		if drone is curr_uav:
			continue
		#*this is q_j -q_i -delta_ij
		vector_uavs = get_vector(uavs[drone]['position'],  uavs[curr_uav]['position'])
		vector_uavs_delt = get_vector(vector_uavs, uavs[curr_uav]['deltas'][drone])
		diff_vector = np.array(vector_uavs_delt)
		vector_sum += diff_vector
	formation_vector = KA * vector_sum
	return formation_vector

def curl_free_vel_field(curr_uav, uavs, obs, short_dist, border):
	real_DO = border + obs['radius']
	formation_attraction_force = gradient_formation(curr_uav, uavs)
	#print(f'attraction force formation is {formation_attraction_force}')
	repulsive_obs_force = np.zeros(2)
	if short_dist <= real_DO and short_dist > 0:
		repulsive_obs_force = curl_free_field_force(uavs[curr_uav], obs, short_dist, real_DO)
		#print(f'repulsive force obstacle is {repulsive_obs_force}')
	return formation_attraction_force + repulsive_obs_force
