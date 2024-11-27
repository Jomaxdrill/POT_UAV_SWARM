import math
import numpy as np
from utilities import distance,get_vector
KA= 5 #attractive gain coefficient
KR = 1e12 #repulsive gain coefficient

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
	print(f'formation_vector is {formation_vector}')
	return formation_vector

def gradient_obstacle(pos_uav, obs, shor_dist, total_DO):
	return KR *((1/shor_dist)-(1/total_DO))*(1/shor_dist**3)*np.array((get_vector(pos_uav, obs)))

#New position vector at the end
def pot_field(curr_uav, uavs, obs, short_dist, border):
	real_DO = border + obs['radius']
	formation_attraction_force = gradient_formation(curr_uav, uavs)
	print(f'attraction force formation is {formation_attraction_force}')
	repulsive_obs_force = np.zeros(2)
	if short_dist <= real_DO and short_dist > 0:
		repulsive_obs_force = gradient_obstacle(uavs[curr_uav]['position'], obs['center'], short_dist, real_DO)
	print(f'repulsive force obstacle is {repulsive_obs_force}')
	return formation_attraction_force + repulsive_obs_force
