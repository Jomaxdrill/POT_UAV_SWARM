import numpy as np
from utilities import distance,get_vector
from curl_free import rot_matrix_field
KF = 10 #formation gain coefficient
KR = 2e15 #repulsive gain coefficient
DO = 0 #limit distance of the potential field influence
MIU = 10#espace force component, #* in theory every UAV should have one unique
#*in theory every UAV should have one unique or maybe not think, this is my desire reference
#*this will be determined by how I want to set my final geometry
# def repulsive_field_fixed_obstacle(pos_uav, obs, short_dist):
# 	real_DO = DO + obs['radius'] #assume this security radius that is supposed only for the drone generates from obstacle
# 	if short_dist < real_DO and short_dist > 0:
# 		return 1/2 * KR * ((1/short_dist)-(1/real_DO))**2
# 	return 0

#*THIS RETURNS A VECTOR
def gradient_force_fixed_obstacle(pos_uav, obs, shor_dist, total_DO):
	force_factor = KR * ((1/shor_dist)-(1/total_DO))*(1/shor_dist**2)
	#the distance vector to the obstacle - the position vector of the obstacle
	# unitary_vector = get_vector(get_vector(pos_uav,obs['center']), obs['center'])/shor_dist
	unitary_vector = get_vector(pos_uav, obs['center'])/shor_dist
	return  force_factor * unitary_vector

#allows UAV i to leave the danger zone and continue the swarm movement
def escape_component(curr_uav, uavs):
	total_esc_vector = np.zeros(2)
	for drone in uavs:
		if drone is not curr_uav:
			diff_vector = np.array(get_vector(uavs[drone]['position'], uavs[curr_uav]['position']))
			#*this is q_j -q_i -delta_ij norm'2 to the square
			escape_norm = (np.linalg.norm(get_vector(get_vector(uavs[curr_uav]['position'], uavs[drone]['position']),
											uavs[curr_uav]['deltas'][drone])))**2
			escape_vector =  (1/escape_norm) *  diff_vector
			# if escape_vector.shape != (2,):
			# 	print(diff_vector)
			# 	print(escape_norm)
			# 	print(escape_vector)
			total_esc_vector += escape_vector
	return MIU * total_esc_vector

def gradient_formation(curr_uav, uavs):
	vector_sum = np.zeros(2)
	for drone in uavs:
		if drone is curr_uav:
			continue
		#*this is q_j -q_i -delta_ij
		diff_vector = np.array(get_vector(get_vector(uavs[drone]['position'],  uavs[curr_uav]['position']),
								uavs[curr_uav]['deltas'][drone]))
		vector_sum += diff_vector
	formation_vector = KF * vector_sum
	print(f'formation_vector is {formation_vector}')
	return formation_vector

def hybrid_algorithm(curr_uav, uavs, obs, border):
	#assume this security radius that is supposed only for the drone generates from obstacle
	#real_DO = obs['radius']*0.20 + obs['radius'] #Asume danger zone equal to 20% of the radius of the obstacle
	real_DO = border + obs['radius']
	shortest_dist_to_obs = distance(uavs[curr_uav]['position'], obs['center'])
	print(f'danger zone distance is  {real_DO}')
	print(f'shortest distance for {curr_uav}  is {shortest_dist_to_obs}')
	if shortest_dist_to_obs <= real_DO and shortest_dist_to_obs > 0:
		print('I entered repulsive')
		# compute the gradient of the repulsive field
		repulsive_force = gradient_force_fixed_obstacle(uavs[curr_uav]['position'], obs, shortest_dist_to_obs, real_DO)
		# compute the escape force
		escaping_force = escape_component(curr_uav, uavs)
		# print(f'repulsive force of obstacle is for drone {curr_uav} {repulsive_force}')
		# print(f'escaping force of obstacle is for drone {curr_uav} {escaping_force}')
		# compute the new position
		#TODO: check correct dimension of repulsive force to multiply
		#the control action is and this should be probably be multiplied by a step size
		#numerical integration will be applied as model is single integrator
		vector_curl = (repulsive_force @ rot_matrix_field(uavs[curr_uav]['direction'],
                                            uavs[curr_uav]['position'],
                                            obs['center']) ) + escaping_force
		print(f'vector_curl repulsive for drone {curr_uav} is {vector_curl}')
		return vector_curl
	return gradient_formation(curr_uav, uavs)
