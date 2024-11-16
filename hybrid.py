import numpy as np
from utilities import distance,get_vector,short_distance
from curl_free import rot_matrix_field
KF = 14 #formation gain coefficient
KR = 500 #repulsive gain coefficient
DO = 15 #limit distance of the potential field influence
miu = 15 #espace force component, #* in theory every UAV should have one unique
delta = [15, 15] # expected relative distance between UAVS in X,Y direction at the end of formation
#*in theory every UAV should have one unique or maybe not think, this is my desire reference
#*this will be determined by how I want to set my final geometry
def repulsive_field_fixed_obstacle(pos_uav, obs):
	#TODO: Get the function that guarantees you get the shortest distance from your robot to the obstacle in question
	shortest_dist_to_obs = short_distance(pos_uav,obs)
	if shortest_dist_to_obs < DO and shortest_dist_to_obs > 0:
		return 1/2 * KR * ((1/shortest_dist_to_obs)-(1/DO))**2
	return 0

#*THIS RETURNS A VECTOR
def gradient_force_fixed_obstacle(pos_uav, obs, shor_dist):
	force_factor = KR * ((1/shor_dist)-(1/DO))*(1/shor_dist**2)
	unitary_vector = get_vector(pos_uav, obs)/shor_dist
	return  force_factor * unitary_vector

#allows UAV i to leave the danger zone and continue the swarm movement
def escape_force(curr_uav, uavs):
	total_esc_vector = [0,0]
	for drone in uavs:
		if drone is not curr_uav:
			diff_vector = get_vector(drone,curr_uav)
			escape_norm = (np.linalg.norm(get_vector(diff_vector,delta)))**2
			escape_vector = miu * diff_vector / escape_norm
			total_esc_vector = total_esc_vector + escape_vector

def gradient_formation(curr_uav, uavs):
    vector_sum = np.array([0,0])
    for drone in uavs:
        diff_vector = get_vector(get_vector(drone,curr_uav),delta)
        vector_sum += diff_vector
    return KF * vector_sum

def hybrid_algorithm(curr_uav, uavs, obs):
    shortest_dist_to_obs = short_distance(curr_uav,obs)
    if shortest_dist_to_obs < DO and shortest_dist_to_obs > 0:
        # compute the gradient of the repulsive field
        repulsive_force = gradient_force_fixed_obstacle(curr_uav, obs, 'repulsive')
        # compute the escape force
        escape_force = escape_force(curr_uav, uavs)
        # compute the new position
        #TODO: check correcto dimension of repulsive force to multiply
        #the control action is and this should be probably be multiplied by a step size
        #numerical integration will be applied as model is single integrator
        return repulsive_force @ rot_matrix_field(curr_uav, obs) + escape_force
    return gradient_formation(curr_uav, uavs)
