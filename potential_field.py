import math
from utilities import distance,get_vector
KA= 14 #attractive gain coefficient
KR = 500 #repulsive gain coefficient
DO = 15 #limit distance of the potential field influence
#DO_UAV #? consider a limit distance specific for drones?
# def attractive_field(pos_uav, pos_goal):
# 	return 1/2 * KA * (distance(pos_uav,pos_goal))**2

# def repulsive_field(pos_uav,obs):
# 	dist_to_obstacle = distance(pos_uav,obs)
# 	if dist_to_obstacle < DO and dist_to_obstacle > 0:
# 		return 1/2 * KR * (DO - dist_to_obstacle)**2
# 	return 0

def gradient_goal(pos_uav, pos_goal):
	return KA*(get_vector(pos_goal,pos_uav))

def gradient_obstacle(pos_uav,obs):
	dist_to_obstacle = distance(pos_uav,obs)
	if dist_to_obstacle < DO and dist_to_obstacle > 0:
		return KR *((1/dist_to_obstacle)-(1/DO))*(1/dist_to_obstacle**3)(get_vector(pos_uav, obs))
	return 0

#New position vector at the end
def total_pot_field(pos_uav,pos_goal,obstacles, other_uavs):
	rep_field = []
	# Repulsive force from obstacles
	for obs in obstacles:
		rep_field.append(gradient_obstacle(pos_uav, obs))
	# Repulsive force from other drones
	for drone in other_uavs:
		rep_field.append(gradient_obstacle(pos_uav, drone))
	return gradient_goal(pos_uav, pos_goal) + sum(rep_field)

def drone_control():
#* new position = iteration of total_potential field until all pos_uav are equal to pos_goal
	return