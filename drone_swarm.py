
from itertools import combinations
import numpy as np
import math
#*threshold for actions and general dimensions
STEP_TIME = 0.1 #seconds
DUR_ACTION = 1 #seconds
RADIUS_GOAL = 100 #mm
WIDTH_SPACE = 6000  # Horizontal dimension of space
HEIGHT_SPACE = 2000  # Vertical dimension of space
N_UAVS = 6 # Number of Drones/Agents in the process
def rotation_vector_by(angle):
    if angle < 0:
        angle = angle % 360
    angle_rad = np.radians(angle)
    return np.array([[round(np.cos(angle_rad), 2),
                        round(-np.sin(angle_rad), 2)],
                            [round(np.sin(angle_rad), 2),
                                round(np.cos(angle_rad), 2)]])
##* Positions/states will be created followed a geometrical rule.
# If 0 -ERROR no drones
#If 1 center of the formation
#If 2 or greater define the positions based on the positions by 360/uavs in a given radius beggining from angle 0 in circunference
#eg: if uav== 2 then one uav will be every 180 degrees from 0 to 360.
#if uav==3 then each uav will be every 120 degrees
def initial_positions(radius, center, total_uavs):
    init_state = {}
    MIN_ANGLE = 360/ total_uavs
    MATRIX_ROT = rotation_vector_by(MIN_ANGLE)
    HOM_TRANS = np.array([[1,0,0,center[0]],
                        [0,1,0,center[1]],
                        [0,0,1,0],
                        [0,0,0,1]]
                        )
    vector_init = [radius, 0]
    #Rotate this vector N uavs by the respective angle
    for drone in range(0,total_uavs):
        rotated_vector = MATRIX_ROT @ vector_init
        init_position = HOM_TRANS @ [*rotated_vector,1]
        init_state[drone] = init_position[:2]
        vector_init = init_state[drone]


def generate_unique_pairs(uavs):
    return list(combinations(uavs, 2))



