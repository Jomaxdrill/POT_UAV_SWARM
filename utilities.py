import math
import numpy as np
def get_vector(node_a, node_b):
    return tuple(x - y for x, y in zip(node_a, node_b))
#TODO: get the function of angle between vectors
def get_angle(node_a, node_b):
    #? Use dot product to get the angle
    dot_vectors= np.dot(node_a, node_b)
    magnitude_a = np.linalg.norm(node_a)
    magnitude_b = np.linalg.norm(node_b)
    return math.acos(dot_vectors / (magnitude_a * magnitude_b))
def distance(node_a, node_b):
	"""
	Returns the Euclidean distance between two nodes.

	Args:
		node_a (tuple): The first node.
		node_b (tuple): The second node.

	Returns:
		float: The Euclidean distance between the two nodes.

	"""
	substract_vector = get_vector(node_a, node_b)
	#? Euclidean distance squared has given better performance
	return np.linalg.norm(substract_vector)
	# return round(math.sqrt(substract_vector[0]**2 + substract_vector[1]**2),2)

def rotation_vector_by(angle):
	if angle < 0:
		angle = angle % 360
	angle_rad = np.radians(angle)
	return np.array([[round(np.cos(angle_rad), 2),
						round(-np.sin(angle_rad), 2)],
							[round(np.sin(angle_rad), 2),
								round(np.cos(angle_rad), 2)]])
