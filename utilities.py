import math
import numpy as np
from itertools import combinations
def get_vector(node_a, node_b):
	return tuple(x - y for x, y in zip(node_a, node_b))
#TODO: get the function of angle between vectors
def get_angle(node_a, node_b):
	#? Use dot product to get the angle
	dot_vectors= np.dot(node_a, node_b)
	magnitude_a = np.linalg.norm(node_a)
	magnitude_b = np.linalg.norm(node_b)
	if magnitude_a ==0:
		print(f'node a {node_a} is null')
	if magnitude_b == 0:
		print(f'node b {node_b} is null')
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

def interpolate_line(start_point, end_point, inter_points=30):
	"""
	This function calculates a set of points that lie on a straight line segment between two given points.
	The line is interpolated using linear interpolation.

	Parameters:
	start_point (tuple): A tuple representing the (x, y) coordinates of the start point of the line segment.
	end_point (tuple): A tuple representing the (x, y) coordinates of the end point of the line segment.
	inter_points (int, optional): The number of intermediate points to calculate between the start and end points.
		Defaults to 30.

	Returns:
	list: A list of tuples, where each tuple represents the (x, y) coordinates of an interpolated point.

	Example:
	>>> interpolate_line((0, 0), (10, 10), 5)
	[(0.0, 0.0), (2.5, 2.5), (5.0, 5.0), (7.5, 7.5), (10.0, 10.0)]
	"""
	points = []
	for idx in range(inter_points):
		t_idx = idx / (inter_points - 1)  # Calculate the parameter t
		x_point = start_point[0] + t_idx * (end_point[0] - start_point[0])  # Linear interpolation for x coordinate
		y_point = start_point[1] + t_idx * (end_point[1] - start_point[1])  # Linear interpolation for y coordinate
		points.append((x_point, y_point))
	return points

def generate_unique_pairs(uavs):
	num_entities = range(0,uavs)
	return list(combinations(num_entities, 2))