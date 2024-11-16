import math
def get_vector(node_a, node_b):
    return tuple(x - y for x, y in zip(node_a, node_b))
#TODO: get the function of angle between vectors
def get_angle(node_a, node_b):
    #? Use dot product??
    return
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
	return round(math.sqrt(substract_vector[0]**2 + substract_vector[1]**2),2)

#TODO: get the function about this
#*Returns the shortest distance between a given robot location and an obstacle(a 2d surface)
def shortest_distance(node_a, nodes):
    short_distance = None
    return short_distance