import matplotlib.pyplot as plt
import numpy as np
def plot_trajectories(states, obstacles,constraints):
	WIDTH, HEIGHT, BORDER = constraints
	fig, ax = plt.subplots()
	# Plot obstacles
	for idx, obs in obstacles.items():
		if obs['geometry'] == 0:
			circle_obs = plt.Circle(obs['center'], obs['radius'], color='r', label=f'obstacle {idx}')
			ax.add_patch(circle_obs)
		if obs['geometry'] == 1:
			edge = obs['radius']*(np.sqrt(2)/2)
			edge_x = obs['center'][0] - edge
			edge_y = obs['center'][1] - edge
			square_obs = plt.Rectangle((edge_x, edge_y) ,2*edge , 2*edge, color='g', label=f'obstacle {idx}', fill= False,linewidth=2.5)
			circle_obs = plt.Circle(obs['center'], obs['radius']+ BORDER, color='r', label=f'obstacle {idx}')
			ax.add_patch(circle_obs)
			ax.add_patch(square_obs)
	#Plot drone trajectories and set dynamically plot limits
	X_max = -np.inf
	Y_max = -np.inf
	X_min = np.inf
	Y_min = np.inf
	for drone_id in states:
		path = np.array(states[drone_id]['path'])
		X_pos = path[:,0]
		Y_pos = path[:,1]
		max_X = max(X_pos)
		max_Y = max(Y_pos)
		min_X = min(X_pos)
		min_Y = min(Y_pos)
		if max_X > X_max:
			X_max = max_X
		if max_Y > Y_max:
			Y_max = max_Y
		if min_X < X_min:
			X_min = min_X
		if min_Y < Y_min:
			Y_min = min_Y
		ax.plot(X_pos, Y_pos, '.-', label=f'Drone {drone_id}')
		#plot start and final points
		ax.plot(X_pos[0], Y_pos[0],'X-', color='black', markersize=5)
		#ax.plot(X_pos[-1], Y_pos[-1],'*-')
	print(f'limits are: {X_min};{X_max};{Y_min};{Y_max}')
	#plot final formation expected
	points_formation = []
	for drone_id in states:
		points_formation.append(states[drone_id]['path'][-1])
	#add last element to enclose the polygon formation
	points_formation.append(points_formation[0])
	formation_lines = []
	#create lines from these points
	for idx in range(len(points_formation)-1):
		formation_lines.append([points_formation[idx], points_formation[idx+1]])
	print(f'lines are {np.shape(formation_lines)}')
	formation_lines = np.reshape(formation_lines,(len(formation_lines)*2, 2))
	ax.plot(formation_lines[:,0],formation_lines[:,1],'<--', linewidth=5)
	ax.set_xlabel('x (m)')
	ax.set_ylabel('y (m)')
	# ax.set_xlim(X_min*1.10, X_max*1.10)
	# ax.set_ylim(Y_min*1.10, Y_max*1.10)
	# ax.set_xlim(-1000, 1000)
	# ax.set_ylim(-1000, 1000)
	ax.set_aspect( 1 )
	ax.set_title('Drone Trajectories')
	ax.grid()
	ax.legend()
	plt.show()
def plot_pos_vels(states):
	fig, ax = plt.subplots()
	for drone_id in states:
		path = np.array(states[drone_id]['path'])
		vels = np.array(states[drone_id]['velocities'])
		X_pos = path[:,0]
		Y_pos = path[:,1]
		# vel_X = vels[:,0]
		# vel_Y = vels[:,1]
		vel_X = np.ones(len(X_pos), dtype=float)
		vel_Y = np.ones(len(Y_pos), dtype=float)
		ax.plot(X_pos, Y_pos, 'o-', label=f'Drone {drone_id}')
		ax.plot(X_pos, Y_pos, 'o-', label='Drone Position')
		ax.quiver(X_pos[:-1], Y_pos[:-1], vel_X[:-1], vel_Y[:-1], color='yellow',label=f'Drone Velocities {drone_id}')
	ax.set_xlabel('x [m]')
	ax.set_ylabel('y [m]')
	ax.set_title('Drone Position and Velocities')
	ax.grid()
	ax.legend()
	plt.show()

def plot_short_distance(states, step, obstacle):
	_, ax = plt.subplots()
	for drone in states.keys():
		short_dist_drone_obs = states[drone]['short_dist'][obstacle]
		num_steps = len(states[drone]['short_dist'][obstacle])
		final_time = step * num_steps
		time_range = np.linspace(0, final_time,endpoint=False, num=num_steps)
		ax.plot(time_range, short_dist_drone_obs, label=f'Drone {drone}')
	ax.set_xlabel('Time [s]')
	ax.set_ylabel('Distances [m]')
	ax.set_title(f'Distances from each of the drones to the obstacle {obstacle}')
	ax.grid()
	ax.legend()
	plt.show()