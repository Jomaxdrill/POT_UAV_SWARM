import matplotlib.pyplot as plt
import numpy as np
def plot_trajectories(states, obstacles, formation):
	fig, ax = plt.subplots()
	# Plot obstacles
	for idx, obs in obstacles.items():
		if obs['geometry'] == 0:
			circle_obs = plt.Circle(obs['center'], obs['radius'], color='r', label=f'obstacle {idx}')
			ax.add_patch(circle_obs)
		if obs['geometry'] == 1:
			edge = 2*obs['radius']/(np.sqrt(2))
			edge_x = obs['center'][0] - edge
			edge_y = obs['center'][1] - edge
			square_obs = plt.Rectangle(obs['center'],(edge_x, edge_y) ,edge , edge, color='r', label=f'obstacle {idx}')
			ax.add_patch(square_obs)
	#Plot drone trajectories
	for drone_id in states:
		path = np.array(states[drone_id]['path'])
		X_pos = path[:,0]
		Y_pos = path[:,1]
		ax.plot(X_pos, Y_pos, 'o-', label=f'Drone {drone_id}')
		#plot start and final points
		ax.plot(X_pos[0], Y_pos[0],'X-')
		ax.plot(X_pos[-1], Y_pos[-1],'*-')
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
	ax.set_xlim(0, 1000)
	ax.set_ylim(0, 1000)
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
	ax.set_xlabel('x (m)')
	ax.set_ylabel('y (m)')
	ax.set_title('Drone Position and Velocities')
	ax.grid()
	ax.legend()
	plt.show()