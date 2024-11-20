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
    #plot final formation expected
    # formation = plt.Polygon()
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_xlim(0, 10000)
    ax.set_ylim(0, 10000)
    ax.set_aspect( 1 )
    ax.set_title('Drone Trajectories')
    ax.grid()
    ax.legend()
    plt.show()