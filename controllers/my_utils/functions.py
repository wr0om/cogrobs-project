
from classes_and_constants import CPU_CHANNEL, DRONE_CHANNEL
import ast
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline


channels_to_str = {CPU_CHANNEL: "CPU", DRONE_CHANNEL: "DRONE"}

def find_new_obstacle1(obs_ps0, detected_obstacle0, detected_obstacle1, def_values0, prev_new):
    found_new = []
    for i in range(8):
        if detected_obstacle0[i] and detected_obstacle1[i] and (detected_obstacle0[(i+1)%8] and detected_obstacle1[(i+1)%8] or detected_obstacle0[(i-1)%8] and detected_obstacle1[(i-1)%8]): 
            if detected_obstacle0[(i+2)%8] and detected_obstacle1[(i+2)%8] or detected_obstacle0[(i-2)%8] and detected_obstacle1[(i-2)%8]:
                found_new.append(False)
            else:
                if prev_new[i] and def_values0[i] < 15:
                    found_new.append(False)
                else:
                    found_new.append(True)
        elif obs_ps0[i] < 500.0 and def_values0[i]>2:
            found_new.append(True)
        else:
            found_new.append(False)
    return found_new

def send_msg_to_cpu(emitter, msg):
    emitter.setChannel(CPU_CHANNEL)
    emitter.send(str(msg).encode('utf-8'))
    #print(f"Sent position to CPU: {msg}")

def get_msg_from_cpu(receiver):
    messages = []
    while receiver.getQueueLength() > 0:
        message = receiver.getString()
        if message not in [None, ""]:
            message = ast.literal_eval(message)
            print(f"Received message from CPU: {message}")
            messages.append(message)
        receiver.nextPacket()

    return messages

def coords_to_string(coords):
    return f"{coords[0]},{coords[1]},{coords[2]}"

def send_msg_to_drone(emitter, msg):
    emitter.setChannel(DRONE_CHANNEL)
    emitter.send(str(msg).encode('utf-8'))
    print(f"Sent message to Drone: {msg}")

def get_enemy_drones_positions(enemy_drone_names, enemy_drone_robots):
    """
    Get the positions of enemy drones in the environment.

    Parameters:
    enemy_drone_names: List of names of enemy drones.
    enemy_drone_robots: List of robot objects representing enemy drones.

    Returns:
    A list of tuples containing the positions of enemy drones.
    """
    enemy_drone_positions = dict()
    for enemy_name, enemy_robot in zip(enemy_drone_names, enemy_drone_robots):
        enemy_position = enemy_robot.getField("translation").getSFVec3f()
        enemy_drone_positions[enemy_name] = enemy_position
    return enemy_drone_positions

def plot_drone_movement(all_drone_locations, destroyed_drone_locations, total_drone_time, total_drone_distance, file_path):
    # Extract coordinates from the movement and destroyed locations
    x_coords = [coord[0] for coord in all_drone_locations]
    y_coords = [coord[1] for coord in all_drone_locations]
    z_coords = [coord[2] for coord in all_drone_locations]

    destroyed_x = [coord[0] for coord in destroyed_drone_locations]
    destroyed_y = [coord[1] for coord in destroyed_drone_locations]
    destroyed_z = [coord[2] for coord in destroyed_drone_locations]

    # Interpolation of the path
    t = np.linspace(0, len(x_coords) - 1, len(x_coords))
    t_new = np.linspace(0, len(x_coords) - 1, 300)  # More points for smoothness

    # Create smooth paths for x, y, z coordinates
    x_smooth = make_interp_spline(t, x_coords)(t_new)
    y_smooth = make_interp_spline(t, y_coords)(t_new)
    z_smooth = make_interp_spline(t, z_coords)(t_new)

    # Create the figure and 3D axis
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Time progression color gradient
    time_colors = np.linspace(0, total_drone_time, len(t_new))  # Linearly spaced time values for color gradient
    colors = plt.cm.plasma(time_colors / total_drone_time)  # Normalize colors to range between 0 and 1

    # Plot the interpolated smooth path of the drone with time-based colors
    for i in range(len(t_new) - 1):
        ax.plot(x_smooth[i:i+2], y_smooth[i:i+2], z_smooth[i:i+2],
                color=colors[i], linewidth=2)

    # Create a dummy scatter plot for the color bar (time-based gradient legend)
    time_color_bar = ax.scatter(x_smooth, y_smooth, z_smooth, c=time_colors,
                                cmap='plasma', s=0)  # Set `s=0` to avoid visible points

    # Add a color bar to represent the time progression along the path
    cbar = plt.colorbar(time_color_bar, pad=0.1)
    cbar.set_label('Time Progression (s)', rotation=270, labelpad=15)

    # Emphasize the destroyed drone locations, plotted on top
    ax.scatter(
        destroyed_x, destroyed_y, destroyed_z,
        color='red', s=300, marker='x', edgecolors='black', linewidth=2.5,
        label="Destroyed Drones", zorder=10
    )

    # Add text labels next to each destroyed drone marker
    for i, (x, y, z) in enumerate(zip(destroyed_x, destroyed_y, destroyed_z)):
        ax.text(x, y, z + 0.3, f"{i+1}", color='black', fontsize=12, fontweight='bold', ha='right')

    # Set labels and title with total time and distance (formatted to 2 decimal places)
    ax.set_title(f'3D Friendly Drone Movement', fontsize=15, fontweight='bold')
    ax.set_xlabel('X Coordinate', fontsize=12)
    ax.set_ylabel('Y Coordinate', fontsize=12)
    ax.set_zlabel('Z Coordinate', fontsize=12)

    # Customize the grid, background, and view angle
    ax.grid(True)
    ax.set_facecolor('#f0f0f0')
    ax.view_init(elev=30, azim=45)

    # Annotate with total time and distance on the plot (formatted to 2 decimal places)
    ax.text2D(0.05, 0.95, f"Total Time: {total_drone_time:.2f} seconds\nTotal Distance: {total_drone_distance:.2f} units",
              transform=ax.transAxes, fontsize=13, verticalalignment='top')

    # Add legend for clarity
    ax.legend(loc="best", fontsize=10)

    # Save the plot to a file
    plt.savefig(file_path, dpi=300)

    # Show the plot
    plt.show()