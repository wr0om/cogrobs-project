"""odometer_calculation controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import sys
import os
import math
import ast
import numpy as np


libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)
from controller import Robot, Supervisor
from classes_and_constants import *
from planner import DronePathFinderWithUncertainty
from functions import *

np.random.seed(SEED)

path_to_save = "../experiments/"

if INPLACE:
    path_to_save += f"inplace/"
elif MOVING:
    path_to_save += f"moving_spheres/"
elif ADVERSARIAL:
    path_to_save += f"adversarial/"
else:
    path_to_save += f"moving/"

if USE_RANDOM:
    path_to_save += "baseline/"
elif USE_CENTROIDS:
    path_to_save += "centroid/"
elif USE_RADIUS:
    path_to_save += "radius/"
else:
    path_to_save += "optimal/"

if not os.path.exists(path_to_save):
    os.makedirs(path_to_save)

file_path = path_to_save + f"seed_{SEED}.png"



def replan_path(drones_positions, emitter, use_random=False, use_radius=False, all_drone_radii=None):
    drone_position = drones_positions["Drone"]
    position_list = [drone_position] + [position for drone, position in drones_positions.items() if drone != "Drone"]

    # if only one drone left, send its location
    if len(position_list) == 2:
        print("Only one drone left, sending its location")
        plan_coords = [position_list[1]]
        message = ("CPU", plan_coords)
        send_msg_to_drone(emitter, message)
        return plan_coords


    if use_radius and all_drone_radii:
        radii_list = [radius for drone, radius in all_drone_radii.items() if drone != "Drone"]
    else:
        radii_list = [0 for drone in drones_positions.keys() if drone != "Drone"]

  
    if use_random:
        enemy_drone_positions = position_list[1:]
        # np random permutation of enemy drone positions
        plan_coords = np.random.permutation(enemy_drone_positions)
        # convert to regular list
        plan_coords = [tuple(coords) for coords in plan_coords]
        message = ("CPU", plan_coords)
        send_msg_to_drone(emitter, message)
        return plan_coords

    position_list = [coords_to_string(position) for position in position_list]

    path_finder = DronePathFinderWithUncertainty(position_list, radii_list)
    print("Planning path...")
    planned_path = path_finder.solve()
    print(f"Planned_path: {planned_path}")

    if planned_path:
        plan_coords = path_finder.get_plan_coords()
        print(f"goal_location: {plan_coords[0]}") 
        # TODO: send goal_location to Drone 
        message = ("CPU", plan_coords)
        send_msg_to_drone(emitter, message)
    else:
        print("ERROR!!!! No plan found!")
    return plan_coords


def run_robot(robot):
    # get the time step of the current world.
    timestep = 64

    # Get the emitter device
    emitter = robot.getDevice('emitter')
    # receiver = robot.getDevice('receiver')
    # receiver.enable(timestep)
    # receiver.setChannel(CPU_CHANNEL)

    # Create a Supervisor instance
    supervisor = Supervisor()

    # Get the root node
    root = supervisor.getRoot()

    # Get the children field of the root node
    children_field = root.getField("children")
    num_children = children_field.getCount()

    # List to hold robot names
    robot_names = []

    # Iterate through children nodes
    for i in range(num_children):
        child_node = children_field.getMFNode(i)

        # Check if the node is of type 'Robot'
        if child_node is not None and child_node.getTypeName() == "Robot":
            # Access the 'name' field of the robot node
            name_field = child_node.getField("name")
            if name_field is not None and name_field.getSFString() != "main_computer":
                robot_name = name_field.getSFString()  # Use getSFString() to get the single string value
                robot_names.append(robot_name)  # Append the robot name to the list

    # Output the robot names
    print("Robot names in the environment:")
    for name in robot_names:
        print(name)


    # get robot objects
    drone_robots = [supervisor.getFromDef(name) for name in robot_names]
    drones_positions = get_enemy_drones_positions(robot_names, drone_robots)

    last_replanner = 0
    last_drone_positions = drones_positions
    current_plan = None
    goal_location = None
    all_drone_locations = []
    destroyed_drone_locations = []

    # for metrics
    start_time = 0
    last_drone_location = drones_positions["Drone"]
    total_drone_distance = 0
    total_drone_time = 0

    # computer doens't start until our drone is high enough
    while drones_positions["Drone"][2] < 3.9:
        #print("Drone is not high enough, waiting...")
        drones_positions = get_enemy_drones_positions(robot_names, drone_robots)
        robot.step(timestep)


    # experiments
    use_random = USE_RANDOM
    use_centroids = USE_CENTROIDS
    all_drone_centroids = {drone: {"centroid": position, "count": 1} for drone, position in drones_positions.items() if drone != "Drone"}
    use_radius = USE_RADIUS
    all_drone_radii = {drone: 0 for drone in all_drone_centroids.keys()}

    sim_time = 0.0
    while robot.step(timestep) != -1:
        sim_time += timestep / 1000.0  # Convert ms to seconds

        current_time = sim_time
        #print(f"Current time: {current_time}")
        drones_positions = get_enemy_drones_positions(robot_names, drone_robots)

        drone_pos = drones_positions["Drone"]
        for drone, position in drones_positions.items():
            if drone != "Drone":
                centroid = np.array(all_drone_centroids[drone]["centroid"])
                count = all_drone_centroids[drone]["count"]
                new_centroid = (count * centroid + np.array(position)) / (count + 1)
                all_drone_centroids[drone]["centroid"] = new_centroid
                all_drone_centroids[drone]["count"] += 1

        if use_radius:
            # calculate radius as the largest distance from the centroid up to now
            for drone, centroid in all_drone_centroids.items():
                drone_position = drones_positions[drone]
                distance = np.linalg.norm(np.array(drone_position) - np.array(centroid["centroid"]))
                if distance > all_drone_radii[drone]:
                    all_drone_radii[drone] = distance

        if use_centroids:
            drones_positions = {drone: all_drone_centroids[drone]["centroid"] for drone in all_drone_centroids.keys()}
            drones_positions["Drone"] = drone_pos


        # for metrics
        total_drone_distance += np.linalg.norm(np.array(drones_positions["Drone"]) - np.array(last_drone_location))
        last_drone_location = drones_positions["Drone"]
        all_drone_locations.append(drones_positions["Drone"])
        
        distances_from_drone = {drone: np.linalg.norm(np.array(drones_positions["Drone"]) - np.array(position))\
                                 for drone, position in drones_positions.items() if drone != "Drone"}
        
        for drone, distance in distances_from_drone.items():
            if distance < EPSILON:
                print(f"Drone is close to {drone}, REMOVING FROM CPU")
                destroyed_drone_locations.append(drones_positions[drone])
                
                # DELETE CRASHED DRONE FROM LISTS
                drones_positions.pop(drone)
                drone_robots.pop(robot_names.index(drone))
                robot_names.remove(drone)
                all_drone_centroids.pop(drone)
                all_drone_radii.pop(drone)

                if len(drones_positions) == 1:
                    print("All drones are removed, STOPPING")
                    end_time = sim_time
                    total_drone_time = end_time - start_time
                    print(f"Total Time: {total_drone_time}")
                    print(f"Total Distance Traveled: {total_drone_distance}")
                    print("Plotting drone movement...")
                    plot_drone_movement(all_drone_locations, destroyed_drone_locations,\
                                         total_drone_time, total_drone_distance, file_path)
                    robot.step(-1)
                    break

        # replan only after removing a drone and 8 seconds have passed
        if current_time - last_replanner > 8 or \
              len(last_drone_positions) != len(drones_positions):
            plan_coords = replan_path(drones_positions, emitter,\
                                       use_random, use_radius, all_drone_radii)
            last_replanner = current_time
            last_drone_positions = drones_positions


if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    run_robot(robot)
