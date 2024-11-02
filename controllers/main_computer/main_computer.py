"""odometer_calculation controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import sys
import os
import math
import ast
import time
import numpy as np

libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)
from controller import Robot, Supervisor
from classes_and_constants import DRONE_CHANNEL, CPU_CHANNEL, ENEMY_DRONE_CHANNEL, EPSILON
from planner import DronePathFinder
from functions import *


def replan_path(drones_positions, emitter):
    # get list of positions from drones, but "Drone" is the first element of the list
    drone_position = drones_positions["Drone"]
    position_list = [drone_position] + [position for drone, position in drones_positions.items() if drone != "Drone"]
    position_list = [coords_to_string(position) for position in position_list]

    path_finder = DronePathFinder(position_list)
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
    return planned_path, plan_coords

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
    #replan_path(drones_positions, emitter)

    last_replanner = time.time()
    last_drone_positions = drones_positions
    current_plan = None
    goal_location = None

    while robot.step(timestep) != -1:
        current_time = time.time()
        drones_positions = get_enemy_drones_positions(robot_names, drone_robots)
        distances_from_drone = {drone: np.linalg.norm(np.array(drones_positions["Drone"]) - np.array(position))\
                                 for drone, position in drones_positions.items() if drone != "Drone"}
        for drone, distance in distances_from_drone.items():
            if distance < EPSILON:
                print(f"Drone is close to {drone}, REMOVING FROM CPU")
                # DELETE DRONE FROM LISTS
                drones_positions.pop(drone)
                drone_robots.pop(robot_names.index(drone))
                robot_names.remove(drone)

        # replan only after removing a drone and 5 seconds have passed
        if current_time - last_replanner > 5 or len(last_drone_positions) != len(drones_positions):
            current_plan, plan_coords = replan_path(drones_positions, emitter)
            last_replanner = current_time
            last_drone_positions = drones_positions


if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    run_robot(robot)
    
