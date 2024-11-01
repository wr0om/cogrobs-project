"""odometer_calculation controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import sys
import os
import math
import ast
import time


libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)
from controller import Robot, Emitter, Receiver
from classes_and_constants import DRONE_CHANNEL, CPU_CHANNEL
from classes_and_constants import Location, Edge, GraphNode, Entity, Graph
from classes_and_constants import get_graph
from planner import DronePathFinder
from functions import *


# graph = get_graph()
# entities_channels = [DRONE_CHANNEL]

# got_nodes_position = False
# sent_possitions_to_entities = False
# verified_entities_got_positions = False

# def get_nodes_position(graph ,receiver):
#     nodes = graph.get_nodes()
#     len_nodes = len(nodes)
#     non_null_count = 0
#     while receiver.getQueueLength() > 0:
#             message = receiver.getString()
#             if message not in [None, ""]:
#                 non_null_count += 1
#                 data = ast.literal_eval(message)  # Safely parse the string back into a tuple
#                 name, position = data
#                 graph.set_node_position(name, position)
#                 #print(f"CPU Received position from {name}: {position}")
#                 receiver.nextPacket()
    
#     #print(non_null_count, len_nodes)
#     if non_null_count == len_nodes:
#         #print("CPU: All nodes position received from sensors")
#         global got_nodes_position
#         got_nodes_position = True

# def send_possitions_to_entities(graph, emitter):
#     nodes = graph.get_nodes()

#     for entity_channel in entities_channels:
#         for node_name, node in nodes.items():
#             message = (node_name, node.fisical_position)
#             emitter.setChannel(entity_channel)
#             emitter.send(str(message).encode('utf-8'))
#         #print(f"All nodes position sent to {entity_channel}")
    
#     #print("CPU: All nodes position sent to entities")
#     global sent_possitions_to_entities
#     sent_possitions_to_entities = True

# def verify_entities_got_positions(receiver):
#     all_channels = set(entities_channels)
#     collected_channels = set()
#     for channel in entities_channels:
#         while receiver.getQueueLength() > 0:
#             message = receiver.getString()
#             if message not in [None, ""]:
#                 collected_channels.add(int(channel))
#             receiver.nextPacket()
    
#     if all_channels == collected_channels:
#         #print("CPU: entities received positions successfully")
#         global verified_entities_got_positions
#         verified_entities_got_positions = True



def get_drone_positions(receiver):
    drones_positions = {}
    new_names = set()
    while receiver.getQueueLength() > 0:
        message = receiver.getString()
        if message not in [None, ""]:
            data = ast.literal_eval(message)  # Safely parse the string back into a tuple
            name, position = data
            drones_positions[name] = position
            if name in new_names:
                #print("Got repeated name, stopping receiver.")
                break
            new_names.add(name)
        receiver.nextPacket()
    return drones_positions

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
        goal_location = path_finder.get_next_coords()
        print(f"goal_location: {goal_location}") 
        # TODO: send goal_location to Drone 
        message = ("CPU", goal_location)
        send_msg_to_drone(emitter, message)
    else:
        print("ERROR!!!! No plan found!")
    return planned_path, goal_location

def run_robot(robot):
    # get the time step of the current world.
    timestep = 64

    # Get the emitter device
    emitter = robot.getDevice('emitter')
    receiver = robot.getDevice('receiver')
    receiver.enable(timestep)
    receiver.setChannel(CPU_CHANNEL)

    last_replanner = time.time()
    last_drone_positions = drones_positions = {}
    current_plan = None
    goal_location = None

    while robot.step(timestep) != -1:
        current_time = time.time()
        if receiver.getQueueLength() > 0:
            drones_positions = get_drone_positions(receiver)
            #print(f"CPU got drones positions: {drones_positions}")

        if current_time - last_replanner > 5 and last_drone_positions != drones_positions \
            and len(drones_positions) > 1 and "Drone" in drones_positions.keys():
            current_plan, goal_location = replan_path(drones_positions, emitter)
            last_replanner = current_time
            last_drone_positions = drones_positions


if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    run_robot(robot)
    

    
