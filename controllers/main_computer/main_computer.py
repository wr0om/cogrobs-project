"""odometer_calculation controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import sys
import os
libraries_path = os.path.abspath('../my_utils')
sys.path.append(libraries_path)

from controller import Robot, Emitter, Receiver
from classes_and_constans import RED, GREEN, BLUE, ORANGE, NOCOLOR, WAITER_CHANNEL, CLEANER_CHANNEL, DRONE_CHANNEL, NODES_CHANNEL
from classes_and_constans import Location, Edge, GraphNode, Entity, Graph
from classes_and_constans import get_graph
import math
import ast

graph = get_graph()
entities_channels = [DRONE_CHANNEL]

got_nodes_position = False
sent_possitions_to_entities = False
verified_entities_got_positions = False

def get_nodes_position(graph ,receiver):
    nodes = graph.get_nodes()
    len_nodes = len(nodes)
    non_null_count = 0
    while receiver.getQueueLength() > 0:
            message = receiver.getString()
            if message not in [None, ""]:
                non_null_count += 1
                data = ast.literal_eval(message)  # Safely parse the string back into a tuple
                name, position = data
                graph.set_node_position(name, position)
                print(f"CPU Received position from {name}: {position}")
                receiver.nextPacket()
    
    print(non_null_count, len_nodes)
    if non_null_count == len_nodes:
        print("CPU: All nodes position received from sensors")
        global got_nodes_position
        got_nodes_position = True

def send_possitions_to_entities(graph, emitter):
    nodes = graph.get_nodes()

    for entity_channel in entities_channels:
        for node_name, node in nodes.items():
            message = (node_name, node.fisical_position)
            emitter.setChannel(entity_channel)
            emitter.send(str(message).encode('utf-8'))
        print(f"All nodes position sent to {entity_channel}")
    
    print("CPU: All nodes position sent to entities")
    global sent_possitions_to_entities
    sent_possitions_to_entities = True

def verify_entities_got_positions(receiver):
    all_channels = set(entities_channels)
    collected_channels = set()
    for channel in entities_channels:
        while receiver.getQueueLength() > 0:
            message = receiver.getString()
            if message not in [None, ""]:
                collected_channels.add(int(channel))
            receiver.nextPacket()
    
    if all_channels == collected_channels:
        print("CPU: entities received positions successfully")
        global verified_entities_got_positions
        verified_entities_got_positions = True

def run_robot(robot):
    # get the time step of the current world.
    timestep = 64

    # Get the emitter device
    emitter = robot.getDevice('emitter')
    receiver = robot.getDevice('receiver')
    receiver.enable(timestep)

    # define channels for communication
    
    while robot.step(timestep) != -1:
        if not got_nodes_position:
            get_nodes_position(graph, receiver)

        if got_nodes_position and not sent_possitions_to_entities:
            send_possitions_to_entities(graph, emitter)

        if sent_possitions_to_entities and not verified_entities_got_positions:
            verify_entities_got_positions(receiver)

if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    run_robot(robot)
    

    
