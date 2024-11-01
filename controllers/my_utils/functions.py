
from classes_and_constants import CPU_CHANNEL, DRONE_CHANNEL
from classes_and_constants import Location, Edge, GraphNode, Entity, Graph
from classes_and_constants import get_graph

import math
import ast

channels_to_str = {CPU_CHANNEL: "CPU", DRONE_CHANNEL: "DRONE"}

def get_positions_graph_from_cpu(receiver, emitter, graph, got_positions = False):
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
                print(f"{channels_to_str[receiver.getChannel()]} Received position {name}: {position} from CPU")
                receiver.nextPacket()
    
    print(non_null_count, len_nodes)
    if non_null_count == len_nodes:
        print("All nodes position received")
        got_positions = True
        # send verification to cpu
        message = emitter.getChannel()
        emitter.setChannel(CPU_CHANNEL)
        emitter.send(str(message).encode('utf-8'))
    else:
        got_positions = False
    
    return got_positions

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
