
from abc import abstractmethod
# make sure the graph is bidirectional
RED = "red"
GREEN = "green"
BLUE = "blue"
ORANGE = "orange"
NOCOLOR = "no color"

# Channels
CPU_CHANNEL = 0
DRONE_CHANNEL = 1
ENEMY_DRONE_CHANNEL = 2

# NODES_CHANNEL = 2
# WAITER_CHANNEL = 3
# CLEANER_CHANNEL = 4

class Location:
    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def deep_copy(self):
        pass

class Edge(Location):
    def __init__(self, node1, node2): 
        self.nodes = set([node1, node2])
        self.entity_contained = None

    def __str__(self):
        return str(self.nodes)
    
    def __hash__(self):
        return hash(str(self.nodes))
    
    def __eq__(self, other):
        return self.nodes == other.nodes
    
    def get_nodes(self):
        return self.nodes
    
    def add_entity(self, entity):
        self.entity_contained = entity

    def deep_copy(self):
        return Edge(self.nodes[0].deep_copy(), self.nodes[1].deep_copy())
    
class GraphNode(Location):
    def __init__(self, name=None, color=None):
        self.color = color
        self.name = name
        self.entity_contained = None
        self.fisical_position = None
        self.neighbours = []
        self.edges = []

    def __str__(self):
        return str(self.name)
    
    def __hash__(self):
        return hash(str(self.name))
    
    def __eq__(self, other):
        return self.name == other.name

    def add_entity(self, entity):
        self.entity_contained = entity

    def add_edge(self, edge):
        self.edges.append(edge)
        
    def add_neighbour(self, node):
        if node not in self.neighbours:
            self.neighbours.append(node)
    
    def get_neighbours(self):
        return self.neighbours

    def deep_copy(self):
        new_node = GraphNode(self.name, self.color)
        if self.entity_contained is not None:
            new_node.add_entity(self.entity_contained.deep_copy())
        for neighbour in self.neighbours:
            new_node.add_neighbour(neighbour.deep_copy())
        for edge in self.edges:
            new_node.add_edge(edge.deep_copy())
        return new_node
    
class Entity:
    def __init__(self, name):
        self.name = name
        self.location : Location = None
    
    def deep_copy(self):
        new_entity = Entity(self.name)
        if self.location is not None:
            if isinstance(self.location, GraphNode):
                new_entity.location = self.location.deep_copy()
            elif isinstance(self.location, Edge):
                new_entity.location = self.location.deep_copy()
        return new_entity

class Graph:
    def __init__(self):
        self.nodes = {}
        self.edges = {}
        self.entities = {}
    
    def add_node(self, node_name: str):
        node = GraphNode(node_name)
        self.nodes[node_name] = node

    def add_edge(self, node1_name, node2_name):
        if node1_name not in self.nodes.keys():
            print("Node does not exist")
            return
        if node2_name not in self.nodes.keys():
            print("Node does not exist")
            return

        node1 = self.nodes[node1_name]
        node2 = self.nodes[node2_name]

        if (node1.name, node2.name) in self.edges.keys() or (node2.name, node1.name) in self.edges.keys():
            print("Edge already exists")
            return

        node1.add_edge(node2)
        node2.add_edge(node1)
        node1.add_neighbour(node2)
        node2.add_neighbour(node1)
        self.edges[(node1.name, node2.name)] = Edge(node1, node2)

    def add_entity(self, entity):
        self.entities[entity.name] = entity
        if isinstance(entity.location, GraphNode):
            node_name = entity.location.name
            if node_name in self.nodes.keys():
                self.nodes[node_name].add_entity(entity)
            else:
                print("Node does not exist")
        elif isinstance(entity.location, Edge):
            edge = entity.location
            if edge in self.edges.keys():
                self.edges[edge].add_entity(entity)
            else:
                print("Edge does not exist")
        else:
            print("Invalid location")

    def get_node(self, node_name):
        return self.nodes[node_name]

    def get_edge(self, node1, node2):
        if (node1, node2) in self.edges.keys():
            return self.edges[(node1, node2)]
        elif (node2, node1) in self.edges.keys():
            return self.edges[(node2, node1)]
        else:
            print("Edge does not exist")
            return None
    
    def get_entity(self, entity_name):
        return self.entities[entity_name]
    
    def get_entities(self):
        return self.entities
    
    def get_nodes(self):
        return self.nodes
    
    def get_edges(self):
        return self.edges
    
    def set_node_position(self, node_name, position):
        self.nodes[node_name].fisical_position = position
        
    def deep_copy(self):
        new_graph = Graph()
        for node in self.nodes.values():
            new_graph.add_node(node.name)
        for edge in self.edges.values():
            edge_nodes_ls = list(edge.nodes)
            node1 = edge_nodes_ls[0].name
            node2 = edge_nodes_ls[1].name
            new_graph.add_edge(node1, node2)
        for entity in self.entities.values():
            new_graph.add_entity(entity.deep_copy())
        return new_graph

world_graph = Graph()

world_graph.add_node("tl_1")
world_graph.add_node("tl_2")
world_graph.add_node("ml")
world_graph.add_node("ml_1")
world_graph.add_node("ml_2")
world_graph.add_node("bl_1")
world_graph.add_node("bl_2")
world_graph.add_node("tbl_tl")
world_graph.add_node("tbl_bl")

world_graph.add_node("tr_1")
world_graph.add_node("tr_2")
world_graph.add_node("mr")
world_graph.add_node("mr_1")
world_graph.add_node("mr_2")
world_graph.add_node("br_1")
world_graph.add_node("br_2")
world_graph.add_node("tbl_tr")
world_graph.add_node("tbl_br")

world_graph.add_node("k")
world_graph.add_node("k_in")

world_graph.add_edge("tl_2", "tl_1")
world_graph.add_edge("tl_1", "ml_1")
world_graph.add_edge("ml_1", "bl_1")
world_graph.add_edge("bl_1", "bl_2")
world_graph.add_edge("bl_2", "ml_2")
world_graph.add_edge("ml_2", "ml")
world_graph.add_edge("ml_2", "tl_2")
world_graph.add_edge("ml", "tbl_tl")
world_graph.add_edge("ml", "tbl_bl")
world_graph.add_edge("ml", "ml_1")


world_graph.add_edge("tr_1", "mr_1")
world_graph.add_edge("mr_1", "br_1")
world_graph.add_edge("br_1", "br_2")
world_graph.add_edge("br_2", "mr_2")
world_graph.add_edge("mr_2", "mr")
world_graph.add_edge("mr_2", "tr_2")
world_graph.add_edge("mr", "tbl_tr")
world_graph.add_edge("mr", "tbl_br")
world_graph.add_edge("mr", "mr_1")

world_graph.add_edge("ml_2", "mr_1")
world_graph.add_edge("bl_2", "br_1")

world_graph.add_edge("tr_1", "k")
world_graph.add_edge("tr_2", "k")
world_graph.add_edge("k", "k_in")


def get_graph():
    return world_graph.deep_copy()