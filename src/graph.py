import sys
from src.node import Node

"""
Graph class : 
    - attributes : 
        - nodes : nodes composing the graph
    
    - Objectives : 
        - represents a Graph composed of Nodes (type Node)
    
    - types specifications : 
        - nodes is a list of Nodes type
        - previous_node and shortest_path are dicts

Invoke class :
    - create an instance by giving a list of already created Nodes instances
    - call dijkstra_algorithm(startingNode) to run dijkstra
    /!\ dijkstra_algorithm RETURNS 2 OBJECTS
        - previous_node : list of path to access to a node from the starting node, 
        each item represent ONE iteration (two nodes that are DIRECTLY linked)
        - shortest_path : shortest distance to every accessible nodes

"""

class Graph(object):
    def __init__(self, nodes):
        self.nodes = nodes

    def get_nodes(self):
        return self.nodes

    def get_outgoing_edges(self, node):
        values =  node.neighbors.values()
        return [value[0] for value in values]
    
    def value(self, node1, node2):
        return node1.distance(node2)
    
    def dijkstra_algorithm(self, start_node):
        unvisited_nodes = self.nodes[:]
        shortest_path = {}
        previous_nodes = {}
        max_value = sys.maxsize

        for node in unvisited_nodes:
            shortest_path[node] = max_value
        shortest_path[start_node] = 0
        
        while unvisited_nodes:
            current_min_node = None
            for node in unvisited_nodes: # Iterate over the nodes
                if current_min_node == None:
                    current_min_node = node
                elif shortest_path[node] < shortest_path[current_min_node]:
                    current_min_node = node
                    
            # The code block below retrieves the current node's neighbors and updates their distances
            neighbors = self.get_outgoing_edges(current_min_node)
            for neighbor in neighbors:
                tentative_value = shortest_path[current_min_node] + self.value(current_min_node, neighbor)
                if tentative_value < shortest_path[neighbor]:
                    shortest_path[neighbor] = tentative_value
                    # We also update the best path to the current node
                    previous_nodes[neighbor] = current_min_node
    
            # After visiting its neighbors, we mark the node as "visited"
            unvisited_nodes.remove(current_min_node)
        return previous_nodes, shortest_path


"""
psueod code

init : turn to free path (or at least not free path behind)

while free path right or left or forward :
    turn to free path
    while not free path right or left:
        move forward
    newnode =  node(self.position)
    distance = distance( self.previousnode, newnode)
    newarc = arc( self.lastnode, newnode, distance, ang)


"""