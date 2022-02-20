"""
Node class : 
    - attributes : 
        - neighbors : neighbors in the graph
    
    - Objectives : 
        - represents a Node in a Graph 
        - represent an entity able to encrypt, send , transmit (as a router), receive a message and store it.

Invoke class :
    - import Node class
    - call addNeighbors to add a Node as a neighboor of another one
"""

class Node():
    nbrNode = 0
    def __init__(self):
        self.neighbors = []
        self.id = Node.nbrNode
        Node.nbrNode += 1
    
    def __str__(self):
        return f'Node : {self.id} - Neighbors = {self.neighbors}'
    
    def __repr__(self):
        return f'Node{self.id}'
    
    def addNeighbors(self, node, distance, angle):
        if node.id not in [arc.end_node.id for arc in self.neighbors]:
            self.neighbors.append(Arc(self, node, distance, angle))
            return True
        return False
    
    def distance(self, node):
        for arc in self.neighbors:
            if node.id == arc.end_node.id:
                return arc.distance
        return -1   


class Arc():
    def __init__(self, start_node, end_node, distance, angle):
        self.start_node = start_node
        self.end_node = end_node
        self.distance = distance
        self.angle = angle

    def __str__(self):
        return f'Arc : from {self.start_node} to {self.end_node}, d={self.distance:.2f}m, ang={self.angle}°'

    def __repr__(self):
        return f'Arc(from={self.start_node.id}, to={self.end_node.id}, d={self.distance:.2f}m, ang={self.angle}°)'