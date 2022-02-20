from src.car import Car  # get robot simulator
from src.control import RobotControl
from src.filtre import * 
from src.node import Node
import numpy as np
import time


if __name__ == "__main__":
    rb = Car()
    ctrl = RobotControl()
    startingNode = Node()

    nodes = [startingNode]
    free_ang = ctrl.find_free_path(rb)
    if free_ang != 0: ctrl.turn_odo(rb, free_ang)
    front=  rb.get_sonar('front')
    front_heading = 90
    lastodo = sum(rb.get_odometers())//2
    rb.set_speed(100., 100.)
    while front == 0.0 or front > 0.35:
        front=  rb.get_sonar('front')
        free_ang = ctrl.find_free_path(rb)
        if free_ang != 0:
            rb.set_speed(0., 0.)
            ctrl.move_odo(rb, 0.15)
            distance = ctrl.cacl_distance_odo(abs(sum(rb.get_odometers())//2 - lastodo))
            newNode = Node()
            newNode.addNeighbors(nodes[-1], distance, -free_ang)
            nodes[-1].addNeighbors(newNode, distance, free_ang)
            nodes.append(newNode)
            angToTurn = free_ang + front_heading - rb.get_heading()
            while angToTurn > 180: angToTurn -= 360
            while angToTurn <-180: angToTurn += 360
            ctrl.turn_odo(rb, angToTurn)
            lastodo = sum(rb.get_odometers())//2

            front_heading += free_ang
            while front_heading >= 360: front_heading -= 360
            while front_heading <= 0 : front_heading += 360 

            rb.set_speed(100., 100.)     
        else: 
            time.sleep(0.05)
    rb.stop()

    rb.full_end() #fin du circuit

    for node in nodes:
        print(node)   
    