import argparse
import numpy as np
import time

from src.car import Car  # get robot simulator
from src.control import RobotControl
from src.filtre import * 
from src.node import Node

# Constante pour PID
KP = 20
KI = 0
KD = 5
dist_to_wall = 20

if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog='SAS Robot project')
    parser.add_argument('--showgraph', action='store_true')
    parser.add_argument('--shownodes', action='store_true')
    parser.add_argument('--savecsv', action='store_true')
    parser.add_argument('-kp', type=float, help='Coefficient protionel', default=None)
    parser.add_argument('-kd', type=float, help='Coefficient intégral', default=None)
    parser.add_argument('-ki', type=float, help='Coefficient dérivé ', default=None)
    args = parser.parse_args()

    if args.kp is None: kp = KP
    else: kp = args.kp
    if args.kd is None: kd = KD
    else: kd = args.kd
    if args.ki is None: ki = KI
    else: ki = args.ki

    # instancie le robot et le controleur
    rb = Car()
    ctrl = RobotControl()

    # prepare la gestions des noeuds
    startingNode = Node()
    nodes = [startingNode]

    traces = []

    print("Mission started.")
    free_ang = 0
    while free_ang != 180:
        face_to_folow = ctrl.face_to_folow(rb)
        last_odometers = sum(rb.get_odometers())//2
        trace = ctrl.follow_guideline_PID(rb, 
                            rb.get_sonar(face_to_folow), face_to_folow, 
                            kp, kd, ki,
                            nomspeed = 40, 
                            stop = 0.25, 
                            graph=(args.showgraph or args.savecsv))
        free_ang = ctrl.turn_to_free_path(rb)
        front_heading = rb.get_heading()
        distance = ctrl.cacl_distance_odo(abs(sum(rb.get_odometers())//2 - last_odometers))
        newNode = Node()
        newNode.addNeighbors(nodes[-1], distance, -free_ang)
        nodes[-1].addNeighbors(newNode, distance, free_ang)
        nodes.append(newNode)
        print("New Node added.")
        traces.append(trace)

    rb.full_end() #fin du circuit

    if args.shownodes:
        for node in nodes:
            print(node)
    if args.savecsv:
        for trace in traces:
            with open('resultat.csv', 'a') as file:
                file.write(f'Kp={kp}, Ki={ki} ,Kd={kd}:'+str(trace)[1:-1].replace(', ',':').replace('.',',')+'\n')
    if args.showgraph:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.title(f'Trace for Kp={kp}, Ki={ki} ,Kd={kd}')
        for trace in traces:
            plt.plot(trace)
        plt.show()

    