import threading
import socket
import sys
import struct
import time
import signal
import numpy as np


class Car:
    # interrupt handler
    def interrupt_handler(self,signal, frame):
        print ('You pressed Ctrl+C! Car will stop in the next 2 seconds ')
        if self.vrep.isAlive():
            self.set_speed(0.,0.)
            self.full_end()
        sys.exit(0)

    def __init__(self):
        # Connect the socket to the port where the server is listening on
        self.server_address = ('localhost', 30100)

        self.distFront = 0.0
        self.distLeft = 0.0
        self.distRight = 0.0
        self.distBack = 0.0
        self.distFrontLeft = 0.0
        self.distFrontRight = 0.0
        self.encoderLeft = 0
        self.encoderRight = 0
        self.speedLeft = 0.0
        self.speedRight = 0.0
        self.heading = 0.0 # new sensor 2020 (magnetic compass)

        #self.debug = True # display debug messages on console
        self.debug = False # do not display debug 

        self.log = 1.0 # controls writing in log file
                       #   0.0 : no log
                       #   1.0 : write log

        self.dtmx = -1.0
        self.dtmn = 1e38
        self.cnt_sock = 0
        self.dta_sock = 0.0
        self.upd_sock = False
        self.car_ready = threading.Event()
        self.car_ready.clear()
 
        # initiate communication thread with V-Rep
        self.simulation_alive = True
        srv = self.server_address
        ev = self.car_ready
        self.vrep = threading.Thread(target=self.vrep_com_socket,args=(srv,ev,))
        self.vrep.start()
        # wait for robot to be ready
        self.car_ready.wait()
        print ("Car ready ...")
        # trap hit of ctrl-x to stop robot and exit (emergency stop!)
        signal.signal(signal.SIGINT, self.interrupt_handler)
        # store start time
        self.rob_start_time = time.time()

    def vrep_com_socket(car,srv,ev):
        while True:
            # Create a TCP/IP socket
            t0 = time.time()
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                sock.connect(srv)
            except:
                print ("Simulation must be alive to execute your python program properly.")
                print ("Type Ctrl-C to exit, start the simulation and then execute your python program.")
                break

            sock.settimeout(0.5)
            vtx = [car.speedLeft,car.speedRight]
            data = [ord(chr(59)),ord(chr(57)),12,0,car.log,vtx[0],vtx[1]]
            strSend = struct.pack('<BBHHfff',data[0],data[1],data[2],data[3],data[4],data[5],data[6]) 
            sock.sendall(strSend)
            upd_sock = True

            data = b''
            try:
                while len(data) < 42:
                    data += sock.recv(42)
            except:
                print ("socker error , duration is %f ms, try to reconnect !!!"%((time.time() - t0)*1000.0))
            if len(data) == 42:
                vrx = struct.unpack('<ccHHfffffffff',data)             
                car.vrep_update_sim_param(upd_sock,vrx)
            else:
                print ("bad data length ",len(data))


            sock.close()
            car.cnt_sock = car.cnt_sock + 1
            tsock = (time.time() - t0)*1000.0
            car.dta_sock += tsock
            if tsock > car.dtmx:
                car.dtmx = tsock
            if tsock < car.dtmn:
                car.dtmn = tsock
            dtm = car.dta_sock/float(car.cnt_sock)
            if car.debug:
                if (car.cnt_sock % 100) == 99:
                    print ("min,mean,max socket thread duration (ms) : ",car.dtmn,dtm,car.dtmx)
            ev.set()

            if not car.simulation_alive:
                break 


    def vrep_update_sim_param(self,upd_sock,vrx):
        self.upd_sock = upd_sock
        self.distFront = vrx[4]
        self.distLeft = vrx[5]
        self.distRight = vrx[6]
        self.distBack = vrx[7]
        self.encoderLeft = int(vrx[10])
        self.encoderRight = int(vrx[11])
        self.heading = vrx[12]
        
    def stop (self):
        """
        Stop the robot by setting the speed of the motors to 0
        """
        self.set_speed(0.,0.)

    def full_end (self):
        """
        Fully stop the simulation of the robot , set motors speed to 0
        and close the connection with the simulator vrep
        sleep for 2 seconds to end cleanly the log file
        """
        self.set_speed(0.,0.)
        self.rob_stop_time = time.time()
        mission_duration = self.rob_stop_time - self.rob_start_time
        time.sleep(2.0)
        print ("clean stop of car")
        print ("mission duration : %.2f"%(mission_duration))
        self.simulation_alive = False

    def set_speed (self,speedLeft,speedRight):
        """
        speedLeft : set speed of left wheel
        speedLeft : set speed of right wheel
        the speed is a value between -100 and +100
        warning the motors have a "dead zone", |speed| must be > 20
        """
        self.speedLeft = float(round(speedLeft))
        self.speedRight = float(round(speedRight))
        self.upd_sock = False
        while not self.upd_sock:
            time.sleep(0.0001)


    def get_odometers (self):
        """
        return the values of ticks counter for the two wheels
        """
        return self.encoderLeft,self.encoderRight

    def get_sonar (self,name):
        """
        return the measurment of the selected sonar (ultrasonic sensor)
        if obstacle between 0.1 m to 1.5 m return the distance to 
        the nearest obstacle, otherwise return 0.0
        distance is returned in meters
        name can be "front","frontleft","frontright","left","right" or "back"
        """
        val = 0.0
        if name == "front":
            val = self.distFront
        elif name == "left":
            val = self.distLeft           
        elif name == "right":
            val = self.distRight
        elif name == "back":
            val = self.distBack
        return val

    def get_multiple_sonars (self,names):
        val = []
        for name in names:
            if name == "front":
                val.append(self.distFront)
            elif name == "left":
                val.append(self.distLeft)           
            elif name == "right":
                val.append(self.distRight)
            elif name == "back":
                val.append(self.distBack)
            elif name == "all":
                val.append(self.distFront)
                val.append(self.distLeft)
                val.append(self.distRight)
                val.append(self.distBack)
        return val

    def get_heading (self):
        return (self.heading)