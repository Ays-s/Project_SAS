import numpy as np
import time
from src.utils import nearest_angle


class RobotControl:
    """Classe de control du robot"""
    def __init__(self):
        """Initialise les parametres du robot"""
        self.distBetweenWheels = 0.12
        self.nTicksPerRevol = 240 #720
        self.wheelDiameter = 0.06      

    def cacl_distance_odo(self, nb_ticks):
        return nb_ticks/self.nTicksPerRevol*2*np.pi*self.wheelDiameter
        
    def move_odo(self, rb, dist):
        """
        Déplacement en ligne droite avec les odometres
        
        Parameters
        ----------
        rb : robot à controler
        dist : float - distance de déplacement 
        """
        start = rb.get_odometers()
        deltatick = dist*self.nTicksPerRevol/self.wheelDiameter/np.pi #nbr de tick pour tourner de ang
        objtick = (start[0] + deltatick, start[1] +deltatick)
        def move_speed():
            odo = rb.get_odometers()
            delta = (objtick[0]-odo[0])//2 #différence entre valeur actuelle et celle voulue
            rb.set_speed(delta, delta) 
            return delta
        
        loopIterTime = 0.050
        while move_speed():
            time.sleep(loopIterTime) # wait     
        # stop the robot 
        rb.stop()         
        
        
    def turn_odo(self, rb, ang):
        """
        Tourne d'un angle précis en fonction en utilisant l'odometre.

        Parameters
        ----------
        rb : robot à controler
        ang : int - angle à tourner en degré
        """
        start = rb.get_odometers()
        deltatick = ang*self.distBetweenWheels/self.wheelDiameter*self.nTicksPerRevol/360+(ang>0)
        objtick = (start[0] + deltatick, start[1] -deltatick)
        def turn_speed():
            odo = rb.get_odometers()
            delta = (objtick[0]-odo[0])//2
            rb.set_speed(delta, -delta)
            return abs(delta)
        
        loopIterTime = 0.050
        while turn_speed():
            time.sleep(loopIterTime)
        time.sleep(loopIterTime)  # wait     
        # stop the robot 
        rb.stop()                 

                    
    def follow_guideline_PID(self, rb, instruction, face, kp, kd, ki,
                            nomspeed = 30, 
                            valFltr1=lambda x:x, valFltr2=lambda x:x, 
                            stop = 0.33, 
                            graph=False):
        """
        Fonction de suivi de mur. Permet de suivre le mur gauche.
        Utilise un correcteur en boucle fermé Proportionnelle Dérivée.

        Parameters
        ----------
        rb : robot à controler
        instruction : foat - distance voulue
        kp : int - gain proportionnel
        kd : int - gain dérivée
        valFltr1 : Premier filtre, optional
            filtre les valeur obtenues. The default is lambda x:x.
        valFltr2 : Deuxieme filtre, optional
            The default is lambda x:x.
        stop : float, optional
            Distance nécessaire pour s'arreter. The default is 0.33.
        """
        print(f'Folowing {face}...')
        other = {'left':'right', 'right':'left'}
        orderSens = {'left':-1, 'right':1}
        sumError = 0
        def find_wall():
            d = rb.get_sonar('front')
            return d == 0.0 or d>stop
        loopIterTime = 0.10
        val = rb.get_sonar(face)
        if graph:
            sonars = [val]
        dist = valFltr2(valFltr1(val))
        error = instruction - dist #initialisation du correcteur
        sumError += error
        lastError = error
        order = kp*error
        rb.set_speed(nomspeed+order, nomspeed-order)
        time.sleep(loopIterTime) # wait   
        while find_wall():
            val = rb.get_sonar(face)
            if not val : #au cas ou val==0 a cause d'un bug 
                if self.correct_wall(rb, face):
                    face = other[face]
                    print(f'Folowing {face}...')
                    instruction = rb.get_sonar(face)
                    val = rb.get_sonar(face)
                    sumError = 0
                val = rb.get_sonar(face)
            if graph:
                sonars.append(val)
            dist = valFltr2(valFltr1(val)) #filtre la valeur du sonar
            error = instruction - dist
            diffError = (error-lastError)/loopIterTime #erreur derivée
            sumError += error #erreur intégrale
            order = orderSens[face]*(kp*error + kd*diffError) # + ki*sumError #ordre de vitesse
            rb.set_speed(nomspeed-order, nomspeed+order)
            lastError = error
            time.sleep(loopIterTime) # wait   
        # stop the robot
        rb.stop()
        if graph:
            return sonars 
                    
        
    def find_free_path(self, rb):
        """
        Trouve le chemin libre (sans mur).

        Parameters
        ----------
        rb : robot à controler
        
        Returns
        -------
        int - angle libre
        """
        sonars = rb.get_multiple_sonars(('left', 'right', 'front'))
        m = [0.50, 3]
        face = [-90, 90, 0, 180]
        for i in range(3):
            if sonars[i] == 0.0:
                return face[i]
            elif sonars[i] > m[0]:
                m = [sonars[i], i]
        return face[m[1]]
    
        
    def verif_wall(self, rb, face, nbr):
         """
         Verifie la distance au mur du sonar "face".

         Parameters
         ----------
         rb : robot à controler
         face : str - face du robot à controler
         nbr : int - nombre de valeurs de sonnar

         Returns
         -------
         float - medianne des valeurs du sonars
         np.array - valeurs du sonar
         """
         loopIterTime = 0.050
         sonars = np.zeros(nbr)
         for i in range(nbr):
             sonars[i] = rb.get_sonar(face)
             time.sleep(loopIterTime)
         return np.median(sonars), sonars
    
    def verif_multiple_walls(self, rb, face, nbr):
        """
        Verifie la distance au mur des sonars de "face".

        Parameters
        ----------
        rb : robot à controler
        face : tuple - tuple des faces du robot à controler
        nbr : int - nombre de valeurs de sonnar

        Returns
        -------
        list - liste des mediannes des valeurs du sonars
        np.array - valeurs du sonar
        """
        loopIterTime = 0.050
        sonars = np.zeros((len(face), nbr))
        for i in range(nbr):
            sonars[:, i] = rb.get_multiple_sonars(face)
            time.sleep(loopIterTime)
        median = [np.median(sonars[i]) for i in range(len(face))]
        return median,sonars

    def turn_to_free_path(self, rb):
        free_ang = self.find_free_path(rb)
        heading = rb.get_heading()
        angToTurn = free_ang + nearest_angle(heading) - heading
        while angToTurn > 180: angToTurn -= 360
        while angToTurn <-180: angToTurn += 360
        self.turn_odo(rb, angToTurn)
        return nearest_angle(angToTurn)

    def face_to_folow(self, rb):
        face_to_folow_dico = {0:'left', -90:'right', 90:'left' }
        return face_to_folow_dico[self.find_free_path(rb)]
    
    def correct_wall(self, rb, face, cmax = 10):
        loopIterTime = 0.050
        c = 0
        time.sleep(loopIterTime)
        if rb.get_sonar(face):
            return False
        rb.set_speed(10, -10)
        while rb.get_sonar(face) == 0.0 and c < cmax:
            c+=1
            time.sleep(loopIterTime)
        if rb.get_sonar(face):
            return False
        c=0
        rb.set_speed(-10, 10)
        while rb.get_sonar(face) == 0.0 and c < cmax*2:
            c+=1
            time.sleep(loopIterTime)
        if rb.get_sonar(face):
            return False
        rb.set_speed(10, -10)
        time.sleep(loopIterTime*c)  
        return True
