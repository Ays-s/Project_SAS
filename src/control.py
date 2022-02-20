import numpy as np
import time


class RobotControl:
    '''Classede control du robot'''
    def __init__(self):
        '''Initialise les parametres du robot'''
        self.distBetweenWheels = 0.12
        self.nTicksPerRevol = 240 #720
        self.wheelDiameter = 0.06      

    def cacl_distance_odo(self, nb_ticks):
        return nb_ticks/self.nTicksPerRevol*2*np.pi*self.wheelDiameter
        
    def move_odo(self, rb, dist):
        '''
        Déplacement en ligne droite avec les odometres
        
        Parameters
        ----------
        rb : robot à controler
        dist : float - distance de déplacement 
        '''
        start = rb.get_odometers()
        deltatick = dist*self.nTicksPerRevol/self.wheelDiameter/np.pi #nbr de tick pour tourner de ang
        objtick = (start[0] + deltatick, start[1] +deltatick)
        def move_speed():
            odo = rb.get_odometers()
            delta = (objtick[0]-odo[0])//2 #différence entre valeur actuelle et celle voulue
            rb.set_speed(delta,delta) 
            return delta
        
        loopIterTime = 0.050
        while move_speed():
            time.sleep(loopIterTime) # wait     
        # stop the robot 
        rb.stop()         
        
        
    def correctWallFolow(self, rb, instruction):
        '''
        Corrige la suivie du mur. Utilise le sonar pour se repositionner 
        parallèle au mur. Puis se repositionne aussi en distance. 

        Parameters
        ----------
        rb : robot à controler
        instruction : float - distance au mur souhaité
        '''
        rb.set_speed(0,0)
        loopIterTime = 0.10
        time.sleep(loopIterTime)
        sonar = rb.get_sonar('left')
                
        while not sonar or sonar >1 : #verification que la valeur 0 obtenue pour  
            rb.set_speed(20,-20)      #entrer dans cette fonction n'est aps une erreur 
            time.sleep(loopIterTime) # wait
            sonar = rb.get_sonar('left') 
        odoStart = rb.get_odometers()[0] #valeur initiale de l'odometre
        while rb.get_sonar('left'):      #tourne jusqu'a perdre le sonar
            rb.set_speed(20,-20)
            time.sleep(loopIterTime) # wait
        odoStop = rb.get_odometers()[0] #valeur finale de l'odometre
        objtick = (odoStart+odoStop)//2 #objectif de tick d'odometre
        self.turn_odo_ticks(rb, objtick) #Tourne
        rb.set_speed(0,0)
        dist = self.verif_wall(rb, 'left', 5)[0] #Verifie si on est pas trop loin du mur
        cons = instruction-dist
        if abs(cons)>0.05: #si on est loin on deplace le robot
            self.turn_odo(rb,90)
            self.move_odo(rb, cons)
            self.turn_odo(rb,-90)
            

    def turn_odo(self, rb, ang):
        '''
        Tourne d'un angle précis en fonction en utilisant l'odometre.

        Parameters
        ----------
        rb : robot à controler
        ang : int - angle à tourner en degré
        '''
        start = rb.get_odometers()
        deltatick = ang*self.distBetweenWheels/self.wheelDiameter*self.nTicksPerRevol/360+(ang>0)
        objtick = (start[0] + deltatick, start[1] -deltatick)
        def turn_speed():
            odo = rb.get_odometers()
            delta = (objtick[0]-odo[0])//2
            rb.set_speed(delta,-delta)
            return abs(delta)
        
        loopIterTime = 0.050
        while turn_speed():
            time.sleep(loopIterTime)
        time.sleep(loopIterTime)  # wait     
        # stop the robot 
        rb.stop()                 

                    
    def follow_guideline_PD(self, rb, instruction, kp, kd, valFltr1=lambda x:x, valFltr2=lambda x:x, stop = 0.33):
        '''
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
        '''
        def find_wall():
            d = rb.get_multiple_sonars(('front','left','right'))
            return (d[0] ==0.0 or d[0]>stop) or (d[1] ==0.0 or d[1]>1) or (d[2] ==0.0 or d[2]>1)
        nomspeed = 30
        loopIterTime = 0.10
        val = rb.get_sonar('left')
        dist = valFltr2(valFltr1(val))
        error = instruction - dist #initialisation du correcteur
        lastError = error
        order = (kp*error)
        rb.set_speed(nomspeed+order,nomspeed-order)
        time.sleep(loopIterTime) # wait   
        while find_wall():
            lastval, val = val, rb.get_sonar('left')
            if not val and not lastval: #au cas ou val==0 a cause d'un bug
                self.correctWallFolow(rb, instruction)
                val = rb.get_sonar('left')
            dist = valFltr2(valFltr1(val)) #filtre la valeur du sonar
            error = instruction - dist
            diffError = (error-lastError)/loopIterTime #erreur derivée
            order = kp*error + kd*diffError #ordre de vitesse
            rb.set_speed(nomspeed+order,nomspeed-order)
            lastError = error
            time.sleep(loopIterTime) # wait   
        # stop the robot
        rb.stop()
                    
        
    def find_free_path(self, rb):
        '''
        Trouve le chemin libre (sans mur).

        Parameters
        ----------
        rb : robot à controler
        
        Returns
        -------
        int - angle libre

        '''
        sonars = rb.get_multiple_sonars(('front','left','right'))
        m = [0.0,4]
        face = [0,-90,90,180]
        for i in range(3):
            if sonars[i] == 0.0:
                return face[i]
            elif sonars[i]>m[0]:
                m = [sonars[i],i]
        return face[m[1]]
    
        
    def verif_wall(self, rb, face, nbr):
         '''
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
         '''
         loopIterTime = 0.10
         sonars = np.zeros(nbr)
         for i in range(nbr):
             sonars[i] = rb.get_sonar(face)
             time.sleep(loopIterTime)
         return np.median(sonars),sonars
    
    def verif_multiple_walls(self, rb, face, nbr):
        '''
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
        '''
        loopIterTime = 0.10
        sonars = np.zeros((len(face),nbr))
        for i in range(nbr):
            sonars[:,i] = rb.get_multiple_sonars(face)
            time.sleep(loopIterTime)
        median = [np.median(sonars[i]) for i in range(len(face))]
        return median,sonars