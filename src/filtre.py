class Filter(list):
    '''Classe de filtre'''
    def __init__(self, values): 
        """ Initialise le filtre :
        Values : list - valeur d'initialisation du filtre
        vf : float - derniere valeur vraie
        """
        self += values
        self.vf = 0
 
    def process(self, value):
        self.append(value)
        self._vf = value
        return value


class lmedianFilter(Filter):
    def __init__(self, values, coefMed): 
        """ Initialise le filtre :
        coefMed : int - ordre du filtre median
        """
        super().__init__(values)
        self.coefMed = coefMed
      
    def process(self, v):
        '''
        Calcul du filtre median.

        Parameters
        ----------
        v : float - valeur a filtrer

        Returns
        -------
        float - valeur filtrée
        '''        
        self.append(v)
        temp = self[-(2*self.coefMed+1):]
        temp.sort()
        self.vf = temp[self.coefMed]
        return self.vf

class averageFilter(Filter):
    def __init__(self, values, coefAvg): 
        """ Initialise le filtre :
        coefAvg : int - ordre du filtre en moyenne glissante
        """
        super().__init__(values)
        self.coefAvg = coefAvg 
              
    def process(self, v):
        '''
        Calcul du filtre en moyenne glissante.

        Parameters
        ----------
        v : float - valeur a filtrer

        Returns
        -------
        float - valeur filtrée
        '''   
        self.append(v)
        self.vf = sum(self[-self.coefAvg:])/self.coefAvg
        return self.vf


class averageFilter(Filter):
    def __init__(self, values, coefRec): 
        """ Initialise le filtre :
        coefRec : float - coeficient du filtre récursif
        """
        super().__init__(values)
        self.coefRec = coefRec 

    def process(self, v):
        '''
        Calcul du filtre recursif.

        Parameters
        ----------
        v : float - valeur a filtrer

        Returns
        -------
        float - valeur filtrée
        '''   
        self.vf = self.coefRec*v + (1-self.coefRec)*self.vf
        return self.vf
 

