import time

import numpy as np

from igraph import *
from igraph.drawing import graph

from measuringBot import MeasuringBot

from utilities import segmentsIntersect, distObj, minDistObjList

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


class Tile():

    def __init__(self, x, y, width, no_history):

        self.x = x # 
        self.y = y # coordonnées du CENTRE
        self.center = (x,y)
        self.width = width
        self.height = width # square tiles

        self.corners = [[x-self.width//2,y-self.height//2],[x+self.width//2,y-self.height//2],[x+self.width//2,y+self.height//2],[x-self.width//2,y+self.height//2]]
        self.polygon = Polygon([[x-self.width//2,y-self.height//2],[x+self.width//2,y-self.height//2],[x+self.width//2,y+self.height//2],[x-self.width//2,y+self.height//2]])
        
        self.seen = 0
        self.covered = 0
        self.obstacle = 0
        self.measured = 0

        self.containsWall = 0
        self.has_changed = False

        self.state = ''.join(str(e) for e in [self.seen,self.covered,self.obstacle,self.measured]) # on stocke l'état sous la forme d'une châine de caractères

        self.graph_status = 0

        # Metriques
        self.metrics_coord = (0,0)
        self.history = [(self.seen,self.covered,self.obstacle,self.measured)]

        self.no_history = no_history

        self.nbVisits = 0


        #################################################################################################################
        # CATALOGUE DES ETATS #                                                                                         #
        #######################                                                                                         #
        #                                                                                                               #
        # 00-- : non vu, non couvert (UWB)                      couleur : transparent (invisible)   (0,0,0,0)           #
        # 01-- : non vu, couvert (UWB)                          couleur : blanc                     (255,255,255,100)   #
        # 1000 : vu, non couvert (UWB), pas d'obstacle          couleur : orange                    (200,100,0,100)     #
        # 101- : vu, non couvert (UWB), obstacle                couleur : rouge                     (200,0,0,100)       #
        # 111- : vu et couvert, obstacle                        couleur : rouge                     (200,0,0,100)       #
        # 1100 : vu et couvert, pas d'obstacle, pas mesuré      couleur : jaune                     (200,200,0,100)     #
        # 1101 : vu et couvert, pas d'obstacle, mesuré          couleur : vert                      (0,200,0,100)       #
        # 1001 : vu et plus couvert, déjà mesuré                couleur : violet                    (200,0,200,100)     #
        #                                                                                                               #
        #################################################################################################################




    def updateDiscrete(self,walls,measuringBot,refPointBots,status,graph_status_dictionary):

        self.has_changed = False
        oldState = self.state

        # On vérifie si la case a été vue. Si oui, elle restera vue.
        if not self.seen and distObj(self,measuringBot) <= measuringBot.radiusDetection:
            if self.isSeen(walls,measuringBot):
                self.seen = 1

        if status in ["init","FirsttransferRefPointBotToMeasuringBot","transferRefPointBotToMeasuringBot", "resetRefPointBot1stStep"]:
            # On vérifie si la case est couverte
            if self.isCovered(walls, refPointBots):
                self.covered = 1
            else:
                self.covered = 0

        # On vérifie si la case contient un obstacle
        # Si la case contient un mur, ça ne vas pas changer.
        if not self.containsWall and False: # on ne vérifie pas pour l'instant
            if self.UWBbotInTile(refPointBots):
                self.obstacle = 1
            else:
                self.obstacle = 0


        if not self.no_history:
            self.history.append((self.seen,self.covered,self.obstacle,self.measured))

        self.state = ''.join(str(e) for e in [self.seen,self.covered,self.obstacle,self.measured])
        if self.state != oldState:
            self.has_changed = True


        # mise à jour du graphe
        self.graph_status = graph_status_dictionary[self.state]


    def UWBbotInTile(self,bots):
        for bot in bots:
            if not isinstance(bot,MeasuringBot):
                point = Point(bot.x,bot.y)
                if self.polygon.contains(point):
                    return True
        return False


    def isSeen(self,walls,measuringBot):

        if self.containsWall:
            for corner in self.corners:
                cornerInView = True
                for wall in walls:
                    if wall.visibleForBot(measuringBot):
                        intersection = segmentsIntersect([(corner[0],corner[1]),(measuringBot.x,measuringBot.y)],[(min(wall.Xs),min(wall.Ys)),(max(wall.Xs),max(wall.Ys))])
                        if intersection is not None:
                            cornerInView = False
                if cornerInView:
                    return True

        else:
            for wall in walls:
                if wall.visibleForBot(measuringBot):
                    intersection = segmentsIntersect([(self.x,self.y),(measuringBot.x,measuringBot.y)],[(min(wall.Xs),min(wall.Ys)),(max(wall.Xs),max(wall.Ys))])
                    if intersection is not None:
                        return False

            return True


    def isCovered(self,walls,refPointBots):
        UWB_in_range = 0
        for bot in refPointBots:
            inView = True
            for wall in walls:
                if wall.visibleForBotUWB(bot):
                    intersection = segmentsIntersect([(self.x,self.y),(bot.x,bot.y)],[(min(wall.Xs),min(wall.Ys)),(max(wall.Xs),max(wall.Ys))])
                    if intersection is not None:
                        inView = False

            if inView and distObj(self,bot) <= bot.UWBradius:
                UWB_in_range += 1

            if UWB_in_range == 3:
                return True

        return False



class Grid():
    def __init__(self,room,measuringBot,refPointBots,tileWidth=50):
        self.time = time.time()

        self.room = room
        self.tileWidth = tileWidth
        self.tiles = {}

        self.refPointBots = refPointBots

        self.measuringBot = measuringBot
        self.oldObjective = measuringBot.objective

        # Initialisation du graphe 
        self.graph = {}
        self.graphLinks = []
        self.adjacencyList = {}

        # Metriques
        self.surface = 0
        self.no_history = False


        
        ### Construction de toute la grille

        # on trouve la zone de l'écran intéressante (zone contenue entre les murs les plus éloignés)

        Xmin, Xmax, Ymin, Ymax = self.room.Xmin, self.room.Xmax, self.room.Ymin, self.room.Xmax

        i = 0
        while Xmin + i*tileWidth <= Xmax + tileWidth:
            j = 0
            while Ymin + j*tileWidth <= Ymax + tileWidth:
                self.tiles[(Xmin + i*tileWidth,Ymin + j*tileWidth)] = Tile(Xmin + i*tileWidth, Ymin + j*tileWidth, self.tileWidth, self.no_history)
                self.tiles[(Xmin + i*tileWidth,Ymin + j*tileWidth)].metrics_coord = (i,j)


                j += 1
            i += 1
            
        
        # On cherche toutes les cases qui contiennent un mur
        obstacles = self.room.obstacles
        for coord in self.tiles:
            k = 0
            while not self.tiles[coord].containsWall and k < len(obstacles):
                obs = obstacles[k]
                point = Point([obs.x,obs.y])
                if self.tiles[coord].polygon.contains(point):
                    self.tiles[coord].containsWall = 1
                    self.tiles[coord].obstacle = 1
                k += 1


        # On définit l'intérieur de la salle
        coordinates = list(self.tiles.keys())
        self.origin = minDistObjList(self.measuringBot,coordinates)
        self.inside = self.findCluster(self.origin)     


        # On nettoie les objets Tile, pour ne pas conserver de cases inutiles (ie on supprime toutes les cases extérieures, mais on garde les murs pour affichage)
        # On crée un noeud dans le graphe pour toutes les cases d'intérieur

        for coord in coordinates:
            if coord in self.inside:

                self.graph[coord] = 0 # création des noeuds
                self.surface += 1
            else :
                if self.tiles[coord].containsWall == 0: # toute case qui ne sera pas un noeud du graphe et ne contient pas de mur est inutile
                    del self.tiles[coord]


        # Dictionnaire des couleurs des cases en fonction des états
        self.graph_status_dictionary = {
            '0000' : 0,
            '0001' : 0,
            '0010' : 0,
            '0011' : 0,
            '0100' : 0,
            '0101' : 0,
            '0110' : 0,
            '0111' : 0,
            '1000' : 2,
            '1001' : 1.5,
            '1010' : -1,
            '1011' : -1,
            '1110' : -1,
            '1111' : -1,
            '1100' : 0.5,
            '1101' : 1
        }

        self.initDuration = time.time() - self.time


    # Méthode pour récupérer les métriques à la fin
    def get_metrics(self):
        # Nombre de cases mesurées
        measuredTiles = 0

        # Longueur (en cases du parcours):
        pathLength = 0

        # Historique des états de chacune des cases :
        history = {}

        # Nombre de passages par case :
        visitsPerTile = {}

        # Calcul 
        for coord in self.tiles:
            if self.tiles[coord].measured == 1:
                measuredTiles += 1

            if not self.no_history:
                history[self.tiles[coord].metrics_coord] = self.tiles[coord].history

            tmp = self.tiles[coord].nbVisits
            visitsPerTile[self.tiles[coord].metrics_coord] = tmp

            pathLength += tmp
    

        return measuredTiles, self.surface, pathLength, history, visitsPerTile


    # Méthode utilisée pour détecter un ensemble de cases connectées (non interrompues par un mur)
    def findCluster(self,start):
        cluster = []
        neighbours = []

        straight, diag = self.getNeighbours(start)
        tmp = straight + diag
        for coord in tmp:
            if coord in self.tiles and self.tiles[coord].containsWall == 0:
                neighbours.append(coord)
        
        while neighbours != []:
            current = neighbours.pop()
            cluster.append(current)
            straight, diag = self.getNeighbours(current)
            tmp = straight + diag
            for coord in tmp:
                if not coord in neighbours and not coord in cluster: 
                    if coord in self.tiles and self.tiles[coord].containsWall == 0:
                        neighbours.append(coord)
        
        return cluster


    ### Méthodes pour la grille

    def update(self,status):

        # Pour ce qui est de la mesure, le changement de valeur doit venir du robot mesureur.
        # Une case a été mesurée si le robot a changé d'objectif
        if status == "movingMeasuringBot" and self.measuringBot.objective != None:
            if self.measuringBot.objective != self.oldObjective:
                self.tiles[tuple(self.measuringBot.objective)].measured = 1
                self.tiles[tuple(self.measuringBot.objective)].nbVisits += 1

        self.oldObjective = self.measuringBot.objective        

        for coord in self.tiles:

            self.tiles[coord].updateDiscrete(self.room.walls,self.measuringBot,self.refPointBots,status,self.graph_status_dictionary) 
            self.graph[coord] = self.tiles[coord].graph_status
            if self.tiles[coord].has_changed:
                self.updateNeighOneNode(coord)
            if self.graph[coord] == -1 or self.graph[coord] == 2 or self.graph[coord] == 1.5:
                self.removeNodeFromGraph(coord)


    ### Méthodes pour le graphe
    def getNeighbours(self, coord):
        x,y = coord
        w = self.tileWidth
        coordLeft = (x-w, y)
        coordRight = (x+w, y)
        coordTop = (x, y-w)
        coordBottom = (x, y+w)
        coordTopLeft = (x-w, y-w)
        coordTopRight = (x+w, y-w)
        coordBottomRight = (x+w, y+w)
        coordBottomLeft= (x-w, y+w)
        coordsStraight = [coordLeft, coordRight, coordTop,coordBottom]
        coordsDiag = [coordTopLeft, coordTopRight, coordBottomRight, coordBottomLeft]
        
        return coordsStraight,coordsDiag


    def updateNeighOneNode(self, coord):
        
        coordsStraight,coordsDiag = self.getNeighbours(coord)

        for neigh in coordsStraight:
            if neigh in self.graph:
                if self.graph[neigh] == 0.5 or self.graph[neigh] == 1:
                    if (coord, neigh) not in self.graphLinks and (neigh, coord) not in self.graphLinks:
                        self.graphLinks.append((coord, neigh))
                    if coord not in self.adjacencyList:
                        self.adjacencyList[coord] = [(neigh, 1)]
                    else:
                        if neigh not in self.adjacencyList[coord]:
                            self.adjacencyList[coord].append((neigh,1))
                    if neigh not in self.adjacencyList:
                        self.adjacencyList[neigh] = [(coord,1)]
                    else:
                        if coord not in self.adjacencyList[neigh]:
                            self.adjacencyList[neigh].append((coord,1))

        for neigh in coordsDiag:
            if neigh in self.graph:
                if self.graph[neigh] == 0.5 or self.graph[neigh] == 1:
                    if (coord, neigh) not in self.graphLinks and  (neigh, coord) not in self.graphLinks:
                        self.graphLinks.append((coord, neigh))
                    if coord not in self.adjacencyList:
                        self.adjacencyList[coord] = [(neigh, np.sqrt(2))]
                    else:
                        if neigh not in self.adjacencyList[coord]:
                            self.adjacencyList[coord].append((neigh,np.sqrt(2)))
                    if neigh not in self.adjacencyList:
                        self.adjacencyList[neigh] = [(coord,np.sqrt(2))]
                    else:
                        if coord not in self.adjacencyList[neigh]:
                            self.adjacencyList[neigh].append((coord,np.sqrt(2)))           


    def removeNodeFromGraph(self, coord):
        self.adjacencyList[coord] = []
        coordsStraight,coordsDiag = self.getNeighbours(coord)
        neighbours = coordsStraight + coordsDiag
        for neigh in neighbours:
            if neigh in self.adjacencyList:
                if coord in self.adjacencyList[neigh]:
                    self.adjacencyList[neigh].remove(coord)

        