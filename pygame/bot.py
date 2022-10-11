from copy import deepcopy
import random
import numpy as np

import pygame

from numpy.core.fromnumeric import argmin
from scipy.spatial import ConvexHull

from utilities import *
import obstacle as obs

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

class Bot():
    def __init__(self, x, y, radius, room, objective, randomObjective = False, randomInterval = 10, color = (0,255,0), haveObjective = True, radiusDetection = 100, showDetails = False):
        self.room = room
        self.x = x
        self.y = y
        self.radius = radius
        self.vel2D = np.asarray([0.0001,0.0001])
        self.lastVel2D = np.asarray([0.0001,0.0001])
        self.objective = objective
        self.radiusDetection = radiusDetection
        self.rotationSpeed = 24
        self.speed = 8
        self.groupObj = []
        self.detectedObj = []
        self.groupObjPoints = []
        self.walls = []
        self.groupWall = []
        self.detectedWall = []
        self.groupWallPoints = []
        self.safeCoeff = 1
        self.ontoObjectiveCoeff = 1
        self.randomObjective = randomObjective
        self.turnAroundCoeff = 1
        self.barycenterGroupObj = {'x' : 0, 'y' : 0}
        self.barycenterGroupWall = {'x' : 0, 'y' : 0}
        self.groupObjRadius = 0
        self.randomInterval = randomInterval
        self.margin=2
        self.polygonPointsAbsolute = createPolygonMask([0, 0], 10, self.radius + self.margin)
        self.polygonPoints = deepcopy(self.polygonPointsAbsolute)
        self.convexHullObstacles = None
        self.groupPolygonPoints = []
        self.mode = "polygon"
        self.color = color
        self.ontoObjective = False
        self.haveObjective = haveObjective
        self.showDetails = showDetails
        self.maxDistConsider = 100
        if not self.haveObjective:
            self.vel2D = np.asarray([0,0])


    def draw(self):

        if not self.showDetails:
            if self.mode == "circle" :
                pygame.draw.circle(self.room.surface1, (255,0,255, 64), (self.barycenterGroupObj['x'], self.barycenterGroupObj['y']), self.groupObjRadius)
                pygame.draw.circle(self.room.surface1, (20,20,20, 64), (self.barycenterGroupObj['x'], self.barycenterGroupObj['y']), 4)
                
            if np.linalg.norm(self.vel2D) !=0:
                vel2DU = self.vel2D/np.linalg.norm(self.vel2D)
                pygame.draw.line(self.room.surface1, (255,255,255), (self.x, self.y), (self.x + vel2DU[0]*self.radius, self.y + vel2DU[1]*self.radius))

            pygame.draw.circle(self.room.surface1, self.color, (self.x, self.y), self.radius) # à la fin pour qu'il apparaisse au dessus du reste

        else:
            # Uncomment/comment for more/less details about the process !
            # pygame.draw.circle(self.room.surface1, (0,150,255, 128), (self.x, self.y), self.radiusDetection)
            if self.mode == "circle" :
                pygame.draw.circle(self.room.surface1, (255,0,255, 64), (self.barycenterGroupObj['x'], self.barycenterGroupObj['y']), self.groupObjRadius)
                pygame.draw.circle(self.room.surface1, (20,20,20, 64), (self.barycenterGroupObj['x'], self.barycenterGroupObj['y']), 4)
                
            if self.haveObjective:
                pygame.draw.circle(self.room.surface1, (255,255,255), (self.objective[0], self.objective[1]), self.radius)
            if self.mode == "polygon":
                if self.groupPolygonPoints == []:
                    self.convexHullObstacles = None
                if self.convexHullObstacles is not None:
                    pygame.draw.polygon(self.room.surface1, (200,50,50, 64), self.groupPolygonPoints)
                    pygame.draw.circle(self.room.surface1, (200,200,200, 64), (self.barycenterGroupObj['x'], self.barycenterGroupObj['y']), 4)
            if np.linalg.norm(self.vel2D) !=0:
                vel2DU = self.vel2D/np.linalg.norm(self.vel2D)
                pygame.draw.line(self.room.surface1, (255,255,255), (self.x, self.y), (self.x + vel2DU[0]*self.radius, self.y + vel2DU[1]*self.radius))
        
            pygame.draw.circle(self.room.surface1, self.color, (self.x, self.y), self.radius) # à la fin pour qu'il apparaisse au dessus du reste


    def cleanVision(self,obstaclesInView):

        forbiddenAreas = []

        # on ajoute les obstacles 1 à 1 à la liste, seulement s'ils sont bien visibles
        # on détermine, en regardant tous les obstacles, un ensemble de zones rectangulaires dans lesquelles ne peuvent pas se trouver des obstacles visibles
        for wall in obstaclesInView.keys():
           
            ### 1: si on est face à un mur, on ne peut pas voir ce qu'il y à derrière
            ### 2: si on voit un coin de mur, ça cache une partie de la pièce en plus

            # à droite d'un mur vertical
            if wall.orientation == 'v':
                if self.x > max(wall.Xs):
                    if min(wall.Ys) <= self.y <= max(wall.Ys):
                        # 1: rectangle
                        forbiddenAreas.append([(0,min(wall.Ys)), (max(wall.Xs)-1,min(wall.Ys)),  (max(wall.Xs)-1,max(wall.Ys)), (0,max(wall.Ys))])
                        # 2: triangles ou trapèzes, on trace la droite qui relie le robot avec le coin et on voit où elle intersecte le bord de la fenêtre
                        
                        cx, cy = max(wall.Xs),min(wall.Ys)
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.radiusDetection: # si le coin est vu par le robot
                            if cy < self.y: # coin au dessus du robot
                                res_top = linesIntersect([(self.x,self.y),(cx,cy)],[(0,0),(self.room.width,0)])
                                if res_top != None:
                                    resX, resY = res_top[0], res_top[1]
                                    if resX > 0: # trapèze
                                        forbiddenAreas.append([(0,0),(resX,0),(cx,cy),(0,cy)])
                                    else: # triangle
                                        res_left = linesIntersect([(self.x,self.y),(cx,cy)],[(0,0),(0,self.room.height)])
                                        if res_left != None:
                                            resX, resY = res_left[0], res_left[1]
                                            forbiddenAreas.append([(0,resY),(cx,cy),(0,cy)])

                        cx, cy = max(wall.Xs),max(wall.Ys)
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.radiusDetection: # si le coin est vu par le robot
                            if cy > self.y: # coin en dessous du robot
                                res_bot = linesIntersect([(self.x,self.y),(cx,cy)],[(0,self.room.height),(self.room.width,self.room.height)])
                                if res_bot != None:
                                    resX, resY = res_bot[0], res_bot[1]
                                    if resX > 0: # trapèze
                                        forbiddenAreas.append([(0,self.room.height),(resX,self.room.height),(cx,cy),(0,cy)])
                                    else: # triangle
                                        res_left = linesIntersect([(self.x,self.y),(cx,cy)],[(0,0),(0,self.room.height)])
                                        if res_left != None:
                                            resX, resY = res_left[0], res_left[1]
                                            forbiddenAreas.append([(0,resY),(cx,cy),(0,cy)])

                    # cas supplémentaires avec les coins, quand le robot n'est pas en face du mur                          
                    elif self.y < min(wall.Ys):  # robot au dessus du mur
                        cx, cy = min(wall.Xs),min(wall.Ys)
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.radiusDetection + wall.width: # si le coin est vu par le robot
                            res_left = linesIntersect([(self.x,self.y),(cx,cy)],[(0,0),(0,self.room.height)])
                            if res_left != None:
                                resX, resY = res_left[0], res_left[1]
                                if cy <= resY < self.room.height: # trapèze
                                    forbiddenAreas.append([(0,resY),(0,self.room.height),(cx,self.room.height),(cx,cy)])
                                else: # triangle
                                    res_bot = linesIntersect([(self.x,self.y),(cx,cy)],[(0,self.room.height),(self.room.width,self.room.height)])
                                    if res_bot != None:
                                        resX, resY = res_bot[0], res_bot[1]
                                        forbiddenAreas.append([(resX,self.room.height),(cx,cy),(cx,self.room.height)])

                    elif self.y > max(wall.Ys): # robot en dessous du mur
                        cx, cy = min(wall.Xs),max(wall.Ys)
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.radiusDetection + wall.width: # si le coin est vu par le robot
                            res_left = linesIntersect([(self.x,self.y),(cx,cy)],[(0,0),(0,self.room.height)])
                            if res_left != None:
                                resX, resY = res_left[0], res_left[1]
                                if cy >= resY > 0: # trapèze
                                    forbiddenAreas.append([(0,resY),(0,0),(cx,0),(cx,cy)])
                                else: # triangle
                                    res_top = linesIntersect([(self.x,self.y),(cx,cy)],[(0,0),(self.room.width,0)])
                                    if res_top != None:
                                        resX, resY = res_top[0], res_top[1]
                                        forbiddenAreas.append([(resX,0),(cx,cy),(cx,0)])


                # à gauche d'un mur vertical
                else:
                    if min(wall.Ys) <= self.y <= max(wall.Ys):
                        # 1: rectangle
                        forbiddenAreas.append([(min(wall.Xs)+1,min(wall.Ys)), (self.room.width,min(wall.Ys)), (self.room.width,max(wall.Ys)), (min(wall.Xs)+1,max(wall.Ys))])
                        # 2: triangle ou trapèze
                        cx, cy = min(wall.Xs),min(wall.Ys)
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.radiusDetection: # si le coin est vu par le robot
                            if cy < self.y: # coin au dessus du robot
                                res_top = linesIntersect([(self.x,self.y),(cx,cy)],[(0,0),(self.room.width,0)])
                                if res_top != None: 
                                    resX, resY = res_top[0], res_top[1]
                                    if resX < self.room.width: # trapèze
                                        forbiddenAreas.append([(self.room.width,0),(resX,0),(cx,cy),(self.room.width,cy)])
                                    else: # triangle
                                        res_right = linesIntersect([(self.x,self.y),(cx,cy)],[(self.room.width,0),(self.room.width,self.room.height)])
                                        if res_right != None:
                                            resX, resY = res_right[0], res_right[1]
                                            forbiddenAreas.append([(self.room.width,resY),(cx,cy),(self.room.width,cy)])

                        cx, cy = min(wall.Xs),max(wall.Ys)
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.radiusDetection: # si le coin est vu par le robot
                            if cy > self.y: # coin en dessous du robot
                                res_bot = linesIntersect([(self.x,self.y),(cx,cy)],[(0,self.room.height),(self.room.width,self.room.height)])
                                if res_bot != None:
                                    resX, resY = res_bot[0], res_bot[1]
                                    if resX < self.room.width: # trapèze
                                        forbiddenAreas.append([(self.room.width,self.room.height),(resX,self.room.height),(cx,cy),(self.room.width,cy)])
                                    else: # triangle
                                        res_right = linesIntersect([(self.x,self.y),(cx,cy)],[(self.room.width,0),(self.room.width,self.room.height)])
                                        if res_right != None:
                                            resX, resY = res_right[0], res_right[1]
                                            forbiddenAreas.append([(self.room.width,resY),(cx,cy),(self.room.width,cy)])
                    
                    # cas supplémentaires avec les coins, quand le robot n'est pas en face du mur                          
                    elif self.y < min(wall.Ys): # robot au dessus du mur
                        cx, cy = max(wall.Xs),min(wall.Ys)
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.radiusDetection + wall.width: # si le coin est vu par le robot
                            res_right = linesIntersect([(self.x,self.y),(cx,cy)],[(self.room.width,0),(self.room.width,self.room.height)])
                            if res_right != None:
                                resX, resY = res_right[0], res_right[1]
                                if cy <= resY < self.room.height: # trapèze
                                    forbiddenAreas.append([(self.room.width,resY),(self.room.width,self.room.height),(cx,self.room.height),(cx,cy)])
                                else: # triangle
                                    res_bot = linesIntersect([(self.x,self.y),(cx,cy)],[(0,self.room.height),(self.room.width,self.room.height)])
                                    if res_bot != None:
                                        resX, resY = res_bot[0], res_bot[1]
                                        forbiddenAreas.append([(resX,self.room.height),(cx,cy),(cx,self.room.height)])
                        
                    elif self.y > max(wall.Ys):
                        cx, cy = max(wall.Xs),max(wall.Ys)
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.radiusDetection + wall.width: # si le coin est vu par le robot
                            res_right = linesIntersect([(self.x,self.y),(cx,cy)],[(self.room.width,0),(self.room.width,self.room.height)])
                            if res_right != None:
                                resX, resY = res_right[0], res_right[1]
                                if  self.room.height >= resY > cy: # trapèze
                                    forbiddenAreas.append([(self.room.width,resY),(self.room.width,0),(cx,0),(cx,cy)])
                                else: # triangle
                                    res_top = linesIntersect([(self.x,self.y),(cx,cy)],[(0,0),(self.room.width,0)])
                                    if res_top != None:
                                        resX, resY = res_top[0], res_top[1]
                                        forbiddenAreas.append([(resX,0),(cx,cy),(cx,0)])


            # en dessous d'un mur horizontal
            elif wall.orientation == 'h':
                if self.y > max(wall.Ys):
                    if min(wall.Xs) <= self.x <= max(wall.Xs) and wall.orientation == 'h':
                        # 1: rectangle
                        forbiddenAreas.append([(min(wall.Xs),0), (max(wall.Xs),0),  (max(wall.Xs),max(wall.Ys)-1), (min(wall.Xs),max(wall.Ys)-1)])
                        # 2: triangle ou trapèze
                        cx, cy = min(wall.Xs),max(wall.Ys)
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.radiusDetection: # si le coin est vu par le robot
                            if cx < self.x: # coin à gauche du robot
                                res_left = linesIntersect([(self.x,self.y),(cx,cy)],[(0,0),(0,self.room.height)])
                                if res_left != None:
                                    resX, resY = res_left[0], res_left[1]
                                    if resY > 0: # trapèze
                                        forbiddenAreas.append([(0,resY),(0,0),(cx,0),(cx,cy)])
                                    else: # triangle
                                        res_top = linesIntersect([(self.x,self.y),(cx,cy)],[(0,0),(self.room.width,0)])
                                        if res_top != None:
                                            resX, resY = res_top[0], res_top[1]
                                            forbiddenAreas.append([(resX,0),(cx,cy),(cx,0)])

                        cx, cy = max(wall.Xs),max(wall.Ys)
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.radiusDetection: # si le coin est vu par le robot
                            if cx > self.x: # coin à droite du robot
                                res_right = linesIntersect([(self.x,self.y),(cx,cy)],[(self.room.width,0),(self.room.width,self.room.height)])
                                if res_right != None:
                                    resX, resY = res_right[0], res_right[1]
                                    if resY > 0: # trapèze
                                        forbiddenAreas.append([(self.room.width,resY),(self.room.width,0),(cx,0),(cx,cy)])
                                    else: # triangle
                                        res_top = linesIntersect([(self.x,self.y),(cx,cy)],[(0,0),(self.room.width,0)])
                                        if res_top != None:
                                            resX, resY = res_top[0], res_top[1]
                                            forbiddenAreas.append([(resX,0),(cx,cy),(cx,0)])
                    
                    # cas supplémentaires avec les coins, quand le robot n'est pas en face du mur                          
                    elif self.x < min(wall.Xs): # robot à gauche du mur
                        cx, cy = min(wall.Xs),min(wall.Ys)
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.radiusDetection + wall.height: # si le coin est vu par le robot
                            res_top = linesIntersect([(self.x,self.y),(cx,cy)],[(0,0),(self.room.width,0)])
                            if res_top != None:
                                resX, resY = res_top[0], res_top[1]
                                if  cx <= resX < self.room.width: # trapèze
                                    forbiddenAreas.append([(resX,0),(self.room.width,0),(self.room.width,cy),(cx,cy)])
                                else: # triangle
                                    res_right = linesIntersect([(self.x,self.y),(cx,cy)],[(self.room.width,0),(self.room.width,self.room.height)])
                                    if res_right != None:
                                        resX, resY = res_right[0], res_right[1]
                                        forbiddenAreas.append([(self.room.width,resY),(cx,cy),(self.room.width,cy)])
                        
                    elif self.x > max(wall.Xs): # robot à droite du mur
                        cx, cy = max(wall.Xs),min(wall.Ys)
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.radiusDetection + wall.height: # si le coin est vu par le robot
                            res_top = linesIntersect([(self.x,self.y),(cx,cy)],[(0,0),(self.room.width,0)])
                            if res_top != None:
                                resX, resY = res_top[0], res_top[1]
                                if cx >= resX > 0: # trapèze
                                    forbiddenAreas.append([(resX,0),(0,0),(0,cy),(cx,cy)])
                                else: # triangle
                                    res_left = linesIntersect([(self.x,self.y),(cx,cy)],[(0,0),(0,self.room.height)])
                                    if res_left != None:
                                        resX, resY = res_left[0], res_left[1]
                                        forbiddenAreas.append([(0,resY),(cx,cy),(0,cy)])
                        

                # au dessus d'un mur horizontal
                else:
                    if min(wall.Xs) <= self.x <= max(wall.Xs):
                        # 1: rectangle
                        forbiddenAreas.append([(min(wall.Xs),min(wall.Ys)+1), (max(wall.Xs),min(wall.Ys)+1), (max(wall.Xs),self.room.height), (min(wall.Xs),self.room.height)])
                        # 2: triangle ou trapèze
                        cx, cy = min(wall.Xs),min(wall.Ys)
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.radiusDetection: # si le coin est vu par le robot
                            if cx < self.x: # coin à gauche du robot                          
                                res_left = linesIntersect([(self.x,self.y),(cx,cy)],[(0,0),(0,self.room.height)])
                                if res_left != None:
                                    resX, resY = res_left[0], res_left[1]
                                    if 0 <= resY < self.room.height: # trapèze
                                        forbiddenAreas.append([(0,resY),(0,self.room.height),(cx,self.room.height),(cx,cy)])
                                    else: # triangle
                                        res_bot = linesIntersect([(self.x,self.y),(cx,cy)],[(0,self.room.height),(self.room.width,self.room.height)])
                                        if res_bot != None:
                                            resX, resY = res_bot[0], res_bot[1]
                                            forbiddenAreas.append([(resX,self.room.height),(cx,cy),(cx,self.room.height)])

                        cx, cy = max(wall.Xs),min(wall.Ys)
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.radiusDetection: # si le coin est vu par le robot           
                            if cx > self.x: # coin à droite du robot
                                res_right = linesIntersect([(self.x,self.y),(cx,cy)],[(self.room.width,0),(self.room.width,self.room.height)])
                                if res_right != None:
                                    resX, resY = res_right[0], res_right[1]
                                    if resY < self.room.height: # trapèze
                                        forbiddenAreas.append([(self.room.width,resY),(self.room.width,self.room.height),(cx,self.room.height),(cx,cy)])
                                    else: # triangle
                                        res_bot = linesIntersect([(self.x,self.y),(cx,cy)],[(0,self.room.height),(self.room.width,self.room.height)])
                                        if res_bot != None:
                                            resX, resY = res_bot[0], res_bot[1]
                                            forbiddenAreas.append([(resX,self.room.height),(cx,cy),(cx,self.room.height)])
                        
                    # cas supplémentaires avec les coins, quand le robot n'est pas en face du mur                          
                    elif self.x < min(wall.Xs): # robot à gauche du mur
                        cx, cy = min(wall.Xs),max(wall.Ys)
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.radiusDetection + wall.height: # si le coin est vu par le robot
                            res_bot = linesIntersect([(self.x,self.y),(cx,cy)],[(0,self.room.height),(self.room.width,self.room.height)])
                            if res_bot != None:
                                resX, resY = res_bot[0], res_bot[1]
                                if cx <= resX < self.room.width: # trapèze
                                    forbiddenAreas.append([(resX,self.room.height),(self.room.width,self.room.height),(self.room.width,cy),(cx,cy)])
                                else: # triangle
                                    res_right = linesIntersect([(self.x,self.y),(cx,cy)],[(self.room.width,0),(self.room.width,self.room.height)])
                                    if res_right != None:
                                        resX, resY = res_right[0], res_right[1]
                                        forbiddenAreas.append([(self.room.width,resY),(cx,cy),(self.room.width,cy)])
                        
                    elif self.x > max(wall.Xs): # robot à droite du mur
                        cx, cy = max(wall.Xs),max(wall.Ys)
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.radiusDetection + wall.height: # si le coin est vu par le robot
                            res_bot = linesIntersect([(self.x,self.y),(cx,cy)],[(0,self.room.height),(self.room.width,self.room.height)])
                            if res_bot != None:
                                resX, resY = res_bot[0], res_bot[1]
                                if cx >= resX > 0: # trapèze
                                    forbiddenAreas.append([(resX,self.room.height),(0,self.room.height),(0,cy),(cx,cy)])
                                else: # triangle
                                    res_left = linesIntersect([(self.x,self.y),(cx,cy)],[(0,0),(0,self.room.height)])
                                    if res_left != None:
                                        resX, resY = res_left[0], res_left[1]
                                        forbiddenAreas.append([(0,resY),(cx,cy),(0,cy)])

        result = {}

        for wall in obstaclesInView.keys():
            result[wall] = []
            for obstacle in obstaclesInView[wall]:
                isVisible = True
                compteur = 0
                while isVisible and compteur < len(forbiddenAreas):
                    area = forbiddenAreas[compteur]
                    point = Point(obstacle.x, obstacle.y) # utilisation de la bibliothèque shapely
                    polygon = Polygon(area)
                    if polygon.contains(point):
                        isVisible = False
                    compteur += 1

                if isVisible:
                    result[wall].append(obstacle)

        return result, forbiddenAreas


    def visibleObstacles(self,wall):
        visibleObs = []
        visibleSides = []

        if self.x > max(wall.Xs):
            if not 'right' in visibleSides:
                visibleSides.append('right')

        if self.x < min(wall.Xs):
            if not 'left' in visibleSides:
                visibleSides.append('left')

        # pour Y ce qui est écrit ici ne semble pas logique, mais en fait il faut se souvenir qu'on indexe depuis le coin en haut à gauche
        if self.y > max(wall.Ys):
            if not 'bot' in visibleSides:
                visibleSides.append('bot')

        if self.y < min(wall.Ys):
            if not 'top' in visibleSides:
                visibleSides.append('top')

        for obstacle in wall.obstacles:
            if obstacle.positionInWall in visibleSides and distObj(obstacle,self) <= self.radiusDetection + obstacle.radius:
                visibleObs.append(obstacle)
        
        return visibleObs
    
    
    def vision(self,debug):
        # on identifie les murs qui sont à portée du robot
        wallsInView = []
        for wall in self.room.walls:
            if wall.visibleForBot(self):
                wallsInView.append(wall)

        # on identifie les obstacles (portions de mur) qui sont visibles pour le robot
        obstaclesInView = {}
        for wall in wallsInView:
            obstaclesInView[wall] = self.visibleObstacles(wall)

        # on élimine les incompatibilités, et on récupère au passage les zones non visibles
        obstaclesInView, forbiddenAreas = self.cleanVision(obstaclesInView)
        
        # on compose toutes ces zones non visibles avec le cercle de vision du robot pour obtenir la vraie zone visible
        if debug :
            visibleSurface = pygame.Surface((self.room.width,self.room.height),  pygame.SRCALPHA)
            visibleSurface.fill((0,0,0,200))
            pygame.draw.circle(visibleSurface,(0,255,0,10),(self.x,self.y),self.radiusDetection) # zone transparente de départ
            for area in forbiddenAreas:    
                pygame.draw.polygon(visibleSurface,(40,0,0,200),area)
        else :
            visibleSurface = pygame.Surface((self.room.width,self.room.height),  pygame.SRCALPHA)
            visibleSurface.fill((0,0,0,200))
            pygame.draw.circle(visibleSurface,(0,0,0,0),(self.x,self.y),self.radiusDetection) # zone transparente de départ
            for area in forbiddenAreas:    
                pygame.draw.polygon(visibleSurface,(0,0,0,200),area)

        return wallsInView, obstaclesInView, visibleSurface



    def move(self):
        for i in range(len(self.polygonPoints)):
            self.polygonPoints[i][0] = self.polygonPointsAbsolute[i][0] + self.x
            self.polygonPoints[i][1] = self.polygonPointsAbsolute[i][1] + self.y
        if self.randomObjective:
            if random.random() > (1 - 1/(100*self.randomInterval)) :
                self.objective[0] = random.randrange(50, self.room.self.room.surface1.get_widht() - 50)  
                self.objective[1] = random.randrange(50, self.room.self.room.surface1.get_height - 50) 
                self.haveObjective = True
                self.ontoObjective = False
                self.vel2D = np.asarray([0.01,0.01])
        if self.haveObjective:
            self.goToObjective()
        if np.linalg.norm(self.vel2D) !=0:
            vel2DU = self.vel2D/np.linalg.norm(self.vel2D)
            self.x += vel2DU[0]*self.speed*self.safeCoeff*self.ontoObjectiveCoeff 
            self.y += vel2DU[1]*self.speed*self.safeCoeff*self.ontoObjectiveCoeff 


    def checkCollision(self):
        collision = {}
        for obj in self.room.objects:
            if obj != self :
                distO = distObj(self, obj)
                if distO <= self.radiusDetection:

                    if distO < self.radius + obj.radius :
                        print("COLLISION")

                    if (obj not in self.detectedObj):
                        self.detectedObj.append(obj)
                    if distO <= 50 + obj.radius*1.5 + self.radius:
                        sols = circleLineInter(self, obj, self.vel2D)
                        if len(sols)>0:
                            if len(sols)>1:
                                minDist = distObjDict(self, sols[0])
                                minIndex = 0
                                for i in range(1, len(sols[1:])):
                                    dist = distObjDict(self, sols[i])
                                    if dist < minDist:
                                        minDist = dist
                                        minIndex = i
                                
                                collision[obj] = sols[minIndex]
                            else:
                                collision[obj] = sols[0]
        
        for obj in self.detectedObj:
            if distObj(obj, self) > self.maxDistConsider:
                self.detectedObj.remove(obj)


        return collision


    def checkColObjective(self):
        col = False
        for obj in self.groupObj:
            angleColObj = signedAngle2Vects2(np.array([obj.x- self.objective[0], obj.y - self.objective[1]]), -1*np.array([self.objective[0]- self.x, self.objective[1] - self.y]))

            
            if len(circleLineInter(self, obj, np.array([self.objective[0]- self.x, self.objective[1] - self.y]))) >0 and abs(angleColObj) < np.pi/2:
                col = True
                return col
        return col


    def noColgoToObjective(self):
        self.safeCoeff = 1
        angleCol = (signedAngle2Vects2(self.vel2D, np.array([self.barycenterGroupObj['x'] - self.x, self.barycenterGroupObj['y'] - self.y])))

        if not self.checkColObjective():
    
            angleObj = signedAngle2Vects2(self.vel2D, np.array([self.objective[0]- self.x, self.objective[1] - self.y]))
            rotationSpeed = min(self.rotationSpeed*np.pi/180, abs(angleObj))
            if angleObj >= 0:
                self.vel2D = rotate(self.vel2D, rotationSpeed)
            elif angleObj < 0:
                self.vel2D = rotate(self.vel2D, - rotationSpeed)

        elif self.mode == "circle":
            if distObjDict(self, self.barycenterGroupObj) < self.groupObjRadius:
                if random.random() > (1 - 1/(10000/self.speed))  :
                    self.turnAroundCoeff *= -1
                self.vel2D = self.vel2D
            
            elif len(circleLineInter(self, self.barycenterGroupObj, self.vel2D, objDict = True, objRadius = self.groupObjRadius)) >0 and abs(angleCol) < np.pi/2:
                if angleCol > 0:
                    self.vel2D = rotate(self.vel2D, - self.rotationSpeed*np.pi/180)
                elif angleCol <= 0:
                    self.vel2D = rotate(self.vel2D, self.rotationSpeed*np.pi/180)
            else :
                angleObj = signedAngle2Vects2(self.vel2D, np.array([self.objective[0]- self.x, self.objective[1] - self.y]))
                rotationSpeed = min(self.rotationSpeed*np.pi/180, abs(angleObj))
                if angleObj >= 0:
                    self.vel2D = rotate(self.vel2D, rotationSpeed)
                elif angleObj < 0:
                    self.vel2D = rotate(self.vel2D, - rotationSpeed)

        elif self.mode == "polygon":
            
            polygonInter = polygonLineInter(self, self.groupPolygonPoints,self.barycenterGroupObj, self.vel2D)
            if len (polygonInter) == 1:
                if angleCol > 0:
                    self.vel2D = rotate(self.vel2D, - self.rotationSpeed*np.pi/180)
                elif angleCol <= 0:
                    self.vel2D = rotate(self.vel2D, self.rotationSpeed*np.pi/180)
            elif len(polygonInter) == 2 and abs(angleCol) < np.pi/2:
                if angleCol > 0:
                    self.vel2D = rotate(self.vel2D, - self.rotationSpeed*np.pi/180)
                elif angleCol <= 0:
                    self.vel2D = rotate(self.vel2D, self.rotationSpeed*np.pi/180)
            elif len(polygonInter) >= 2 and abs(angleCol) < np.pi/2:
                if angleCol > 0:
                    self.vel2D = rotate(self.vel2D, - self.rotationSpeed*np.pi/180)
                elif angleCol <= 0:
                    self.vel2D = rotate(self.vel2D, self.rotationSpeed*np.pi/180)
            elif pointInPolygon(self, self.groupPolygonPoints):
                if random.random() > (1 - 1/(10000/self.speed))  :
                    self.turnAroundCoeff *= -1
                self.vel2D = self.vel2D
            else :
                angleObj = signedAngle2Vects2(self.vel2D, np.array([self.objective[0]- self.x, self.objective[1] - self.y]))
                rotationSpeed = min(self.rotationSpeed*np.pi/180, abs(angleObj))
                if angleObj >= 0:
                    self.vel2D = rotate(self.vel2D, rotationSpeed)
                elif angleObj < 0:
                    self.vel2D = rotate(self.vel2D, -rotationSpeed)


    def goToObjective(self):
        distObjective = distObjList(self, self.objective)
        if distObjective < 60*self.speed/self.rotationSpeed + self.radius*2:
            self.ontoObjectiveCoeff = distObjective/((60*self.speed/self.rotationSpeed)**(1))
            # self.ontoObjectiveCoeff = 1
            if distObjective < 3:
                self.ontoObjective = True
                self.haveObjective = False
                self.lastVel2D = self.vel2D
                self.vel2D = np.asarray([0,0])
                self.groupObj = []
                self.groupObjPoints = []
                self.groupWall = []
                self.groupWallPoints = []
                self.groupPolygonPoints = []
                self.convexHullObstacles = None
        else :
            self.ontoObjectiveCoeff = 1

        collision = self.checkCollision() 
        
        checkedCollision = []

        if collision :
            init = True
            minDist = None
            minObj = None
            for obj in collision:
                dist = distObjDict(self, collision[obj])
                angleCol = signedAngle2Vects2(self.vel2D, np.array([obj.x - self.x, obj.y - self.y]))
                if init : 
                    if abs(angleCol) <= np.pi/2:
                        checkedCollision.append(obj)
                        minDist = distObjDict(self, collision[obj])
                        minObj = obj
                        init = False
                else :
                    if abs(angleCol) <= np.pi/2:
                        checkedCollision.append(obj)
                        if dist < minDist :
                            minDist = dist
                            minObj = obj

            if minObj is not None:
                if minObj not in self.groupObj :
                    self.groupObj.append(minObj)
                    self.groupObjPoints+=minObj.polygonPoints[:]

                dist = distObj(self, minObj)
                if  dist - minObj.radius - self.radius < self.speed*40/(np.sqrt(self.rotationSpeed)):
                    self.safeCoeff = max((dist - minObj.radius - self.radius)/(self.speed*40/(np.sqrt(self.rotationSpeed))), 0.01)
            else:
                self.safeCoeff = 1

            

            for obj in checkedCollision:
                if obj not in self.detectedObj:
                    self.detectedObj.append(obj)
            
            if minObj is not None:
                checkedgroupObj = [minObj]
                checkedgroupObjPoints = minObj.polygonPoints[:]
                i=0
                while i<len(checkedgroupObj):
                    for obj in self.groupObj:
                        if obj not in checkedgroupObj:
                            if distObj(obj, checkedgroupObj[i]) < obj.radius + checkedgroupObj[i].radius + 2*self.radius + 2*self.margin:
                                if distObj(obj, self) < self.maxDistConsider :
                                    if minObj is not None and isinstance(minObj, obs.Obstacle) and isinstance(obj, obs.Obstacle) and (minObj.isWall or obj.isWall):
                                        if obj.isWall == minObj.isWall or distObj(obj, self) < 30:
                                            checkedgroupObj.append(obj)
                                            checkedgroupObjPoints+=obj.polygonPoints[:]
                                    else:
                                        checkedgroupObj.append(obj)
                                        checkedgroupObjPoints+=obj.polygonPoints[:]
                    i+=1
                self.groupObj = checkedgroupObj
                self.groupObjPoints = checkedgroupObjPoints

            elif len(self.groupObj) > 0 :
                distList = [distObj(obj, self) for obj in self.groupObj]
                minObj = self.groupObj[argmin(distList)]
                checkedgroupObj = [self.groupObj[argmin(distList)]]
                checkedgroupObjPoints = checkedgroupObj[0].polygonPoints[:]
                i=0
                while i<len(checkedgroupObj):
                    for obj in self.groupObj:
                        if obj not in checkedgroupObj:
                            if distObj(obj, checkedgroupObj[i]) < obj.radius + checkedgroupObj[i].radius + 2*self.radius + 2*self.margin:
                                if distObj(obj, self) < self.maxDistConsider :
                                    if minObj is not None and isinstance(minObj, obs.Obstacle) and isinstance(obj, obs.Obstacle) and (minObj.isWall or obj.isWall):
                                        if obj.isWall == minObj.isWall or distObj(obj, self) < 30:
                                            checkedgroupObj.append(obj)
                                            checkedgroupObjPoints+=obj.polygonPoints[:]
                                    else:
                                        checkedgroupObj.append(obj)
                                        checkedgroupObjPoints+=obj.polygonPoints[:]
                    i+=1
                self.groupObj = checkedgroupObj
                self.groupObjPoints = checkedgroupObjPoints
 
            else :
                self.groupObj = []
                self.groupObjPoints = []
                self.groupPolygonPoints = []
                self.convexHullObstacles = None
            

            i=0
            if minObj is not None and isinstance(minObj, obs.Obstacle) and minObj.isWall:
                while i<len(self.groupObj):
                    for obj in self.detectedObj : 
                        if obj not in self.groupObj :
                            if distObj(obj, self.groupObj[i]) < obj.radius + self.groupObj[i].radius + 2*self.radius + 2*self.margin:
                                if isinstance(obj, obs.Obstacle) and obj.isWall == minObj.isWall or distObj(obj, self) < 10:
                                    self.groupObj.append(obj)
                                    self.groupObjPoints+=obj.polygonPoints[:]
                    i+=1
               

            if (len(self.groupObj)) > 0:
                self.barycenterGroupObj = { 'x' : (1/len(self.groupObj))*np.sum(np.array([obj.x for obj in self.groupObj])), 'y' : (1/len(self.groupObj))*np.sum(np.array([obj.y for obj in self.groupObj]))}

                self.convexHullObstacles = ConvexHull(self.groupObjPoints)
                if self.convexHullObstacles is not None:
                    self.groupPolygonPoints = [self.groupObjPoints[i] for i in list(self.convexHullObstacles.vertices)[:]]
                
                
                angleCol = (signedAngle2Vects2(self.vel2D, np.array([self.barycenterGroupObj['x'] - self.x, self.barycenterGroupObj['y'] - self.y])))
                polygonInter = polygonLineInter(self, self.groupPolygonPoints,self.barycenterGroupObj, self.vel2D)
                if len (polygonInter) == 1:
                    if angleCol > 0:
                        self.vel2D = rotate(self.vel2D, - self.rotationSpeed*np.pi/180)
                    elif angleCol <= 0:
                        self.vel2D = rotate(self.vel2D, self.rotationSpeed*np.pi/180)
                elif not self.checkColObjective():
                    
                    angleObj = signedAngle2Vects2(self.vel2D, np.array([self.objective[0]- self.x, self.objective[1] - self.y]))
                    rotationSpeed = min(self.rotationSpeed*np.pi/180, abs(angleObj))
                    if angleObj >= 0:
                        self.vel2D = rotate(self.vel2D, rotationSpeed)
                    elif angleObj < 0:
                        self.vel2D = rotate(self.vel2D, - rotationSpeed)
                elif pointInPolygon(self, self.groupPolygonPoints):
                    if angleCol > 0:
                        self.vel2D = rotate(self.vel2D, self.turnAroundCoeff*self.rotationSpeed*np.pi/180)
                    elif angleCol <= 0:
                        self.vel2D = rotate(self.vel2D, self.turnAroundCoeff*self.rotationSpeed*np.pi/180)
                elif abs(angleCol) <= np.pi/2 :
                    if angleCol > 0:
                        self.vel2D = rotate(self.vel2D, - self.rotationSpeed*np.pi/180)
                    elif angleCol <= 0:
                        self.vel2D = rotate(self.vel2D, self.rotationSpeed*np.pi/180)
                else:
                    self.noColgoToObjective()
            else:
                self.noColgoToObjective()


        else :
            if len(self.groupObj) > 0 :
                checkedgroupObj = [self.groupObj[0]]
                checkedgroupObjPoints = self.groupObj[0].polygonPoints[:]
                i=0
                while i<len(checkedgroupObj):
                    for obj in self.groupObj:
                        if obj not in checkedgroupObj:
                            if distObj(obj, checkedgroupObj[i]) < obj.radius + checkedgroupObj[i].radius + 2*self.radius + 2*self.margin:
                                if distObj(obj, self) < self.maxDistConsider :
                                    checkedgroupObj.append(obj)
                                    checkedgroupObjPoints+=obj.polygonPoints[:]
                    i+=1
                self.groupObj = checkedgroupObj
                self.groupObjPoints = checkedgroupObjPoints
                self.convexHullObstacles = ConvexHull(self.groupObjPoints)
                if self.convexHullObstacles is not None:
                    self.groupPolygonPoints = [self.groupObjPoints[i] for i in list(self.convexHullObstacles.vertices)[:]]
            else :
                self.groupObj = []
                self.groupObjPoints = []
                self.groupPolygonPoints = []


            self.noColgoToObjective()


    def defineObjective(self, coord):
        self.haveObjective = True
        self.objective = [0,0]
        self.objective[0] = coord[0]
        self.objective[1] = coord[1]
        self.ontoObjective = False
        self.vel2D = self.lastVel2D    