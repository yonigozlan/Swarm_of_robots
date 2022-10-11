import pygame

from bot import Bot
from obstacle import Obstacle
from utilities import *

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

class RefPointBot(Bot):

    def __init__(self, x, y, radius, room, objective, randomObjective = False, randomInterval = 10, color = (0,0,255), haveObjective = True, radiusDetection = 20, showDetails = False, message = "ExplorerBot",):
        super(self.__class__, self).__init__(x, y, radius, room, objective, randomObjective = randomObjective, randomInterval = randomInterval, color = color, haveObjective = haveObjective, radiusDetection = radiusDetection, showDetails = showDetails)
        self.message = message
        self.wallDetectionRadius = 30
        self.UWBradius = 1000
        self.isMoving = False

    def show_self(self):
        print(self.message)

    def sendIndDirection(self, direction):
        pass

    def wallDetectionAction(self):
        self.vel2D = np.asarray([0,0])
        self.haveObjective = False

    def checkCollision(self):
        collision = {}
        
        for obj in self.room.objects:
            if obj != self :
                distO = distObj(self, obj)
                if distO <= self.UWBradius:

                    if distO < self.radius + obj.radius :
                        print("COLLISION")
                    if isinstance(obj, Obstacle) and obj.isWall:
                        if distO <= self.wallDetectionRadius:
                            sols = circleLineInter(self, obj, [self.objective[0]-self.x, self.objective[1]-self.y])
                            if len(sols)>0:
                                for sol in sols:
                                    if np.linalg.norm(self.vel2D)!=0:
                                        angleCheck = signedAngle2Vects2(self.vel2D, (sol['x']-self.x, sol['y']-self.y))
                                        if abs(angleCheck)*360/np.pi < 7.5:
                                            self.wallDetectionAction()
                                            return collision
                    else:
                        if obj not in self.detectedObj:
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


    def cleanVisionUWB(self,wallsInView):

        forbiddenAreas = []

        # on ajoute les obstacles 1 à 1 à la liste, seulement s'ils sont bien visibles
        # on détermine, en regardant tous les obstacles, un ensemble de zones rectangulaires dans lesquelles ne peuvent pas se trouver des obstacles visibles
        for wall in wallsInView:
           
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
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.UWBradius: # si le coin est vu par le robot
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
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.UWBradius: # si le coin est vu par le robot
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
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.UWBradius + wall.width: # si le coin est vu par le robot
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
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.UWBradius + wall.width: # si le coin est vu par le robot
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
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.UWBradius: # si le coin est vu par le robot
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
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.UWBradius: # si le coin est vu par le robot
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
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.UWBradius + wall.width: # si le coin est vu par le robot
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
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.UWBradius + wall.width: # si le coin est vu par le robot
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
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.UWBradius: # si le coin est vu par le robot
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
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.UWBradius: # si le coin est vu par le robot
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
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.UWBradius + wall.height: # si le coin est vu par le robot
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
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.UWBradius + wall.height: # si le coin est vu par le robot
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
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.UWBradius: # si le coin est vu par le robot
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
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.UWBradius: # si le coin est vu par le robot           
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
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.UWBradius + wall.height: # si le coin est vu par le robot
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
                        if np.sqrt((cx-self.x)**2+(cy-self.y)**2) < self.UWBradius + wall.height: # si le coin est vu par le robot
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

        return forbiddenAreas

        
    def UWBcover(self):
        # on identifie les murs qui sont à portée du robot
        wallsInView = []
        for wall in self.room.walls:
            if wall.visibleForBotUWB(self):
                wallsInView.append(wall)

        # on élimine les incompatibilités, et on récupère au passage les zones non visibles
        forbiddenAreas = self.cleanVisionUWB(wallsInView)
        
        # on compose toutes ces zones non visibles avec le cercle de vision du robot pour obtenir la vraie zone visible
        visibleSurface = pygame.Surface((self.room.width,self.room.height),  pygame.SRCALPHA)
        pygame.draw.circle(visibleSurface,(0,0,20,20),(self.x,self.y),self.UWBradius) # zone bleue (transparente)
        for area in forbiddenAreas:    
            pygame.draw.polygon(visibleSurface,(0,0,0,0),area)

        return visibleSurface


    def instantMovement(self):
        pass