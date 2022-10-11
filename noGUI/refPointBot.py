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


    def instantMovement(self):
        pass