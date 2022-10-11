from copy import deepcopy
import random
import numpy as np

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

        

    def move(self):
        for i in range(len(self.polygonPoints)):
            self.polygonPoints[i][0] = self.polygonPointsAbsolute[i][0] + self.x
            self.polygonPoints[i][1] = self.polygonPointsAbsolute[i][1] + self.y
        if self.randomObjective:
            if random.random() > (1 - 1/(100*self.randomInterval)) :

                self.objective[0] = random.randrange(50, self.room.self.room.width - 50)  
                self.objective[1] = random.randrange(50, self.room.self.room.height - 50) 

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