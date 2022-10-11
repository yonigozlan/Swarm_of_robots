from copy import deepcopy
import pygame

from utilities import createPolygonMask


class Obstacle():
    def __init__(self, x, y, radius, room, color = (100,100,100), movable = False, vel = 2, margin = 2, isWall = False, spacing = 15, positionInWall = None):
        self.isWall = isWall
        self.spacing = spacing
        self.positionInWall = positionInWall
        self.room = room
        self.color = color
        self.x = x
        self.y = y
        self.radius = radius
        self.vel = vel
        self.movable = movable
        self.margin = margin
        self.polygonPointsAbsolute = createPolygonMask([0, 0], 10, self.radius + self.margin)
        self.polygonPoints = deepcopy(self.polygonPointsAbsolute)

        for i in range(len(self.polygonPoints)):
            self.polygonPoints[i][0] = self.polygonPointsAbsolute[i][0] + self.x
            self.polygonPoints[i][1] = self.polygonPointsAbsolute[i][1] + self.y

        self.seen = False


    def draw(self):
        surface = self.room.surface1
        pygame.draw.circle(surface, self.color, (self.x, self.y), self.radius)