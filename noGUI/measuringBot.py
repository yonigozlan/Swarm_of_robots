from bot import Bot

class MeasuringBot(Bot):

    def __init__(self, x, y, radius, room, objective, randomObjective = False, randomInterval = 10, color = (255,0,0), haveObjective = True, radiusDetection = 200, showDetails = False, message = "MeasuringBot",):
        super(self.__class__, self).__init__(x, y, radius, room, objective, randomObjective = randomObjective, randomInterval = randomInterval, color = color, haveObjective = haveObjective, radiusDetection = radiusDetection, showDetails = showDetails)
        self.message = message

    def show_self(self):
        print(self.message)
        