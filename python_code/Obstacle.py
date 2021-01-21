class Obstacle(object):
    def __init__(self, position):
        self.position = position
        self.positionHistory = [position]
        self.ObstaclePotentialForces = []