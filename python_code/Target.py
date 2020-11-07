class Target(object):
    def __init__(self, position):
            self.position = position
            self.velocity = (0,0,0)

    def setVelocity(self, newVelocity):
        self.velocity = newVelocity

    def getVelocity(self):
        return self.velocity