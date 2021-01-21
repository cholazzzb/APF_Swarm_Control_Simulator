class class2(object):
    def __init__(self, position):
        self.position = position
        self.total_repel_force = []
    def calculate_repel_force(self, agents, obstacles):
        for agent in agents:
            total_repel_force = (0,0,0)
            for obstacle in range obstacles:
                total_repel_force = total_repel_force + obstacle.repel_force
            self.total_repel_force.append(total_repel_force)
            print('test')