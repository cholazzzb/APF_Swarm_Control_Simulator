class Simulator(object):
    def __init__(self):
        self.entities = []
    
    def set_agent_entities(self, Agents):
        for Agent in Agents:
            self.entities.append(('b', 'o', Agent.positionHistory))

    def set_obstacle_entities(self, Obstacles):
        for Obstacle in Obstacles:
            self.entities.append(('g', 'o', Obstacle.positionHistory))

    def set_target_entities(self, Targets):
        for Target in Targets:
            self.entities.append(('r', 'o', Target.positionHistory))