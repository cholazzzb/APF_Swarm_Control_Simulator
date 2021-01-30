from Robotagent import Robotagent
from AgentReport import AgentReport
import matplotlib.pyplot as plt

specs = {"mass": 1.25}
initialState = [0,0]
initialInput = 0
PIDController = [2,0.3,0.1]

Robot1 = Robotagent(specs, initialState, initialInput, PIDController)
Report1 = AgentReport(Robot1)

for i in range(50):
    print('--------- Time : ', Robot1.t, ' ----------') 
    Robot1.control(20)
    Robot1.updateState()
    
    Report1.updateReport(Robot1.getState())

Report1.generateReport()

plt.pause(600)