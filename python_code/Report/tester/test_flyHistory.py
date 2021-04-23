import sys
sys.path.append('../')

from FlyHistory import FlyHistory

Report = FlyHistory([-6,6], [-6,6], [-6,6])
Report.addObject(1, 2, 1, 0.15, 'g', 1)
Report.addObject(2, 2, 1, 0.15, 'g', 1)
Report.addObject(3, 2, 1, 0.15, 'g', 1)


Report.draw()
