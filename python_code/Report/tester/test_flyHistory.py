import sys
sys.path.append('../')

from FlyHistory import FlyHistory

Report = FlyHistory([-6,6], [-6,6], [-6,6])
Report.addObject([1,2,3], [2,4,6], [1,1,1], 0.25, 'g', 1)

Report.draw()
