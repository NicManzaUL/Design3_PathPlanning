import sys
sys.path.append("../")
from pathPlanning.Navigation import runScenario1
from pathPlanning.Navigation import runScenario2
if __name__ == '__main__':
    print(__file__ + " start!!")
    runScenario2()
    print(__file__ + " Done!!")