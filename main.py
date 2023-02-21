import sys
sys.path.append("../")
from pathPlanning.Navigation import runScenario

if __name__ == '__main__':
    print(__file__ + " start!!")
    runScenario()
    print(__file__ + " Done!!")