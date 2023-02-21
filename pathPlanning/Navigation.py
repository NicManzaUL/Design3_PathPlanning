from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import json
import os
from pathPlanning.NavUtil import NavParam
from pathPlanning.Planner import PathPlanner
import pathPlanning.Mapper as Mapper
from pathPlanning import GoalSequencer
from flag_utils import flagUtils

pathPlanner = PathPlanner()
navParam = NavParam()


dir_path = os.path.dirname(os.path.realpath(__file__))
# Specify the path to the file relative to the script directory
map_path = os.path.join(dir_path, "pathPlanning", "mapData.json")


# Parameters
GRID_SIZE = navParam.getGRID_SIZE() # potential grid size [m]
DIMENSION = navParam.getDIMENSION()
KP = navParam.getATRC_POTL_GAIN() # attractive potential gain
ETA = navParam.getRPLV_POTL_GAIN() # repulsive potential gain
AREA_WIDTH = navParam.getAREA_WIDTH()  # potential area width [m]
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = navParam.getOSCILLATIONS_DETECTION_LENGTH()
ROBOT_RADIUS = 2.5  # robot radius [m]
LAST_POSITION = (int, int)



show_animation = True


def setCurrentCoord(x, y):
    LAST_POSITION=(x, y)

def scale(coord):
    return coord#*GRID_SIZE


def moveTo(start, goal):

    list, list = pathPlanner.potential_field_planning(
        start, goal, Mapper.getObstacles(), GRID_SIZE, ROBOT_RADIUS, KP)


def moveToCube(color, start):

    list, list = pathPlanner.potential_field_planning(
        start, (Mapper.getLocation("cube", color)), Mapper.getObstacles(), GRID_SIZE, ROBOT_RADIUS, KP)

def moveToGrid(name, start):
    
    list, list = pathPlanner.potential_field_planning(
        start, Mapper.getLocation("grid", name), Mapper.getObstacles(), GRID_SIZE, ROBOT_RADIUS, KP)


def runScenario1():
    print("potential_field_planning start")


    start = (scale(2), scale(5))
    #goal = (scale(30), scale(30))
    
    f = open('./pathPlanning/mapData.json')
    listObstacles = json.load(f)
     # [(x coord, y coord, size)]
   
    if show_animation:
        plt.grid(True)
        plt.axis("equal")

    # path generation
    moveToCube('red', start)

    start = Mapper.getLocation('cube', 'red')
    moveToGrid('6', start)

    start = Mapper.getLocation('grid', '6')
    moveToCube('green', start)

    start = Mapper.getLocation('cube', 'green')
    moveToGrid('7', start)

    start = Mapper.getLocation('grid', '7')
    moveToCube('blue', start)

    start = Mapper.getLocation('cube', 'blue')
    moveToGrid('8', start)

    start = Mapper.getLocation('grid', '8')
    moveToCube('yellow', start)

    start = Mapper.getLocation('cube', 'yellow')
    moveToGrid('3', start)

    start = Mapper.getLocation('grid', '3')
    moveToCube('black', start)

    start = Mapper.getLocation('cube', 'black')
    moveToGrid('4', start)

    start = Mapper.getLocation('grid', '4')
    moveToCube('white', start)

    start = Mapper.getLocation('cube', 'white')
    moveToGrid('5', start)

    
    if show_animation:
        plt.show()

def runScenario2():
    print("potential_field_planning start")

    start = (scale(2), scale(5))
    #goal = (scale(30), scale(30))

    f = open('./pathPlanning/mapData.json')
    listObstacles = json.load(f)
    # [(x coord, y coord, size)]

    if show_animation:
        plt.grid(True)
        plt.axis("equal")

    orders = GoalSequencer.produceSequence(
        flagUtils.countryToFlag(flagUtils.hexToCountry('1'))
    )

    for order in orders:
        moveToCube(order[0], start)
        start = Mapper.getLocation('cube', order[0])
        print(f"getting {order[0]} cube")
        moveToGrid(str(order[1]), start)
        start = Mapper.getLocation('grid', str(order[1]))
    
    if show_animation:
        plt.show()
