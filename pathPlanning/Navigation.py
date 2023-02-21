from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import json
import os
from pathPlanning.NavUtil import NavParam
from pathPlanning.Planner import PathPlanner
import pathPlanning.Mapper as Mapper

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
    moveToGrid('BL', start)

    start = Mapper.getLocation('grid', 'BL')
    moveToCube('green', start)

    start = Mapper.getLocation('cube', 'green')
    moveToGrid('BM', start)

    start = Mapper.getLocation('grid', 'BM')
    moveToCube('blue', start)

    start = Mapper.getLocation('cube', 'blue')
    moveToGrid('BR', start)

    start = Mapper.getLocation('grid', 'BR')
    moveToCube('yellow', start)

    start = Mapper.getLocation('cube', 'yellow')
    moveToGrid('ML', start)

    start = Mapper.getLocation('grid', 'ML')
    moveToCube('black', start)

    start = Mapper.getLocation('cube', 'black')
    moveToGrid('MM', start)

    start = Mapper.getLocation('grid', 'MM')
    moveToCube('white', start)

    start = Mapper.getLocation('cube', 'white')
    moveToGrid('ML', start) 

    
    if show_animation:
        plt.show()
