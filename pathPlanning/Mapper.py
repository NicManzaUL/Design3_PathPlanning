from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import json
from pathPlanning.NavUtil import NavParam


navParam = NavParam()
LAST_POSITION = (int, int)

def setCurrentCoord(x, y):
    LAST_POSITION=(x, y)

def getCurrentCoord():
    return (getCurrentCoord_X, getCurrentCoord_Y)

def getCurrentCoord_X():
    return LAST_POSITION[0]
    
def getCurrentCoord_Y():
    return LAST_POSITION[1]

def getObstacles():
    f = open('./pathPlanning/mapData.json')
    listObstacles = json.load(f)
    obstacles = [(0, 0),(navParam.getDIMENSION()[0], 0), (0, navParam.getDIMENSION()[1]), (navParam.getDIMENSION()[0], navParam.getDIMENSION()[1]),(navParam.getDIMENSION()[0]*0.5, 0), (0, navParam.getDIMENSION()[1]*0.5), (navParam.getDIMENSION()[0]*0.5, navParam.getDIMENSION()[1]*0.5)]
    
    for obst in listObstacles['obstacles']:
        obstacles.append((obst['x'], obst['y']))
    return obstacles

def getLocation(type, spec):
    f = open('./pathPlanning/mapData.json')
    listObject = json.load(f)
    x = listObject[type][spec]['x']
    y = listObject[type][spec]['y']
    print("({},{})!".format(x, y))
    return (float(x), float(y))



def calc_potential_field(goal_x, goal_y, obstacles_x, obstacles_y, resolution,ROBOT_RADIUS, start_x, start_y, tolerance):
    minx = min(min(obstacles_x), start_x, goal_x) - navParam.getAREA_WIDTH() / 2.0
    miny = min(min(obstacles_y), start_y, goal_y) - navParam.getAREA_WIDTH() / 2.0
    maxx = max(max(obstacles_x), start_x, goal_x) + navParam.getAREA_WIDTH() / 2.0
    maxy = max(max(obstacles_y), start_y, goal_y) + navParam.getAREA_WIDTH() / 2.0
    xw = int(round((maxx - minx) / resolution))
    yw = int(round((maxy - miny) / resolution))

    # calc each potential
    potential_map = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * resolution + minx

        for iy in range(yw):
            y = iy * resolution + miny
            ug = calc_attractive_potential(x, y, goal_x, goal_y)
            uo = calc_repulsive_potential(x, y, obstacles_x, obstacles_y,ROBOT_RADIUS, tolerance)
            uf = ug + uo
            potential_map[ix][iy] = uf

    return potential_map, minx, miny

def calc_attractive_potential(x, y, goal_x, goal_y):
    return 0.5 * navParam.getATRC_POTL_GAIN() * np.hypot(x - goal_x, y - goal_y)

def calc_repulsive_potential(x, y, obstacles_x, obstacles_y,ROBOT_RADIUS, tolerance):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(obstacles_x):
        d = np.hypot(x - obstacles_x[i], y - obstacles_y[i])
        if dmin >= d:
            dmin = d
            minid = i

    # calc repulsive potential
    dq = np.hypot(x - obstacles_x[minid], y - obstacles_y[minid])/tolerance

    if dq <=ROBOT_RADIUS:
        if dq <= 0.1:
            dq = 0.1

        return 0.5 * navParam.getRPLV_POTL_GAIN() * (1.0 / dq - 1.0 /ROBOT_RADIUS) ** 2
    else:
        return 0.0