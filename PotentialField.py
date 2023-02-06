from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import json

# Parameters
GRID_SIZE =0.5 # potential grid size [m]
DIMENSION = (25, 50)
KP = 2.5 # attractive potential gain
ETA = 100 # repulsive potential gain
AREA_WIDTH = 0.0  # potential area width [m]
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 3
ROBOT_RADIUS = 2.5  # robot radius [m]
LAST_POSITION = (int, int)



show_animation = True


def calc_potential_field(goal_x, goal_y, obstacles_x, obstacles_y, resolution,ROBOT_RADIUS, start_x, start_y, tolerance):
    minx = min(min(obstacles_x), start_x, goal_x) - AREA_WIDTH / 2.0
    miny = min(min(obstacles_y), start_y, goal_y) - AREA_WIDTH / 2.0
    maxx = max(max(obstacles_x), start_x, goal_x) + AREA_WIDTH / 2.0
    maxy = max(max(obstacles_y), start_y, goal_y) + AREA_WIDTH / 2.0
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
    return 0.5 * KP * np.hypot(x - goal_x, y - goal_y)

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

        return 0.5 * ETA * (1.0 / dq - 1.0 /ROBOT_RADIUS) ** 2
    else:
        return 0.0

def get_motion_model():
    # dx, dy
    east=[1, 0]
    north=[0, 1]
    west=[-1, 0]
    south= [0, -1]
    southwest=[-1, -1]
    northwest=[-1, 1]
    southeast=[1, -1]
    northeast= [1, 1]
    motion = [east, north, west, south, southwest, northwest, southeast, northeast]

    return motion

def oscillations_detection(previous_ids, ix, iy):
    previous_ids.append((ix, iy))

    if (len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH):
        previous_ids.popleft()

    # check if contains any duplicates by copying into a set
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            return True
        else:
            previous_ids_set.add(index)
    return False

def potential_field_planning(start, goal, obstacles, resolution,ROBOT_RADIUS, tolerance):

    start_x = start[0]
    start_y = start[1]

    goal_x = goal[0]
    goal_y = goal[1]

    obstacles_x = []
    [obstacles_x.append(scale(ox[0])) for ox in obstacles]
    obstacles_y = []
    [obstacles_y.append(scale(oy[1])) for oy in obstacles]

    # calc potential field
    potential_map, minx, miny = calc_potential_field(goal_x, goal_y, obstacles_x, obstacles_y, resolution,ROBOT_RADIUS, start_x, start_y, tolerance)

    # search path
    d = np.hypot(start_x - goal_x, start_y - goal_y)

    ix = round((start_x - minx) / resolution)
    iy = round((start_y - miny) / resolution)
    gix = round((goal_x - minx) / resolution)
    giy = round((goal_y - miny) / resolution)

    if show_animation:
        draw_heatmap(potential_map)
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(ix, iy, "*k")
        plt.plot(gix, giy, "*m")

    rx, ry = [start_x], [start_y]
    motion = get_motion_model()
    previous_ids = deque()

    while d >= resolution:
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(potential_map) or iny >= len(potential_map[0]) or inx < 0 or iny < 0:
                p = float("inf")  # outside area
                print("outside potential!")
              
            else:
                p = potential_map[inx][iny]

            if p < minp:
                minp = p
                minix = inx
                miniy = iny
            print(minix, miniy)
            
        ix = minix
        iy = miniy

        xp = ix * resolution + minx
        yp = iy * resolution + miny
        d = np.hypot(goal_x - xp, goal_y - yp)
        rx.append(xp)
        ry.append(yp)

        if (oscillations_detection(previous_ids, ix, iy)):
            print("Oscillation detected at ({},{})!".format(ix, iy))
            break

        if show_animation:
            plt.plot(ix, iy, ".r")
            plt.pause(0.01)

    print("Done")
    setCurrentCoord(rx[-1], ry[-1])
    return rx, ry

def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)

def setCurrentCoord(x, y):
    LAST_POSITION=(x, y)


def getCurrentCoord():
    return (getCurrentCoord_X, getCurrentCoord_Y)

def getCurrentCoord_X():
    return LAST_POSITION[0]
    
def getCurrentCoord_Y():
    return LAST_POSITION[1]

def scale(coord):
    return coord#*GRID_SIZE

def getObstacles():
    f = open('mapData.json')
    listObstacles = json.load(f)
    obstacles = [(0, 0),(DIMENSION[0], 0), (0, DIMENSION[1]), (DIMENSION[0], DIMENSION[1]),(DIMENSION[0]*0.5, 0), (0, DIMENSION[1]*0.5), (DIMENSION[0]*0.5, DIMENSION[1]*0.5)]
    for obst in listObstacles['obstacles']:
        obstacles.append((obst['x'], obst['y']))
    return obstacles

def getLocation(type, spec):
    f = open('mapData.json')
    listObject = json.load(f)
    x = listObject[type][spec]['x']
    y = listObject[type][spec]['y']
    print("({},{})!".format(x, y))
    return (float(x), float(y))

def moveTo(start, goal):

    list, list = potential_field_planning(
        start, goal, getObstacles(), GRID_SIZE, ROBOT_RADIUS, KP)


def moveToCube(color, start):

    list, list = potential_field_planning(
        start, (getLocation("cube", color)), getObstacles(), GRID_SIZE, ROBOT_RADIUS, KP)

def moveToGrid(name, start):
    
    list, list = potential_field_planning(
        start, getLocation("grid", name), getObstacles(), GRID_SIZE, ROBOT_RADIUS, KP)


def main():
    print("potential_field_planning start")



    start = (scale(2), scale(5))
    #goal = (scale(30), scale(30))
    
    f = open('mapData.json')
    listObstacles = json.load(f)
     # [(x coord, y coord, size)]
   
    if show_animation:
        plt.grid(True)
        plt.axis("equal")

    # path generation
    moveToCube('red', start)

    start = getLocation('cube', 'red')
    moveToGrid('BL', start)

    start = getLocation('grid', 'BL')
    moveToCube('green', start)

    start = getLocation('cube', 'green')
    moveToGrid('BM', start)

    start = getLocation('grid', 'BM')
    moveToCube('blue', start)

    start = getLocation('cube', 'blue')
    moveToGrid('BR', start)

    start = getLocation('grid', 'BR')
    moveToCube('yellow', start)

    start = getLocation('cube', 'yellow')
    moveToGrid('ML', start)

    start = getLocation('grid', 'ML')
    moveToCube('black', start)

    start = getLocation('cube', 'black')
    moveToGrid('MM', start)

    start = getLocation('grid', 'MM')
    moveToCube('white', start)

    start = getLocation('cube', 'white')
    moveToGrid('ML', start)

    

    
    if show_animation:
        plt.show()

if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")