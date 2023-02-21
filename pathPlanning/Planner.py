from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import json
from pathPlanning.NavUtil import NavParam
import pathPlanning.Mapper as Mapper

navParam = NavParam()
show_animation = True

pointStack=[]

class PathPlanner:
    def __init__(self):
        pass

    
    def get_motion_model(self):
    # dx, dy
        scale = 2
        motion_dict = {
            (scale, 0)      :'0',
            (scale, scale)  :'45',
            (0, scale)      :'90',
            (-scale, scale) :'135',
            (-scale, 0)     :'180',
            (-scale, -scale):'225',
            (0, -scale)     :'270',
            (scale, -scale) :'315'
        }
        motion = motion_dict
    
        return motion

    def oscillations_detection(self, previous_ids, ix, iy):
        previous_ids.append((ix, iy))

        if (len(previous_ids) > navParam.getOSCILLATIONS_DETECTION_LENGTH()):
            previous_ids.popleft()

        # check if contains any duplicates by copying into a set
        previous_ids_set = set()
        for index in previous_ids:
            if index in previous_ids_set:
                return True
            else:
                previous_ids_set.add(index)
        return False

    def potential_field_planning(self, start, goal, obstacles, resolution,ROBOT_RADIUS, tolerance):

        start_x = start[0]
        start_y = start[1]

        goal_x = goal[0]
        goal_y = goal[1]

        obstacles_x = []
        [obstacles_x.append(1*(ox[0])) for ox in obstacles]
        obstacles_y = []
        [obstacles_y.append(1*(oy[1])) for oy in obstacles]

        # calc potential field
        potential_map, minx, miny = Mapper.calc_potential_field(goal_x, goal_y, obstacles_x, obstacles_y, resolution,ROBOT_RADIUS, start_x, start_y, tolerance)

        # search path
        d = np.hypot(start_x - goal_x, start_y - goal_y)

        ix = round((start_x - minx) / resolution)
        iy = round((start_y - miny) / resolution)
        gix = round((goal_x - minx) / resolution)
        giy = round((goal_y - miny) / resolution)

        if show_animation:
            self.draw_heatmap(potential_map)
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(ix, iy, "*k")
            plt.plot(gix, giy, "*m")

        rx, ry = [start_x], [start_y]
        motions = self.get_motion_model()
        previous_ids = deque()

        current_angle="0"
        last_angle=0
        module_angle=1
        direction = "none"
        delta=0
        print("RESET current&last angle: 0")
        while d >= resolution:
            minp = float("inf")
            minix, miniy = -1, -1
 
            for i, (motion, angle) in enumerate(motions.items()):
                inx = int(ix + motion[0])
                iny = int(iy + motion[1])
                if inx >= len(potential_map) or iny >= len(potential_map[0]) or inx < 0 or iny < 0:
                    p = float("inf")  # outside area
                    print("outside potential!")
              
                else:
                    p = potential_map[inx][iny]
                

                if p < minp:
                    minp = p
                    minix = inx
                    miniy = iny
                    current_angle=angle
                
            delta = int(current_angle)-int(last_angle)
            #print("current : {} last: {} delta: {}".format(int(current_angle), int(last_angle), delta))
            
            done=False
            if(delta==0):
                    direction = "foward"
                    module_angle+=1
                    #print(direction)

            if(delta>0):
                print("{}-> {} ".format("+ "*module_angle, module_angle))
                print("... ... ... ... ...") #straight foward
                direction = "left"
                if(int(current_angle)>(int(last_angle)+180)):
                    delta = int(current_angle)-(int(last_angle)+360)
                    direction = "right"
                module_angle=1
                done=True

            if((delta<0)&(done!=True)):
                print("{}-> {} ".format("+ "*module_angle, module_angle))
                print("... ... ... ... ...") #straight foward
                direction = "right"
                if((int(current_angle)+180)<int(last_angle)):
                    delta = (int(current_angle)+360)-int(last_angle)
                    direction = "left"
                module_angle=1
            
            #print(direction)

                #print(minix, miniy)
                #print("... ... ...")
            
            ix = minix
            iy = miniy

            xp = ix * resolution + minx
            yp = iy * resolution + miny
            d = np.hypot(goal_x - xp, goal_y - yp)
            rx.append(xp)
            ry.append(yp)
            
            if(direction!='foward'):
                print("turn {} degrees {} ".format(abs(delta), direction))
                pass
            last_angle=current_angle


            if (self.oscillations_detection(previous_ids, ix, iy)):
                print("Oscillation detected at ({},{})!".format(ix, iy))
                break

            if show_animation:
                plt.plot(ix, iy, ".r")
                plt.pause(0.01)

        print("{}-> {} ".format("+ "*module_angle, module_angle))
        print("Done")
        #setCurrentCoord(rx[-1], ry[-1])
        return rx, ry
    
    def draw_heatmap(self, data):
        data = np.array(data).T
        plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)


    def toDirections(self, x, y):
        pointStack.append((x, y))


    


