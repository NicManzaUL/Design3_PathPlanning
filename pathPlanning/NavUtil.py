from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import json


class NavParam:
    def __init__(self):
        self.GRID_SIZE =0.5 # potential grid size [m]
        self.DIMENSION = (50, 50)
        self.ATRC_POTL_GAIN = 2.5 # attractive potential gain
        self.RPLV_POTL_GAIN = 100 # repulsive potential gain
        self.AREA_WIDTH = 0.0  # potential area width [m]
        # the number of previous positions used to check oscillations
        self.OSCILLATIONS_DETECTION_LENGTH = 3
    
    def getGRID_SIZE(self):
        return self.GRID_SIZE
    
    def getDIMENSION(self):
        return self.DIMENSION
    
    def getATRC_POTL_GAIN(self):
        return self.ATRC_POTL_GAIN
    
    def getRPLV_POTL_GAIN(self):
        return self.RPLV_POTL_GAIN
    
    def getAREA_WIDTH(self):
        return self.AREA_WIDTH
    
    def getOSCILLATIONS_DETECTION_LENGTH(self):
        return self.OSCILLATIONS_DETECTION_LENGTH
