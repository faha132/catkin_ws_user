import numpy as np
import os
from matplotlib import pyplot as plt
import math
from scipy import ndimage

def create_mat(map_file):
    """
    for each spot find closest point in given angle
    todo set frame set radius
    """
    #hyperparams
    distance_radius = 5
    alpha = np.pi/3
    #rest
    #given yaw orientation in euler
    yaw = 3*np.pi/2
    #make sure lines have no gaps
    map_file = ndimage.binary_dilation(map_file,iterations=1).astype(map_file.dtype)
    height, width = map_file.shape[:2]
    ar = np.zeros((height,width,3))
    print ar.shape
    for direction in range(3):
        for y,row in enumerate(map_file):
            for x,elem in enumerate(row):
                det = False
                dist_acc = 1
                #0 somewhat left 1 straight line 2 somewhat right
                while not det:
                    #ceil for val  != origin
                    #subtraction y axis because numpy has 0,0 left upper corner
                    x2 = int(math.ceil(x+math.cos(yaw - alpha*(direction-1))*dist_acc))
                    y2 = int(math.ceil(y-math.sin(yaw - alpha*(direction-1))*dist_acc))
                    dist_acc += 1
                    if (((0 < x2) and (x2  < width)) and ((0 < y2) and (y2 < height))):
                        if map_file[y2,x2] == 1:
                            det = True
                    else:
                        dist_acc=0
                        break
                ar[y,x,direction] = dist_acc
    np.save("mat_map.npy",ar)
 
map_mat = np.load("map.npy")
create_mat(map_mat)
map_mat = np.load("mat_map.npy")
for a in range(3):
    plt.imshow(map_mat[:,:,a])
    plt.show()
