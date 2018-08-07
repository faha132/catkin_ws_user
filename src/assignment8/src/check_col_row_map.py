import rospkg
import numpy as np
rospack = rospkg.RosPack()
file_path = rospack.get_path('assignment8') + '/src/'
matrix = np.load(file_path + 'matrix100cm_lane1.npy')
matrix2 = np.load(file_path + 'matrix100cm_lane2.npy')
print matrix.shape
print matrix2.shape
