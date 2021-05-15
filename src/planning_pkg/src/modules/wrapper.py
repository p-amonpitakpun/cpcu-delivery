from math import ceil
from cv2 import resize
import numpy

SCALE = 0.5
MAP_SIZE = (250, 250)

def point_wrapper(point):
    return point
    # return ceil(point * SCALE)

def map_wrapper(map):
    return map
    # _map = numpy.array(map)
    # print(_map.shape)
    # return resize(_map, dsize=MAP_SIZE)