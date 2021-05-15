import cv2
import glob
import numpy as np
import os


def logOdd(x):
    y = (1 - x) * (x > 0) + 0.0001
    return np.log(x / y + 0.0001)

dir_path = os.path.dirname(os.path.realpath(__file__))
IMAGE_FOLDER_PATH = dir_path + '/../images/'
IMAGE_FILE_REGEX = IMAGE_FOLDER_PATH + '*.png'
MAP_FOLDER_PATH = dir_path + '/../saves/'

paths = glob.glob(IMAGE_FILE_REGEX)
for i, path in enumerate(paths):
    print(i, path, '\t\t',i)
idx = int(input('please select a file: ').strip())

img = cv2.imread(paths[idx])
img = 255 - cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# cv2.imshow('reverted', img)
# cv2.waitKey(0)

grid = logOdd(img / 255)

filename = '_'.join(input('please enter filename: ').strip().split())
np.save(MAP_FOLDER_PATH + filename, grid)