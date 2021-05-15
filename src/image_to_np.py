import cv2
import glob
import numpy as np
import os


dir_path = os.path.dirname(os.path.realpath(__file__))
IMAGE_FOLDER_PATH = dir_path + '/../images/'
IMAGE_FILE_REGEX = IMAGE_FOLDER_PATH + '*.png'
MAP_FOLDER_PATH = dir_path + '/../saves/'

paths = glob.glob(IMAGE_FILE_REGEX)
for i, path in enumerate(paths):
    print(i, path, '\t\t',i)
idx = int(input('please select a file: ').strip())

img = cv2.imread(paths[idx])

filename = '_'.join(input('please enter filename: ').strip().split())
np.save(MAP_FOLDER_PATH + filename, img)