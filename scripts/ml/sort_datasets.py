# short program to make it easier to delete weird images from datasets without the need to open every single file

import cv2
import os
from os import listdir
import random

path = "/mnt/D2F891CDF891AFE9/Programmieren/Robocup/2022/Main Repo 2022/robocup/ml_data/silver/silver_rois/"
new_path = "/home/lukas/Desktop/testdir/"
img_paths = []
for images in os.listdir(path): 
    if (images.endswith(".png")):
        img_paths.append(path + images)

for img_path in img_paths:
    img = cv2.imread(img_path)
    foo = True
    while(foo):
        cv2.imshow("Image", img)
        k = cv2.waitKey(33)
        if k == 121: # if 'y' has been pressed, save img
            cv2.imwrite((new_path + str(random.randint(0, 9999999999999999999999999)) + ".png"), img)
            break
        elif k == 110: # if 'n' has been pressed, don't save img
            foo = False