import cv2
import os, os.path

data_dir = str(input("Input data dir: "))

images_right = os.listdir(os.path.join(data_dir, "right"))
images_left = os.listdir(os.path.join(data_dir, "left"))
images_deadend = os.listdir(os.path.join(data_dir, "deadend"))
images_no_intersection = os.listdir(os.path.join(data_dir, "no_intersection"))