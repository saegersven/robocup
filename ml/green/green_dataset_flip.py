import cv2
<<<<<<< HEAD
import os

path = r'D:\Programmieren\Robocup\2022\Main Repo 2020\robocup\scripts\ml\data\right'
newPath = r'D:\Programmieren\Robocup\2022\Main Repo 2020\robocup\scripts\ml\data\left'
fileNames = os.listdir(path)

for fileName in fileNames:
	curPath = str(path + "\\" + fileName)
	print("Flipping:", curPath)

	image = cv2.imread(curPath)
	image = cv2.flip(image, 1)

	pathToSave = str(newPath + "\\" + fileName)
	pathToSave = pathToSave[:-6]
	fileName = fileName[:-4]
	fileName = fileName + " f" + ".jpg"
	pathToSave = pathToSave + fileName

	cv2.imwrite(pathToSave, image)
=======
import os, os.path

data_dir = str(input("Input data dir: "))

images_right = os.listdir(os.path.join(data_dir, "right"))
images_left = os.listdir(os.path.join(data_dir, "left"))
images_deadend = os.listdir(os.path.join(data_dir, "deadend"))
images_no_intersection = os.listdir(os.path.join(data_dir, "no_intersection"))
>>>>>>> 000b0000787a76aa0dbfa71b7e606c92a739bbdf
