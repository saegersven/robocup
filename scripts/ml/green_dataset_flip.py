import cv2
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
