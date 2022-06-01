# Mimeographs data in given folder

import cv2
import os
from PIL import Image,ImageFilter 
from PIL import ImageEnhance
import shutil
import errno
from skimage.util import random_noise
import numpy as np

pathToMimeographImages = "/data/Programmieren/Robocup/2022/Main Repo 2022/robocup/ml_data/green/no_green" 

"""
# flip images:
fileNames = os.listdir(pathToMimeographImages)
for fileName in fileNames:
	curPath = str(pathToMimeographImages + "/" + fileName)
	image = cv2.imread(curPath)
	image = cv2.flip(image, 1)
	pathToSave = str(pathToMimeographImages + "/" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " f=180.jpg"
	cv2.imwrite(pathToSave, image)

# flip images:
fileNames = os.listdir(pathToMimeographImages)
for fileName in fileNames:
	curPath = str(pathToMimeographImages + "/" + fileName)
	image = cv2.imread(curPath)
	image = cv2.flip(image, 1)
	pathToSave = str(pathToMimeographImages + "/" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " f=1802.jpg"
	cv2.imwrite(pathToSave, image)

# change contrast:
fileNames = os.listdir(pathToMimeographImages)
for fileName in fileNames:
	curPath = str(pathToMimeographImages + "/" + fileName)
	image = Image.open(curPath)
	enh = ImageEnhance.Contrast(image)
	out = enh.enhance(0.8) # 0.8, 1.2, 1.4

	pathToSave = str(pathToMimeographImages + "/" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " c=0,8.jpg"
	out.save(pathToSave)
"""
# change brightness:
fileNames = os.listdir(pathToMimeographImages)
for fileName in fileNames:
	curPath = str(pathToMimeographImages + "/" + fileName)
	image = Image.open(curPath)
	enhancer = ImageEnhance.Brightness(image)
	im_output = enhancer.enhance(0.9)
	pathToSave = str(pathToMimeographImages + "/" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " b=0,8.jpg"
	im_output.save(pathToSave)
"""
# add noise:
fileNames = os.listdir(pathToMimeographImages)
for fileName in fileNames:
	curPath = str(pathToMimeographImages + "/" + fileName)
	image = cv2.imread(curPath)
	image = random_noise(image, mode='s&p',amount=0.01)
	image = np.array(255*image, dtype = 'uint8')
	pathToSave = str(pathToMimeographImages + "/" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " n=0.01.jpg"
	cv2.imwrite(pathToSave, image)
"""