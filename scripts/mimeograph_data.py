# Mimeographs given images by flipping and chaning the contrast to decrease training data for tensorflow model

import cv2
import os
from PIL import Image,ImageFilter 
from PIL import ImageEnhance
import shutil
import errno
from skimage.util import random_noise
import numpy as np



leftPath = r"E:\Robocup\robocup\ml\green\data\left" # path to mimeograph images
rightPath = r"E:\Robocup\robocup\ml\green\data\right" # path to mimeograph images

tempPath = r"E:\Robocup\robocup\ml\green\temp" # path to mimeograph images

newLeftPath = r"E:\Robocup\robocup\ml\green\data_new\left" # path were ALL images are saved (both original, flipped and those with changed contrast)
newRightPath = r"E:\Robocup\robocup\ml\green\data_new\right" # path were ALL images are saved (both original, flipped and those with changed contrast)


# flip left images to temp folder:
fileNames = os.listdir(leftPath)
for fileName in fileNames:
	curPath = str(leftPath + "\\" + fileName)
	image = cv2.imread(curPath)
	image = cv2.flip(image, 1)
	pathToSave = str(tempPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " f=90.jpg"
	cv2.imwrite(pathToSave, image)


# flip right images to left folder:
fileNames = os.listdir(rightPath)
for fileName in fileNames:
	curPath = str(rightPath + "\\" + fileName)
	image = cv2.imread(curPath)
	image = cv2.flip(image, 1)
	pathToSave = str(leftPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " f=90.jpg"
	cv2.imwrite(pathToSave, image)


# copy images from temp folder (flipped left images) to right:
fileNames = os.listdir(tempPath)
for fileName in fileNames:
	curPath = str(tempPath + "\\" + fileName)
	image = cv2.imread(curPath)
	pathToSave = str(rightPath + "\\" + fileName)
	cv2.imwrite(pathToSave, image)

# change contrast from left and save new images to newLeft folder
fileNames = os.listdir(leftPath)
for fileName in fileNames:
	curPath = str(leftPath + "\\" + fileName)
	image = Image.open(curPath)
	enh = ImageEnhance.Contrast(image)
	out = enh.enhance(0.8) # 0.8, 1.2, 1.4

	pathToSave = str(newLeftPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " c=0,8.jpg"
	out.save(pathToSave)

# change contrast from left and save new images to newLeft folder
fileNames = os.listdir(leftPath)
for fileName in fileNames:
	curPath = str(leftPath + "\\" + fileName)
	image = Image.open(curPath)
	enh = ImageEnhance.Contrast(image)
	out = enh.enhance(1.2) # 0.8, 1.2, 1.4

	pathToSave = str(newLeftPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " c=1,2.jpg"
	out.save(pathToSave)

# change contrast from left and save new images to newLeft folder
fileNames = os.listdir(leftPath)
for fileName in fileNames:
	curPath = str(leftPath + "\\" + fileName)
	image = Image.open(curPath)
	enh = ImageEnhance.Contrast(image)
	out = enh.enhance(1.4) # 0.8, 1.2, 1.4

	pathToSave = str(newLeftPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " c=1,4.jpg"
	out.save(pathToSave)





# change contrast from right and save new images to newRight folder
fileNames = os.listdir(rightPath)
for fileName in fileNames:
	curPath = str(rightPath + "\\" + fileName)
	image = Image.open(curPath)
	enh = ImageEnhance.Contrast(image)
	out = enh.enhance(0.8) # 0.8, 1.2, 1.4

	pathToSave = str(newRightPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " c=0,8.jpg"
	out.save(pathToSave)

# change contrast from right and save new images to newRight folder
fileNames = os.listdir(rightPath)
for fileName in fileNames:
	curPath = str(rightPath + "\\" + fileName)
	image = Image.open(curPath)
	enh = ImageEnhance.Contrast(image)
	out = enh.enhance(1.2) # 0.8, 1.2, 1.4

	pathToSave = str(newRightPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " c=1,2.jpg"
	out.save(pathToSave)

# change contrast from right and save new images to newRight folder
fileNames = os.listdir(rightPath)
for fileName in fileNames:
	curPath = str(rightPath + "\\" + fileName)
	image = Image.open(curPath)
	enh = ImageEnhance.Contrast(image)
	out = enh.enhance(1.4) # 0.8, 1.2, 1.4

	pathToSave = str(newRightPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " c=1,4.jpg"
	out.save(pathToSave)


# copy images from left folder to newLeft:
fileNames = os.listdir(leftPath)
for fileName in fileNames:
	curPath = str(leftPath + "\\" + fileName)
	image = cv2.imread(curPath)
	pathToSave = str(newLeftPath + "\\" + fileName)
	cv2.imwrite(pathToSave, image)

# copy images from right folder to newRight:
fileNames = os.listdir(rightPath)
for fileName in fileNames:
	curPath = str(rightPath + "\\" + fileName)
	image = cv2.imread(curPath)
	pathToSave = str(newRightPath + "\\" + fileName)
	cv2.imwrite(pathToSave, image)



# delete images in temp folder 
fileNames = os.listdir(tempPath)
for fileName in os.listdir(tempPath):
	file_path = os.path.join(tempPath, fileName)
	try:
		if os.path.isfile(file_path) or os.path.islink(file_path):
			os.unlink(file_path)
		elif os.path.isdir(file_path):
			shutil.rmtree(file_path)
	except Exception as e:
		print('Failed to delete %s. Reason: %s' % (file_path, e))

# change brightness for images in newLeftPath and copy to temp folder
fileNames = os.listdir(newLeftPath)
for fileName in fileNames:
	curPath = str(newLeftPath + "\\" + fileName)
	image = Image.open(curPath)
	enhancer = ImageEnhance.Brightness(image)
	im_output = enhancer.enhance(0.8)
	pathToSave = str(tempPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " b=0,8.jpg"
	im_output.save(pathToSave)

fileNames = os.listdir(newLeftPath)
for fileName in fileNames:
	curPath = str(newLeftPath + "\\" + fileName)
	image = Image.open(curPath)
	enhancer = ImageEnhance.Brightness(image)
	im_output = enhancer.enhance(0.9)
	pathToSave = str(tempPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " b=0,9.jpg"
	im_output.save(pathToSave)

fileNames = os.listdir(newLeftPath)
for fileName in fileNames:
	curPath = str(newLeftPath + "\\" + fileName)
	image = Image.open(curPath)
	enhancer = ImageEnhance.Brightness(image)
	im_output = enhancer.enhance(1.1)
	pathToSave = str(tempPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " b=1,1.jpg"
	im_output.save(pathToSave)

fileNames = os.listdir(newLeftPath)
for fileName in fileNames:
	curPath = str(newLeftPath + "\\" + fileName)
	image = Image.open(curPath)
	enhancer = ImageEnhance.Brightness(image)
	im_output = enhancer.enhance(1.2)
	pathToSave = str(tempPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " b=1,2.jpg"
	im_output.save(pathToSave)

fileNames = os.listdir(newLeftPath)
for fileName in fileNames:
	curPath = str(newLeftPath + "\\" + fileName)
	image = Image.open(curPath)
	enhancer = ImageEnhance.Brightness(image)
	im_output = enhancer.enhance(1.3)
	pathToSave = str(tempPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " b=1,3.jpg"
	im_output.save(pathToSave)

# copy images from temp folder (images from newLeft with different brightness) to newLeft:
fileNames = os.listdir(tempPath)
for fileName in fileNames:
	curPath = str(tempPath + "\\" + fileName)
	image = cv2.imread(curPath)
	pathToSave = str(newLeftPath + "\\" + fileName)
	cv2.imwrite(pathToSave, image)



# delete images in temp folder 
fileNames = os.listdir(tempPath)
for fileName in os.listdir(tempPath):
	file_path = os.path.join(tempPath, fileName)
	try:
		if os.path.isfile(file_path) or os.path.islink(file_path):
			os.unlink(file_path)
		elif os.path.isdir(file_path):
			shutil.rmtree(file_path)
	except Exception as e:
		print('Failed to delete %s. Reason: %s' % (file_path, e))

# change brightness for images in newRightPath and copy to temp folder
fileNames = os.listdir(newRightPath)
for fileName in fileNames:
	curPath = str(newRightPath + "\\" + fileName)
	image = Image.open(curPath)
	enhancer = ImageEnhance.Brightness(image)
	im_output = enhancer.enhance(0.8)
	pathToSave = str(tempPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " b=0,8.jpg"
	im_output.save(pathToSave)

fileNames = os.listdir(newRightPath)
for fileName in fileNames:
	curPath = str(newRightPath + "\\" + fileName)
	image = Image.open(curPath)
	enhancer = ImageEnhance.Brightness(image)
	im_output = enhancer.enhance(0.9)
	pathToSave = str(tempPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " b=0,9.jpg"
	im_output.save(pathToSave)

fileNames = os.listdir(newRightPath)
for fileName in fileNames:
	curPath = str(newRightPath + "\\" + fileName)
	image = Image.open(curPath)
	enhancer = ImageEnhance.Brightness(image)
	im_output = enhancer.enhance(1.1)
	pathToSave = str(tempPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " b=1,1.jpg"
	im_output.save(pathToSave)

fileNames = os.listdir(newRightPath)
for fileName in fileNames:
	curPath = str(newRightPath + "\\" + fileName)
	image = Image.open(curPath)
	enhancer = ImageEnhance.Brightness(image)
	im_output = enhancer.enhance(1.2)
	pathToSave = str(tempPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " b=1,2.jpg"
	im_output.save(pathToSave)

fileNames = os.listdir(newRightPath)
for fileName in fileNames:
	curPath = str(newRightPath + "\\" + fileName)
	image = Image.open(curPath)
	enhancer = ImageEnhance.Brightness(image)
	im_output = enhancer.enhance(1.3)
	pathToSave = str(tempPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " b=1,3.jpg"
	im_output.save(pathToSave)

# copy images from temp folder (images from newRight with different brightness) to newRight:
fileNames = os.listdir(tempPath)
for fileName in fileNames:
	curPath = str(tempPath + "\\" + fileName)
	image = cv2.imread(curPath)
	pathToSave = str(newRightPath + "\\" + fileName)
	cv2.imwrite(pathToSave, image)

# delete images in temp folder 
fileNames = os.listdir(tempPath)
for fileName in os.listdir(tempPath):
	file_path = os.path.join(tempPath, fileName)
	try:
		if os.path.isfile(file_path) or os.path.islink(file_path):
			os.unlink(file_path)
		elif os.path.isdir(file_path):
			shutil.rmtree(file_path)
	except Exception as e:
		print('Failed to delete %s. Reason: %s' % (file_path, e))

# add noise to images in newLeft folder and copy to temp:
fileNames = os.listdir(newLeftPath)
for fileName in fileNames:
	curPath = str(newLeftPath + "\\" + fileName)
	image = cv2.imread(curPath)
	image = random_noise(image, mode='s&p',amount=0.01)
	image = np.array(255*image, dtype = 'uint8')
	pathToSave = str(tempPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " n=0.01.jpg"
	cv2.imwrite(pathToSave, image)

# copy images from temp folder (noisy newLeft images) to newLeft:
fileNames = os.listdir(tempPath)
for fileName in fileNames:
	curPath = str(tempPath + "\\" + fileName)
	image = cv2.imread(curPath)
	pathToSave = str(newLeftPath + "\\" + fileName)
	cv2.imwrite(pathToSave, image)


# delete images in temp folder 
fileNames = os.listdir(tempPath)
for fileName in os.listdir(tempPath):
	file_path = os.path.join(tempPath, fileName)
	try:
		if os.path.isfile(file_path) or os.path.islink(file_path):
			os.unlink(file_path)
		elif os.path.isdir(file_path):
			shutil.rmtree(file_path)
	except Exception as e:
		print('Failed to delete %s. Reason: %s' % (file_path, e))

# add noise to images in newRight folder and copy to temp:
fileNames = os.listdir(newRightPath)
for fileName in fileNames:
	curPath = str(newRightPath + "\\" + fileName)
	image = cv2.imread(curPath)
	image = random_noise(image, mode='s&p',amount=0.01)
	image = np.array(255*image, dtype = 'uint8')
	pathToSave = str(tempPath + "\\" + fileName)
	pathToSave = pathToSave[:-4]
	pathToSave = pathToSave + " n=0.01.jpg"
	cv2.imwrite(pathToSave, image)

# copy images from temp folder (noisy newRight images) to newRight:
fileNames = os.listdir(tempPath)
for fileName in fileNames:
	curPath = str(tempPath + "\\" + fileName)
	image = cv2.imread(curPath)
	pathToSave = str(newRightPath + "\\" + fileName)
	cv2.imwrite(pathToSave, image)