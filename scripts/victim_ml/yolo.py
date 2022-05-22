import cv2
import os
import numpy as np

import tensorflow as tf
import keras.backend as K
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.models import Sequential
from keras.preprocessing import image

# Input are 640x480 rgb images in folder and CSV file
# Based on YOLO network architecture:
# Image is divided into 8x6 chunks. Each chunk can predict one bounding box
# and class probabilities
# => Output of net is a tensor with shape 8 x 6 x (5 [x, y, w, h, c] + 2 [l, d])
# If no victim in chunk -> l = 0, d = 0, c = 0, x, y, w, h irrelevant (set to 0)
# For images with no victims -> all zero

VICTIMS_IN = "../../ml_data/victims/victims_us"
NO_VICTIMS_IN = "../../ml_data/victims/no_victims"
CSV_PATH = "out_us.csv"

INSPECT_IMAGES = False
targets = []
images = []

input_height = 120
input_width = 160
Xs = 40
Ys = 12
chunk_height = input_height / Ys
chunk_width = input_width / Xs

batch_size = 16

with open(CSV_PATH, "r") as f:
	for row in f.readlines():
		# Example row:
		# 1651946284.400168.png, 1, 0, 390, 55, 455, 124
		# filename, num_victims, victim_class, xmin, ymin, xmax, ymax
		cols = row.split(',')

		image = cv2.imread(VICTIMS_IN + "/" + cols[0], cv2.IMREAD_GRAYSCALE)

		num_victims = int(cols[1])

		target = np.empty((Ys, Xs, 3), dtype=np.float32)

		for y in range(Ys):
			for x in range(Xs):
				v = 0.0
				d = 0.0
				l = 0.0

				x_bb = 0.0
				y_bb = 0.0
				w_bb = 0.0
				h_bb = 0.0
				c_bb = 0.0

				chunk_xmin = x * chunk_width
				chunk_ymin = y * chunk_height
				chunk_xmax = chunk_xmin + chunk_width
				chunk_ymax = chunk_ymin + chunk_height

				for i in range(num_victims):
					j = 2 + i*5
					xmin = float(cols[j + 1])
					ymin = float(cols[j + 2])
					xmax = float(cols[j + 3])
					ymax = float(cols[j + 4])
					x_v = (xmin + xmax) / 2
					y_v = (ymin + ymax) / 2

					# Check if this chunk is responsible for predicting bounding box
					# (Check if the center is inside the chunk)
					if (chunk_xmin < x_v < chunk_xmax) and (chunk_ymin < y_v < chunk_ymax):
						w_bb = xmax - xmin
						h_bb = ymax - ymin
						x_bb = x_v
						y_bb = y_v
						c_bb = 1.0

						# cv2.rectangle(image, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (10), 1)
						# cv2.imshow("Image", image)
						# cv2.waitKey(5)

					leftx = max(xmin, chunk_xmin)
					rightx = min(xmax, chunk_xmax)
					topy = max(ymin, chunk_ymin)
					bottomy = min(ymax, chunk_ymax)

					radius = max(xmax - xmin, ymax - ymin) / 2
					radius2 = radius**2

					# Check if any of the chunk corners is inside the victim
					dist_tl = (chunk_xmin - x_v)**2 + (chunk_ymin - y_v)**2
					dist_bl = (chunk_xmin - x_v)**2 + (chunk_ymax - y_v)**2
					dist_tr = (chunk_xmax - x_v)**2 + (chunk_ymin - y_v)**2
					dist_br = (chunk_xmax - x_v)**2 + (chunk_ymax - y_v)**2

					if(dist_tl <= radius2 or dist_bl <= radius2 or dist_tr <= radius2 or dist_br <= radius2):
						# Set bounding box properties

						v = 1.0
						if(int(cols[j]) == 1):
							# Dead victim
							d = 1.0
							l = 0.0
						else:
							d = 0.0
							l = 1.0

					# # Check if this chunk is responsible for predicting class
					# # (Check if bounding box intersects the chunk)
					# if rightx - leftx > 1 and bottomy - topy > 1:
					# 	if int(cols[j]) == 0:
					# 		# Living victim
					# 		l = 1.0
					# 	else:
					# 		# Dead victim
					# 		d = 1.0
				target[y, x, 0] = v
				target[y, x, 1] = d
				target[y, x, 2] = l
				# target[y, x, 1] = (x_bb - chunk_xmin) / chunk_width
				# target[y, x, 2] = (y_bb - chunk_ymin) / chunk_height
				# target[y, x, 3] = w_bb / input_width
				# target[y, x, 4] = h_bb / input_height
				# target[y, x, 4] = c_bb
				# target[y, x, 5] = l
				# target[y, x, 6] = d
				#target[y, x, 1] = d

				if INSPECT_IMAGES:
					c = 0.5
					if(target[y, x, 0] == 1.0):
						c = 1.0

					for x_ in range(int(chunk_width)):
						for y_ in range(int(chunk_height)):
							image[y * int(chunk_height) + y_, x * int(chunk_width) + x_] *= c

		if INSPECT_IMAGES:
			cv2.imshow("Image", image)
			cv2.waitKey(0)
			print(cols[0])

		targets.append(target)
		images.append(image)

for img in os.listdir(NO_VICTIMS_IN):
	image = cv2.imread(NO_VICTIMS_IN + "/" + img, cv2.IMREAD_GRAYSCALE)
	target = np.zeros((Ys, Xs, 3), dtype=np.float32)

	targets.append(target)
	images.append(image)

images = np.array(images)
targets = np.array(targets)

print(len(images))

# Custom loss function
def loss_fn(y_true, y_pred):
	l_coord = 5
	l_noobj = 0.5
	l = 0.0
	for i in range(len(y_true)):
		for y in range(Ys):
			for x in range(Xs):
				# Is this cell responsible for the bounding box?
				if (y_true[i, y, x, 4] == 1.0):
					l += l_coord * (K.square(y_pred[i, y, x, 0] - y_true[i, y, x, 0]) + K.square(y_pred[i, y, x, 1] - y_true[i, y, x, 1]) +
						K.square(y_pred[i, y, x, 2] - y_true[i, y, x, 2]) + K.square(y_pred[i, y, x, 3] - y_true[i, y, x, 3]))
					l += K.square(y_pred[i, y, x, 4] - y_true[i, y, x, 4])
				else:
					l += l_noobj * K.square(y_pred[i, y, x, 4] - y_true[i, y, x, 4])
				# Does this cell contain a victim?
				if (y_true[i, y, x, 5] == 1.0 or y_true[i, y, x, 6] == 1.0):
					l += K.square(y_pred[i, y, x, 5] - y_true[i, y, x, 5]) + K.square(y_pred[i, y, x, 6] - y_true[i, y, x, 6])
	return l

def alt_loss_fn(y_true, y_pred):
	# a = K.sum(y_true, axis=(0, 1, 2))
	# b = K.sum(y_pred, axis=(0, 1, 2))
	# return K.square(a[0] - b[0]) + K.square(a[1] - b[1])
	return K.square(y_true - y_pred)

model = Sequential([
	layers.Rescaling(1./255, input_shape=(input_height, input_width, 1)),
	layers.Conv2D(8, 3, padding='same', activation='relu'),
	layers.MaxPooling2D(2),
	layers.Conv2D(16, 3, padding='same', activation='relu'),
	layers.MaxPooling2D(6),
	#layers.Conv2D(64, 3, padding='same', activation='relu'),
	#layers.MaxPooling2D(4),
	layers.Dropout(0.2),
	layers.Flatten(),
	#layers.Dense(512, activation='relu'),
	layers.Dense(Ys * Xs * 3, activation='linear'),
	layers.Reshape((Ys, Xs, 3))
])

model.compile(optimizer='adam', loss='mse', metrics=["accuracy"])

model.summary()

model.fit(images, targets, batch_size=batch_size, epochs=50, verbose=1)

model.save('model.h5', save_format='h5')