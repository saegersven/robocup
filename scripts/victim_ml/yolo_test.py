import matplotlib.pyplot as plt
import numpy as np
import pathlib
import tensorflow as tf
import os
import math
import time
import cv2

from tensorflow import keras
from keras.preprocessing import image

Xs = 32
Ys = 24
input_height = 120
input_width = 160
chunk_width = int(input_width / Xs)
chunk_height = int(input_height / Ys)

CLASSES = ["no_victim", "living", "dead"]

data_test_dir = "../../ml_data/victims/test"

test_images = []
for img in os.listdir(data_test_dir):
	img = cv2.imread(data_test_dir + "/" + img, cv2.IMREAD_GRAYSCALE)
	test_images.append(img)
test_images = np.array(test_images)

model = keras.models.load_model("model.h5", compile=False)

t = time.time()
predictions = model.predict(test_images)
print(f"Took: {(time.time() - t) * 1000} ms.")

for i, p in enumerate(predictions):
	image = test_images[i]
	image2 = test_images[i].copy()

	confidence_map = np.zeros((Ys, Xs, 1), dtype=np.uint8)

	for y in range(Ys):
		for x in range(Xs):
			# Draw bounding box along with confidence
			# chunk_xmin = x * 80
			# chunk_ymin = y * 80

			# x_bb = chunk_xmin + p[y, x, 0]*80
			# y_bb = chunk_ymin + p[y, x, 1]*80
			# w_bb = p[y, x, 2] * 320
			# h_bb = p[y, x, 3] * 240
			# xmin = x_bb - 0.5 * w_bb
			# ymin = y_bb - 0.5 * h_bb
			# xmax = xmin + w_bb
			# ymax = ymin + h_bb
			# thickness = int(6 * p[y, x, 4])
			# print(thickness)

			# cv2.rectangle(image, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (10), thickness)
			confidence = min(max(p[y, x, 0], 0.0), 1.0)
			confidence_map[y, x] = int(confidence * 255)
			if(confidence > 0.2):
				print(f"{p[y, x, 0]}, {p[y, x, 1]}")
			for x_ in range(chunk_width):
				for y_ in range(chunk_height):
					image[y * chunk_height + y_, x * chunk_width + x_] *= confidence

	ret, thresholded_confidence = cv2.threshold(confidence_map, 90, 255, cv2.THRESH_BINARY)
	contours, _ = cv2.findContours(thresholded_confidence, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	for c in contours:
		x, y, w, h = cv2.boundingRect(c)
		x *= chunk_width
		y *= chunk_height
		w *= chunk_width
		h *= chunk_height
		cv2.rectangle(image2, (x, y), (x+w, y+h), (10), 1)
		cv2.circle(image2, (int(x+w/2), int(y+h/2)), 2, (10), 2)

	cv2.imshow("Out", image2)
	cv2.imshow("Confidence map", image)
	cv2.waitKey(0)