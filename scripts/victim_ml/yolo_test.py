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
Ys = 20
Cs = 2
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
	debug_image = image.copy()
	#image2 = test_images[i].copy()

	confidence_map = np.zeros((Ys, Xs), dtype=np.uint8)

	average_d = 0.0
	num_cells = 0

	victims = []

	for y in range(Ys):
		for x in range(Xs):
			# Draw bounding box along with confidence
			# chunk_xmin = x * chunk_width
			# chunk_ymin = y * chunk_height

			# x_bb = chunk_xmin + p[y, x, 1]*80
			# y_bb = chunk_ymin + p[y, x, 2]*80
			# w_bb = p[y, x, 3] * 320
			# h_bb = p[y, x, 4] * 240
			# xmin = x_bb - 0.5 * w_bb
			# ymin = y_bb - 0.5 * h_bb
			# xmax = xmin + w_bb
			# ymax = ymin + h_bb
			# thickness = int(6 * p[y, x, 4])
			# print(thickness)
			
			confidence = min(max(p[y, x, 0], 0.0), 1.0)
			confidence_map[y, x] = int(confidence * 255)

			# if(p[y, x, 0] > 0.25):
			# 	#center_x = int((p[y, x, 3] + x) * chunk_width)
			# 	#center_y = int((p[y, x, 4] + y) * chunk_height)
			# 	center_x = (x + 0.5) * chunk_width
			# 	center_y = (y + 0.5) * chunk_height
			# 	cv2.circle(debug_image, (int(center_x), int(center_y)), 2, (90), 3)

			# print(xmin)

			# if(confidence > 0.2):
			# 	cv2.rectangle(image, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (10), 3)

			# for x_ in range(chunk_width):
			# 	for y_ in range(chunk_height):
			# 		image[y * chunk_height + y_, x * chunk_width + x_] *= confidence



	# cv2.imshow("Debug", debug_image)
	# #cv2.imshow("Output", cv2.resize(p, (160, 120)))
	# cv2.waitKey(0)
	# continue

	# ret, thresholded_confidence = cv2.threshold(confidence_map, 90, 255, cv2.THRESH_BINARY)
	# contours, _ = cv2.findContours(thresholded_confidence, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	# for c in contours:
	# 	x, y, w, h = cv2.boundingRect(c)
	# 	x *= chunk_width
	# 	y *= chunk_height
	# 	w *= chunk_width
	# 	h *= chunk_height
	# 	cv2.rectangle(image2, (x, y), (x+w, y+h), (10), 1)
	# 	cv2.circle(image2, (int(x+w/2), int(y+h/2)), 2, (10), 2)

	# cv2.imshow("Out", image2)
	# cv2.imshow("Confidence map", image)

	p = np.clip(p, 0.0, 1.0)

	zeros = np.zeros((p.shape[0], p.shape[1], 1), dtype=p.dtype)
	p_expanded = np.append(p, zeros, axis=2)

	cv2.imshow("Expanded output", cv2.resize(p_expanded, (160, 120)))

	p = cv2.GaussianBlur(p, (1, 3), cv2.BORDER_DEFAULT)
	#p_resized = cv2.resize(p, (160, 120))
	#cv2.imshow("Probability map", p_resized)

	thresh1 = cv2.inRange(p, (0.32, 0.0), (1.0, 1.0))
	thresh2 = cv2.inRange(p, (0.0, 0.32), (1.0, 1.0))
	thresh = cv2.bitwise_or(thresh1, thresh2)
	cv2.imshow("Thresholded", thresh)

	# kernel = np.ones((3, 3), 'uint8')
	# dilatated_thresh = cv2.dilate(thresh, kernel, iterations=1)
	# dilatated_thresh = cv2.resize(dilatated_thresh, (160, 120))

	#bgr_mask = cv2.cvtColor(dilatated_thresh, cv2.COLOR_GRAY2BGR)

	# masked_image = cv2.bitwise_and(image, image, mask=dilatated_thresh)

	# masked_image = cv2.GaussianBlur(masked_image, (3, 3), cv2.BORDER_DEFAULT, 1.5)

	# circles = cv2.HoughCircles(masked_image, cv2.HOUGH_GRADIENT, 1, 20, param1=20, param2=25, minRadius=0, maxRadius=0)
	# circles = np.uint16(np.around(circles))

	# if circles is not None:
	# 	for i in circles[0,:]:
	# 		cv2.circle(debug_image, (i[0], i[1]), i[2], (0, 255, 0), 2)

	# cv2.imshow("Masked", masked_image)
	# cv2.imshow("Image", debug_image)
	# cv2.waitKey(0)
	# continue

	contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	victims = []

	for contour in contours:
		mask = np.zeros(thresh.shape, np.uint8)
		cv2.drawContours(mask, contour, -1, 255, -1)
		mean = cv2.mean(p, mask=mask)

		dead = mean[0] > mean[1]

		rect = cv2.boundingRect(contour)
		x, y, w, h = rect

		x *= chunk_width
		y *= chunk_height
		w *= chunk_width
		h *= chunk_height

		#print(w * h)
		if(w * h < (360.0/60.0)*y+40.0):
			continue

		# Check if contour is too wide to be one victim
		if(w / h > 2.4):
			# Likely three victims
			pass
		elif(w / h > 1.5):
			# Likely two victims
			victims.append((dead, x+w/4*1, y+h/2, w/2, h))
			victims.append((dead, x+w/4*3, y+h/2, w/2, h))
		else:
			victims.append((dead, x+w/2, y+h/2, w, h))

		#cv2.rectangle(debug_image, (x, y), (x+w, y+h), (10), 2)

	# Debug
	for victim in victims:
		d, x, y, w, h = victim
		
		text = "Alive"
		if d:
			text = "Dead"

		cv2.putText(debug_image, text, (int(x-15), int(y-8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (90), 1, cv2.LINE_AA)
		cv2.circle(debug_image, (int(x), int(y)), 2, (90), 3)

	cv2.imshow("Out", debug_image)
	cv2.waitKey(0)

	# x_points = []
	# column_probabilities = []

	# high_probability_rows = []

	# for x in range(Xs):
	# 	x_points.append(x)
	# 	confidence_sum = 0
	# 	for y in range(Ys):
	# 		confidence_sum += confidence_map[y, x]
	# 		if(confidence_sum > 300):
	# 			high_probability_rows.append(x)

	# 	column_probabilities.append(confidence_sum)


	# plt.plot(x_points, column_probabilities)
	# plt.show()