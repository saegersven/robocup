import numpy as np
import tensorflow as tf
import os
import math
import time
import cv2

from tensorflow import keras
from keras.preprocessing import image

model = keras.models.load_model("model.h5")

model.compile(optimizer="adam",
	loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
	metrics=["accuracy"])

cap = cv2.VideoCapture('/dev/cams/front')

while True:
	cap.grab()
	frame = cap.retrieve()

	roi = frame[24:43, 15:67]


	predictions = model.predict(roi)

	print(predictions)

	cv2.imshow("ROI", roi)
	cv2.waitKey(1)