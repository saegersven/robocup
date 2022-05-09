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


CLASSES = ["no_victim", "living", "dead"]

data_test_dir = "../../ml_data/victims/input_regions_test"

test_images = []
for img in os.listdir(data_test_dir):
	img = image.load_img(data_test_dir + "/" + img, target_size=(480, 40), color_mode="grayscale")
	img = np.expand_dims(img, axis=0)
	test_images.append(img)
test_images = np.vstack(test_images)

model = keras.models.load_model("model.h5")

model.compile(optimizer="adam",
	loss=tf.keras.losses.MeanSquaredError(),
	metrics=["accuracy"])

predictions = model.predict(test_images)

print(np.round(predictions, 2))