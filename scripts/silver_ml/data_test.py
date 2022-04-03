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

data_test_dir = "../../ml_data/silver/data_test/no_silver"

test_images = []
for img in os.listdir(data_test_dir):
	img = image.load_img(data_test_dir + "/" + img, target_size=(19, 52)
	#color_mode="grayscale"
	)
	img = image.img_to_array(img)
	img = np.expand_dims(img, axis=0)
	test_images.append(img)
test_images = np.vstack(test_images)

model = keras.models.load_model("model.h5")

model.compile(optimizer="adam",
	loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
	metrics=["accuracy"])

predictions = model.predict(test_images)

print(np.around(predictions,2))