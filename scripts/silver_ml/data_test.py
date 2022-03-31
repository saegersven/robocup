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

class_names = ["left", "right"]

data_test_dir = "../../ml_data/silver/data_test/silver"

test_images = []
for img in os.listdir(data_test_dir):
	img = image.load_img(data_test_dir + "/" + img, target_size=(19, 52)
	#color_mode="grayscale"
	)
	img = image.img_to_array(img)
	img = np.expand_dims(img, axis=0)
	test_images.append(img)
test_images = np.vstack(test_images)

model = keras.models.load_model("model")

model.compile(optimizer="adam",
	loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
	metrics=["accuracy"])

arr = model.get_weights()[0]
arr2 = np.zeros(len(arr))

for i in range(len(arr)):
	arr2[i] = arr[i][1]

arr2 = ((arr2 + 1) * 127).astype('uint8').reshape(19, 52, 3)

cv2.imshow("Weights", arr2)
cv2.waitKey(6000)

#predictions = model.predict(test_images)

#print(np.around(predictions,2))