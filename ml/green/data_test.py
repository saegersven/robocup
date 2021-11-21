import matplotlib.pyplot as plt
import numpy as np
import pathlib
import tensorflow as tf
import os
import math
import time

from tensorflow import keras
from keras.preprocessing import image

data_test_dir = pathlib.Path("data_test")

class_names = ["deadend", "left", "right"]

test_images = []
for img in os.listdir("data_test"):
	img = image.load_img("data_test/" + img, target_size=(48, 80),
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

t_start = time.time()
predictions = model.predict(test_images)

print(f"Took: {time.time() - t_start}s")

print(predictions)

plt.figure(figsize=(10, 10))
for i in range(min(9, len(test_images))):
	ax = plt.subplot(3, 3, i + 1)
	plt.imshow(test_images[i].astype("uint8"),
	#cmap="gray"
	)
	cl_index = 0
	confidence = -100000.0
	for cl in range(len(predictions[i])):
		if(predictions[i][cl] > confidence):
			cl_index = cl
			confidence = predictions[i][cl]

	plt.title(f"{class_names[cl_index]} ({confidence:.2f})")
	plt.axis("off")
plt.show()