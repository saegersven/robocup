import matplotlib.pyplot as plt
import numpy as np
import os
import PIL
import tensorflow as tf
import pathlib
import csv

from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.models import Sequential
from keras.preprocessing import image

LABEL_CSV_PATH = "out.csv"
INPUT_DIR = "../../ml_data/victims/input_regions_us"

input_height = 480
input_width = 40
batch_size = 16

CLASSES = ["no_victim", "living", "dead"]
CLASS_OUTPUTS = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

"""input_images = []
output_classes = []
for c in range(len(CLASSES)):
	path = INPUT_DIR + "/" + CLASSES[c]
	for img in os.listdir(path):
		in_img = image.load_img(path + "/" + img, target_size=(input_height, input_width), color_mode="grayscale")
		in_img = np.expand_dims(in_img, axis=0)
		input_images.append(in_img)
		output_classes.append(CLASS_OUTPUTS[c])

input_images = np.vstack(input_images)
output_classes = np.vstack(output_classes)"""

train_ds = tf.keras.preprocessing.image_dataset_from_directory(
	INPUT_DIR,
	validation_split=0.2,
	subset="training",
	seed=123,
	image_size=(input_height, input_width),
	batch_size=batch_size,
	color_mode="grayscale"
	)

print(train_ds.class_names)

val_ds = tf.keras.preprocessing.image_dataset_from_directory(
	INPUT_DIR,
	validation_split=0.2,
	subset="validation",
	seed=123,
	image_size=(input_height, input_width),
	batch_size=batch_size,
	color_mode="grayscale"
	)

model = Sequential([
	layers.Rescaling(1./255, input_shape=(input_height, input_width, 1)),
	layers.Conv2D(16, 3, padding='same', activation='relu'),
	layers.Conv2D(32, 3, padding='same', activation='relu'),
	layers.MaxPooling2D(),
	layers.Dropout(0.2),
	layers.Conv2D(32, 3, padding='same', activation='relu'),
	layers.MaxPooling2D(),
	layers.Dropout(0.2),
	layers.Flatten(),
	layers.Dense(16, activation='relu'),
	layers.Dense(8, activation='relu'),
	layers.Dense(len(CLASSES), activation='softmax')
])

model.compile(loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=False),
	optimizer='adam', metrics=['accuracy'])

model.summary()

epochs = 10
history = model.fit(train_ds, validation_data=val_ds,
             epochs=epochs)

model.save("model.h5", save_format='h5')

acc = history.history["accuracy"]
val_acc = history.history["val_accuracy"]

loss = history.history["loss"]
val_loss = history.history["val_loss"]

epochs_range = range(epochs)


# plt.figure(figsize=(10, 10))
# for images, labels in val_ds.take(1):
# 	for i in range(9):
# 		ax = plt.subplot(3, 3, i + 1)
# 		plt.imshow(images[i].numpy().astype("uint8"),
# 		#cmap="gray"
# 		)
# 		plt.title(class_names[labels[i]])
# 		plt.axis("off")


plt.figure(figsize=(8, 8))
plt.subplot(1, 2, 1)
plt.plot(epochs_range, acc, label='Training Accuracy')
plt.plot(epochs_range, val_acc, label='Validation Accuracy')
plt.legend(loc='lower right')
plt.title('Training and Validation Accuracy')

plt.subplot(1, 2, 2)
plt.plot(epochs_range, loss, label='Training Loss')
plt.plot(epochs_range, val_loss, label='Validation Loss')
plt.legend(loc='upper right')
plt.title('Training and Validation Loss')
plt.show()