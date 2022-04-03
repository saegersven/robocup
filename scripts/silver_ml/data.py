# Reference: https://www.tensorflow.org/tutorials/images/classification
import matplotlib.pyplot as plt
import numpy as np
import os
import PIL
import tensorflow as tf
import pathlib

from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.models import Sequential

data_dir = pathlib.Path("../../ml_data/silver/data")

image_count = len(list(data_dir.glob("*/*.png")))
print(f"Image count = {image_count}")

left = list(data_dir.glob("silver/*"))

right = list(data_dir.glob("no_silver/*"))

batch_size = 32
img_height = 19
img_width = 52

train_ds = tf.keras.preprocessing.image_dataset_from_directory(
	data_dir,
	validation_split=0.2,
	subset="training",
	seed=123,
	image_size=(img_height, img_width),
	batch_size=batch_size,
	#color_mode="grayscale"
	)

val_ds = tf.keras.preprocessing.image_dataset_from_directory(
	data_dir,
	validation_split=0.2,
	subset="validation",
	seed=123,
	image_size=(img_height, img_width),
	batch_size=batch_size,
	#color_mode="grayscale"
	)

class_names = train_ds.class_names
print(f"Classes: {class_names}")

num_classes = len(class_names)

AUTOTUNE = tf.data.AUTOTUNE

train_ds = train_ds.cache().shuffle(1000).prefetch(buffer_size=AUTOTUNE)
val_ds = val_ds.cache().prefetch(buffer_size=AUTOTUNE)

normalization_layer = layers.experimental.preprocessing.Rescaling(1./255)

normalized_train_ds = train_ds.map(lambda x, y: (normalization_layer(x), y))
normalized_val_ds = val_ds.map(lambda x, y: (normalization_layer(x), y))


model = Sequential([
	layers.Input(shape=(img_height, img_width, 3), dtype="float32"),
	#layers.experimental.preprocessing.Rescaling(1./255, input_shape=(img_height, img_width, 3)),
	#layers.Conv2D(16, 3, padding="same", activation="relu"),
	#layers.MaxPooling2D(),
	#layers.Conv2D(32, 3, padding="same", activation="relu"),
	#layers.MaxPooling2D(),
	#layers.Conv2D(64, 3, padding="same", activation="relu"),
	#layers.MaxPooling2D(),
	layers.Flatten(),
	#layers.Dense(256, activation="relu"), # 128
	layers.Dense(32, activation="relu"), # 128
	#layers.Dense(32, activation="relu"),
	layers.Dense(num_classes, activation="softmax")
])

model.compile(optimizer="adam",
	loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=False),
	metrics=["accuracy"])

model.summary()

epochs = 20
history = model.fit(
	normalized_train_ds,
	validation_data=normalized_val_ds,
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