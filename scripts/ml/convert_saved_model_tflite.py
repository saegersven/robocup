import tensorflow as tf
from tensorflow import keras

path = input("Path to model")

# Open with keras first (direct conversion from SavedModel does not work for some reason)
model = keras.models.load_model(path)

model.compile(optimizer="adam",
	loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
	metrics=["accuracy"])

# Convert to TensorFlow Lite model
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

# Save tflite model to file
with open(path + ".tflite", "wb") as f:
	f.write(tflite_model)