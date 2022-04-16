import tensorflow as tf
from tensorflow import keras

# Open with keras first (direct conversion from SavedModel does not work for some reason)
model = keras.models.load_model("model")

model.compile(optimizer="adam",
	loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
	metrics=["accuracy"])

# Convert to TensorFlow Lite model
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

# Save tflite model to file
with open("model.tflite", "wb") as f:
	f.write(tflite_model)