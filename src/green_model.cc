#include "green_model.h"

GreenModel::GreenModel() {
	// Load model from file
	this->model = tflite::FlatBufferModel::BuildFromFile(RELATIVE_MODEL_PATH);

	// Build interpreter
	tflite::ops::builtin::BuiltinOpResolver resolver;
	tflite::InterpreterBuilder(*model, resolver)(&this->interpreter);
}

GreenModel::~GreenModel() {
	// TODO
}

GreenResult GreenModel::evaluate(cv::Mat& frame) {
	interpreter->AllocateTensors(); // TODO: Maybe move to constructor

	// Get input and output layers
	float* input_layer = interpreter->typed_input_tensor<float>(0);
	float* output_layer = interpreter->typed_output_tensor<float>(0);

	// Flatten RGB image to input layer
	float* input_image_ptr = frame.ptr<float>(0);
	memcpy(input_layer, input_image_ptr, WIDTH * HEIGHT * CHANNELS * sizeof(float));

	// Compute model instance
	interpreter->Invoke();

	// Output layer consists of four floats, ordered as in the GreenResult enum
	int max_index = 0;
	float max = std::numeric_limits<float>::lowest();

	for(int i = 0; i < 4; i++) {
		if(output_layer[i] > max) {
			max = output_layer[i];
			max_index = i;
		}
	}
	return GreenResult(max_index);
}