#include "neural_networks.h"

void NeuralNetworks::load_model(std::string& path) {
	// Load model from file
	this->models.push_back(tflite::FlatBufferModel::BuildFromFile(path));

	// Build interpreter
	tflite::ops::builtin::BuiltinOpResolver resolver;
	interpreters.resize(this->models.size());
	tflite::InterpreterBuilder(*model, resolver)(&this->interpreters[id]);
}

int NeuralNetworks::infere(int id, cv::Mat& frame, float& confidence) {
	interpreters[id]->AllocateTensors(); // TODO: Maybe move to constructor

	// Get input and output layers
	float* input_layer = interpreters[id]->typed_input_tensor<float>(0);
	float* output_layer = interpreters[id]->typed_output_tensor<float>(0);

	// Flatten RGB image to input layer
	float* input_image_ptr = frame.ptr<float>(0);
	memcpy(input_layer, input_image_ptr, WIDTH * HEIGHT * CHANNELS * sizeof(float));

	// Compute model instance
	interpreters[id]->Invoke();

	// Output layer consists of four floats, ordered as in the GreenResult enum
	// Maximum value is the prediction of the NN
	int max_index = 0;
	confidence = std::numeric_limits<float>::lowest();

	for(int i = 0; i < 4; i++) {
		if(output_layer[i] > confidence) {
			confidence = output_layer[i];
			max_index = i;
		}
	}
	return max_index;
}