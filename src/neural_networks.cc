#include "neural_networks.h"

void NeuralNetworks::load_model(std::string& path) {
	int id = this->models.size();

	// Load model from file
	this->models.push_back(tflite::FlatBufferModel::BuildFromFile(path));

	tflite::ops::builtin::BuiltinOpResolver resolver;

	// Make space for new interpreter
	interpreters.resize(this->models.size());

	tflite::InterpreterBuilder(*model, resolver)(&this->interpreters[id]);
	interpreters[id]->AllocateTensors();
}

int NeuralNetworks::infere(int id, cv::Mat& frame, float& confidence) {
	// Get input and output layers
	float* input_layer = interpreters[id]->typed_input_tensor<float>(0);
	float* output_layer = interpreters[id]->typed_output_tensor<float>(0);

	CV_Assert(in.depth() == cv::CV_32F || in.depth() == cv::CV_8U);
	//CV_Assert(in.channels() == 3);

	// Flatten BGR image to input layer
	int rows = frame.rows;
	int cols = frame.cols * in.channels();

	// Sometimes frame is not continuous: The rows are not stored in sequence.
	// We have to get a new pointer for every row. If the matrix is continuous,
	// set rows to 1, so the loop will only execute once.
	if(in.isContinuous()) {
		cols *= rows;
		rows = 1;
	}

	switch(in.depth()) {
		case cv::CV_32F:
			float* p;
			for(int i = 0; i < rows; ++i) {
				p = in.ptr<float>(i);
				memcpy(input_layer, p, cols * sizeof(float));
			}
			break;
		case cv::CV_8U:
			uint8_t* p;
			for(int i = 0; i < rows; ++i) {
				p = in.ptr<uint8_t>(i);
				memcpy(input_layer, p, cols * sizeof(uint8_t))
			}
			break;
		default:
			break;
	}

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