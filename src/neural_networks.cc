#include "neural_networks.h"

#include <vector>
#include <memory>
#include <chrono>
#include <thread>

#include <opencv2/opencv.hpp>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/tools/gen_op_registration.h"

#include "errcodes.h"

void NeuralNetworks::load_model(const std::string& path) {
	int id = models.size();

	tflite::StderrReporter error_reporter;

	// Load model from file
	models.push_back(tflite::FlatBufferModel::BuildFromFile(path.c_str(), &error_reporter));

	tflite::ops::builtin::BuiltinOpResolver resolver;

	// Make space for new interpreter
	interpreters.resize(models.size());

	std::unique_ptr<tflite::Interpreter> interpreter;

	if(tflite::InterpreterBuilder(*models[id], resolver)(&interpreter) != kTfLiteOk) {
		std::cout << "Error building interpreter" << std::endl;
		exit(ERRCODE_NN_BUILD);
	}

	interpreter->AllocateTensors();

	std::shared_ptr<tflite::Interpreter> interpreter_shared = std::move(interpreter);
	interpreters.push_back(interpreter_shared);
}

int NeuralNetworks::infere(int id, cv::Mat& in, float& confidence) {
	tflite::StderrReporter error_reporter;

	std::unique_ptr<tflite::FlatBufferModel> model =
		tflite::FlatBufferModel::BuildFromFile("/home/pi/robocup/ml/green/model.tflite", &error_reporter);

	tflite::ops::builtin::BuiltinOpResolver resolver;
	std::unique_ptr<tflite::Interpreter> interpreter;
	tflite::InterpreterBuilder(*model, resolver)(&interpreter);

	interpreter->AllocateTensors();

	// Get input and output layers
	uint8_t* input_layer = interpreter->typed_input_tensor<uint8_t>(0);
	float* output_layer = interpreter->typed_output_tensor<float>(0);

	CV_Assert(in.depth() == CV_32F || in.depth() == CV_8U);
	//CV_Assert(in.channels() == 3);

	// Flatten BGR image to input layer
	int rows = in.rows;
	int cols = in.cols * in.channels();

	// Sometimes frame is not continuous: The rows are not stored in sequence.
	// We have to get a new pointer for every row. If the matrix is continuous,
	// set rows to 1, so the loop will only execute once.
	if(in.isContinuous()) {
		cols *= rows;
		rows = 1;
	}

	switch(in.depth()) {
		case CV_32F:
		{
			float* p;
			for(int i = 0; i < rows; ++i) {
				p = in.ptr<float>(i);
				memcpy(input_layer, p, cols * sizeof(float));
			}
			break;
		}
		case CV_8U:
		{
			uint8_t* p;
			for(int y = 0; y < rows; ++y) {
				std::cout << std::to_string(y) << ": ";
				p = in.ptr<uint8_t>(y);
				memcpy(input_layer, p, cols * sizeof(uint8_t));
				// for(int x = 0; x < cols; ++x) {
				// 	std::cout << ".";
				// 	input_layer[y * rows + x] = (float)p[x];
				// }
				// std::cout << std::endl;
			}
			break;
		}
		default:
			break;
	}

	std::cout << "Invoking" << std::endl;
	// Compute model instance
	interpreter->Invoke();

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