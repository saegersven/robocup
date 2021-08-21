#pragma once

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/tools/gen_op_registration.h"

#define WIDTH 80
#define HEIGHT 48
#define CHANNELS 3

class NeuralNetworks {
private:
	std::vector<std::unique_ptr<tflite::FlatBufferModel>> nets;
	std::vector<std::unique_ptr<tflite::Interpreter>> interpreters;

public:
	void load_model(std::string& path);
	int infere(cv::Mat& frame);
}