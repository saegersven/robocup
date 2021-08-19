#pragma once

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/tools/gen_op_registration.h"

#define RELATIVE_MODEL_PATH "ml/green/model.tflite"

#define WIDTH 80
#define HEIGHT 48
#define CHANNELS 3

enum class GreenResult {
	no_intersection = 0,
	dead_end = 1,
	left = 2,
	right = 3
}

class GreenModel {
private:
	std::unique_ptr<tflite::FlatBufferModel> model;
	std::unique_ptr<tflite::Interpreter> interpreter;

public:
	GreenModel();
	~GreenModel();
	GreenResult evaluate(cv::Mat& frame);
}