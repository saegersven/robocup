#pragma once

#include <memory>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/model.h"

class SilverML {
private:
	std::unique_ptr<tflite::FlatBufferModel> model;
	std::unique_ptr<tflite::Interpreter> interpreter;

public:
	SilverML();
	bool predict_silver(cv::Mat image);
};