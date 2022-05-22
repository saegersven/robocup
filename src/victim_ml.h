#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

#include <opencv2/opencv.hpp>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/optional_debug_tools.h"

struct Victim {
	bool dead;
	float x, y;
};

class VictimML {
private:
	std::unique_ptr<tflite::FlatBufferModel> model;
	std::unique_ptr<tflite::Interpreter> interpreter;

	float* input_layer;
	float* output_layer;

public:
	VictimML();
	cv::Mat invoke(cv::Mat image);
	std::vector<Victim> extract_victims(cv::Mat probability_map);
};