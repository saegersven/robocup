#pragma once
#ifdef ENABLE_ML
#include <vector>
#include <memory>

#include <opencv2/opencv.hpp>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/tools/gen_op_registration.h"

#define WIDTH 80
#define HEIGHT 48
#define CHANNELS 3

class NeuralNetworks {
private:
	std::vector<std::shared_ptr<tflite::FlatBufferModel>> models;
	std::vector<std::shared_ptr<tflite::Interpreter>> interpreters;

public:
	void load_model(const std::string& path);
	int infere(int id, cv::Mat& frame, float& confidence);
};
#endif