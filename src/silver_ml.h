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

class SilverML {
private:
	std::unique_ptr<tflite::FlatBufferModel> model;
	std::unique_ptr<tflite::Interpreter> interpreter;

	float* input_layer;
	float* output_layer;

	std::atomic<bool> running;
	std::atomic<bool> status;
	std::atomic<bool> has_frame;
	cv::Mat current_frame;
	cv::Mat internal_frame;
	std::mutex frame_swap_lock;

public:
	SilverML();
	void start();
	void stop();
	bool predict_silver(cv::Mat image);

	void internal_loop();
};