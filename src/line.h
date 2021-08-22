#pragma once
#include "utils.h"
#include "neural_networks.h"

#define GREEN_NN_ID 0
#define SILVER_NN_ID 1

#define GREEN_RESULT_NO_INTERSECTION 0
#define GREEN_RESULT_LEFT 1
#define GREEN_RESULT_RIGHT 2
#define GREEN_RESULT_DEAD_END 3

class Line {
private:
	//int front_cam_id;
	NeuralNetworks neural_networks;
	Robot* robot;

	bool running = false;

public:
	Line(Robot* robot, NeuralNetworks neural_networks);
	void start();
	void stop();
	void line(cv::Mat& frame);
	uint8_t green(cv::Mat& frame);
}