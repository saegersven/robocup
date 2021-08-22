#pragma once
#include "utils.h"
#include "neural_networks.h"

#define GREEN_NN_ID 0
#define SILVER_NN_ID 1

#define GREEN_RESULT_NO_INTERSECTION 0
#define GREEN_RESULT_LEFT 1
#define GREEN_RESULT_RIGHT 2
#define GREEN_RESULT_DEAD_END 3

#define FOLLOW_HORIZONTAL_SENSITIVITY 10.0f
#define FOLLOW_ANGLE_SENSITIVITY 10.0f

#define FOLLOW_MOTOR_SPEED 30.0f

class Line {
private:
	//int front_cam_id;
	NeuralNetworks neural_networks;
	Robot* robot;

	bool running = false;

	int8_t last_line_x;
	int8_t last_line_angle;
	
	bool is_black(cv::Mat& in, uint8_t x, uint8_t y);
	uint8_t green(cv::Mat& frame);

public:
	Line(Robot* robot, NeuralNetworks neural_networks);
	void start();
	void stop();
	void line(cv::Mat& frame);
}