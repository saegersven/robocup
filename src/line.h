#pragma once
#include <opencv2/opencv.hpp>

#include "robot.h"
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

#define RUNTIME_AVERAGE_SILVER_PATH "../runtime_data/average_silver.png"
#define SILVER_X 29, 52
#define SILVER_Y 27, 33

class Line {
private:
	int front_cam_id;
	NeuralNetworks neural_networks;
	Robot* robot;

	cv::Mat average_silver;

	bool running = false;

	int8_t last_line_x;
	int8_t last_line_angle;
	
	bool is_black(cv::Mat& in, uint8_t x, uint8_t y);
	uint8_t green(cv::Mat& frame);

	void follow(cv::Mat& frame);

	bool check_silver(cv::Mat& frame);

public:
	Line(int front_cam_id, Robot* robot, NeuralNetworks neural_networks);
	void start();
	void stop();
	void line(cv::Mat& frame);
};