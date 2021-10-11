#pragma once
#include <opencv2/opencv.hpp>

#include "robot.h"

#define GREEN_NN_ID 0
#define SILVER_NN_ID 1

#define GREEN_RESULT_NO_INTERSECTION 0
#define GREEN_RESULT_LEFT 1
#define GREEN_RESULT_RIGHT 2
#define GREEN_RESULT_DEAD_END 3

#define FOLLOW_HORIZONTAL_SENSITIVITY 75.0f
#define FOLLOW_ANGLE_SENSITIVITY 0.6f

#define FOLLOW_MOTOR_SPEED 15

#define RUNTIME_AVERAGE_SILVER_PATH "../runtime_data/average_silver.png"
#define SILVER_X 27, 54
#define SILVER_Y 30, 36

#define BLACK_Y_TOP_OFFSET 22
#define BLACK_Y_BOTTOM_OFFSET 38

class Line {
private:
	int front_cam_id;
	std::shared_ptr<Robot> robot;

	cv::Mat average_silver;

	bool running = false;

	float last_line_pos;
	int8_t last_line_angle;
	
	bool is_black(uint8_t b, uint8_t g, uint8_t r);
	uint8_t green(cv::Mat& frame);

	float motor_weight(float line_pos);
	float line_weight(float distance);
	float pixel_weight(float distance);
	float average_black(cv::Mat in, uint32_t& num_pixels);

	cv::Mat in_range_black(cv::Mat& in);

	void follow(cv::Mat& frame);

	bool check_silver(cv::Mat& frame);

public:
	Line(int front_cam_id, std::shared_ptr<Robot> robot);
	void start();
	void stop();
	void line(cv::Mat& frame);
};