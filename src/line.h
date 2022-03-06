#pragma once
#include <opencv2/opencv.hpp>

#include "robot.h"


#define BLACK_MAX_SUM 250

#define GREEN_RATIO_THRESHOLD 0.6f
#define GREEN_MIN_VALUE 5

#define BLUE_RATIO_THRESHOLD 1.7f
#define BLUE_MIN_VALUE 50

#define GREEN_RESULT_NO_INTERSECTION 0
#define GREEN_RESULT_LEFT 1
#define GREEN_RESULT_RIGHT 2
#define GREEN_RESULT_DEAD_END 3

// LINE FOLLOWING
#define FOLLOW_P_FACTOR 70.0f
#define FOLLOW_I_FACTOR 10.0f
#define FOLLOW_LAST_I_FACTOR 0.75f

#define MINIMUM_DISTANCE 10.0f
#define MAXIMUM_DISTANCE 40.0f

#define FOLLOW_MOTOR_SPEED 35

#define RUNTIME_DISTANCE_WEIGHT_MAP_PATH "/home/pi/robocup/runtime_data/distance_weights.png"

// SILVER DETECTION
#define RUNTIME_AVERAGE_SILVER_PATH "/home/pi/robocup/runtime_data/average_silver.png"
#define SILVER_X 29, 54
#define SILVER_Y 21, 31

// OBSTACLE
#define OBSTACLE_Y_UPPER 30
#define OBSTACLE_Y_LOWER 10
#define OBSTACLE_X_UPPER 60
#define OBSTACLE_X_LOWER 20

#define TURN_100_90 330

#define DEBUG
//#define FPS_COUNTER
//#define DEBUG_RESIZE
//#define MOVEMENT_OFF

bool is_black(uint8_t b, uint8_t g, uint8_t r);

bool is_green(uint8_t b, uint8_t g, uint8_t r);

bool is_blue(uint8_t b, uint8_t g, uint8_t r);

// A group of pixels,
// x and y are the average coordinates
// num_pixels is the number of pixels making up this group
// Returned by find_groups
struct Group {
	float x, y;
	uint32_t num_pixels;
};

class Line {
private:
	float fps = 0.0f;
	std::chrono::time_point<std::chrono::high_resolution_clock> last_frame_t;
	int front_cam_id;
	std::shared_ptr<Robot> robot;

	cv::Mat average_silver;
	cv::Mat distance_weight_map;

	std::atomic<bool> running;

	std::atomic<int> obstacle_active;

	cv::Mat debug_frame;
	uint64_t micros_start;

	float last_line_pos;
	float last_line_angle;

	float line_angle_integral;
	std::chrono::time_point<std::chrono::high_resolution_clock> last_update;
	
	bool abort_obstacle(cv::Mat frame);
	void obstacle();
	bool obstacle_straight_line(uint32_t duration);

	float difference_weight(float x);
	float distance_weight(float x);

	float circular_line(cv::Mat& in);

	bool check_silver(cv::Mat& frame);
	bool black_pixel_threshold_under(int threshold);

	//std::vector<cv::Point> find_green_group_centers_old(cv::Mat frame, cv::Mat& green);

	std::vector<Group> find_groups(cv::Mat frame, cv::Mat& ir, std::function<bool (uint8_t, uint8_t, uint8_t)> f);
	void add_to_group_center(int x_pos, int y_pos, cv::Mat ir, uint32_t& num_pixels, float& center_x, float& center_y);

	uint8_t green_direction(cv::Mat& frame, cv::Mat& black, float& global_average_x, float& global_average_y);
	void green(cv::Mat& frame, cv::Mat& black);

	void follow(cv::Mat& frame, cv::Mat black);

	void rescue_kit(cv::Mat& frame);

public:
	Line(int front_cam_id, std::shared_ptr<Robot> robot);
	void start();
	void stop();
	bool line(cv::Mat& frame);
};