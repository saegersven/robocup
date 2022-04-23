#pragma once

#define ARM_UP 75
#define ARM_DOWN -90
#define ARM_ALMOST_DOWN -81
#define ARM_DROP -10

#define GRAB_OPEN -20
#define GRAB_CLOSED -100

#define REFLECTANCE_THRESHOLD_VICTIM 1000

// Corner positions
#define CORNER_POS_SHORT_LEFT 0
#define CORNER_POS_SHORT_RIGHT 1
#define CORNER_POS_LONG_LEFT 2
#define CORNER_POS_LONG_RIGHT 3

#define WALL_LONG(var) (var & 0x02)
#define WALL_RIGHT(var) (var & 0x01)

enum VictimType {
	ALIVE,
	DEAD
};

struct Victim {
	float screen_size;
	float x_pos, y_pos;
	VictimType type;
};

class Rescue {
private:
	const int ROI_X = 0;
	const int ROI_WIDTH = 639;
	const int ROI_Y = 0;
	const int ROI_HEIGHT = 479;

	std::mutex rescue_mutex;

	std::shared_ptr<Robot> robot;
	std::thread::native_handle_type native_handle;

	cv::VideoCapture cap;
	cv::Mat frame;
	cv::Mat debug_frame;

	void rescue();
	void find_black_corner();
	bool is_corner();
	void drive_to_black_corner();
	void find_exit();

	bool get_largest_circle(cv::Mat roi, cv::Vec3f& out, bool ignore_dead);
	bool find_victim(bool ignore_dead);

public:
	std::atomic<bool> finished;
	Rescue(std::shared_ptr<Robot> robot);
	void start();
	void stop();
};