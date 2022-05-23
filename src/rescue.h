#pragma once

#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>

#include <opencv2/opencv.hpp>
#include <pthread.h>

#include "line.h"
#include "robot.h"
#include "utils.h"
#include "victim_ml.h"

#define CAM_FAR 0
#define CAM_SHORT 1

#define ARM_UP 75
#define ARM_DOWN -90
#define ARM_ALMOST_DOWN -81
#define ARM_DROP -10

#define GRAB_OPEN -20
#define GRAB_CLOSED -100

class Rescue {
private:
	std::shared_ptr<Robot> robot;
	VictimML victimML;

	std::thread::native_handle_type native_handle;

	const std::string cam_filenames[2] = {
		"/dev/cams/back",
		"/dev/cams/front"
	};
	std::vector<cv::VideoCapture> caps;

	cv::Mat capture(uint8_t cam_id);
	void rescue();
	float get_angle_to_right_wall();
	void find_black_corner();
	bool rescue_victim(bool ignore_dead);

public:
	std::atomic<bool> finished;

	Rescue(std::shared_ptr<Robot> robot);
	void start();
	void stop();
};