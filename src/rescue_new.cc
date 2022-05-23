#include "rescue.h"

#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <pthread.h>

#include "line.h"
#include "robot.h"
#include "utils.h"

Rescue::Rescue(std::shared_ptr<Robot> robot) {
	this->robot = robot;
}

void Rescue::start() {
	std::thread rescue_thread([this]() { this->rescue(); });
	this->native_handle = rescue_thread.native_handle();
	rescue_thread.detach();
}

void Rescue::stop() {
	pthread_cancel(this->native_handle);
}

cv::Mat Rescue::capture(uint8_t cam_id) {
	if(!caps[cam_id].isOpened()) {
		caps[cam_id].open(cam_filenames[cam_id], cv::CAP_V4L2);
		if(!caps[cam_id].isOpened()) {
			std::cout << "Failed opening camera " << std::to_string(cam_id) << std::endl;
		}
	}
	cv::Mat frame;
	caps[cam_id].grab();
	caps[cam_id].retrieve(frame);
	caps[cam_id].release();

	return frame;
}

void Rescue::rescue() {
	caps.push_back(cv::VideoCapture());
	caps.push_back(cv::VideoCapture());

	robot->stop();
	robot->beep(100, BUZZER);
	robot->m(100, 100, 180);

	// Take time to align so that wall is on the right
	if(robot->distance(DIST_SIDE_FRONT) > 200) {
		// No wall on the right, so turn 90° clockwise and align
		robot->turn(RAD_90);
		robot->m(100, 100, 800);

		/*if(robot->distance(DIST_SIDE_FRONT) < 200) {
			// There is a wall right now, align
			robot->turn(RAD_90);
			robot->m(100, 100, 800);
			robot->m(-100, -100, 450);
			robot->turn(-RAD_90);
		}*/
	}

	find_black_corner();

	std::cout << "END" << std::endl;
	exit(0);
}

float Rescue::get_angle_to_right_wall() {
	float dist_front = robot->distance(DIST_SIDE_FRONT);
	float dist_back = robot->distance(DIST_SIDE_BACK);
	return std::atan((dist_back - dist_front) / 145.0f);
}

void Rescue::find_black_corner() {
	std::cout << "Searching for corner" << std::endl;
	const float DISTANCE_PER_STEP = 200.0f; // Approximate distance driven each step [mm]
	const float GOAL_DISTANCE = 60.0f;
	while(true) {
		// Align with right wall
		robot->turn(-get_angle_to_right_wall());

		// Check for corner
		cv::Mat frame = capture(CAM_FAR);
		uint32_t num_pixels = 0;
		cv::Mat black = in_range(frame, &is_black, &num_pixels);

		float pixel_fraction = (float)num_pixels / frame.cols / frame.rows;
		std::cout << num_pixels << " black pixels (" << pixel_fraction * 100.0f << " %)" << std::endl;

		if(pixel_fraction > 0.3) {
			std::cout << "Found corner. Aligning to wall" << std::endl;
			// Over 30% of the screen is black -> corner
			robot->turn(-get_angle_to_right_wall());
			std::cout << "Adjusting distance" << std::endl;

			const uint16_t CORNER_WALL_DISTANCE = 350;
			float distance_diff = (float)robot->distance(DIST_FORWARD) - CORNER_WALL_DISTANCE;
			robot->m(100, 100, distance_diff * DISTANCE_FACTOR);

			std::cout << "Aligning with corner" << std::endl;
			robot->turn(deg_to_rad(135.0f));
			robot->m(-100, -100, 900);
			robot->turn(-RAD_90);
			robot->m(100, 100, 450);
			robot->m(-100, -100, 650);
			robot->turn(RAD_180);
			robot->m(-100, -100, 400);
			return;
		}

		std::cout << "No corner, advancing forward" << std::endl;
		robot->m(100, 100, DISTANCE_PER_STEP * DISTANCE_FACTOR);

		if(robot->distance(DIST_FORWARD) < 250) {
			std::cout << "Wall" << std::endl;
			robot->turn(-get_angle_to_right_wall() - 20.0f);
			robot->m(100, 100, 1300);
			robot->m(-100, -100, 450);
			robot->turn(-RAD_90);
		}
	}
}