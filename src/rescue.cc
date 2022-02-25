#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <pthread.h>

#include "robot.h"
#include "rescue.h"
#include "utils.h"

Rescue::Rescue(std::shared_ptr<Robot> robot) {
	this->robot = robot;
}

void Rescue::start() {
	// std::thread rescue_thread([this]() { this->rescue(); });
	// this->native_handle = rescue_thread.native_handle();
	// rescue_thread.detach();
	rescue();
}

void Rescue::stop() {
	pthread_cancel(this->native_handle);
}

void Rescue::rescue() {

	/*
	############### ultimate plan for rescue area: ###############

	1) find black corner, turn robots back to corner (-> find_black_corner())
	2) drive ~15cm forward and align using back cam
	3) save measurement from gyroscope
	4) search for victims, pick one up
	5) drive back to "homepos" and turn to saved value
	6) align using back cam
	7) drive backwards and rescue the victim

	jump to 2)

	*/
	robot->m(30, 30, 1000);
	robot->stop();
	robot->beep(100, BUZZER);

	find_black_corner(); // 1)
}

// see 1)
void Rescue::find_black_corner() {

	// Wallfollower:
	/*
	float target_dist = 6.00; // target distance for wallfollowing (in cm) 
	float multiplier = 7.00;
	while (robot->single_distance(DIST_1, 20) > 35) {
		float error = (robot->single_distance(DIST_2, 20) - target_dist) * multiplier;
		robot->m((-error)+30, (error)+30, 10);
		std::cout << error << std::endl;
	}
	robot->stop();
	robot->beep(1000);
	*/

	while (1) {
		while (robot->single_distance(DIST_1, 10) > 35) robot->m(100, 100, 50);

		// check for corner	using front camera
		robot->turn(deg_to_rad(-45));
		robot->m(100, 100, 600);
		robot->turn(deg_to_rad(90));
		robot->m(50, 50, 700);

		cv::VideoCapture cap("/dev/cams/front");
		cap.set(cv::CAP_PROP_FRAME_WIDTH, 160);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, 96);
		cap.set(cv::CAP_PROP_FPS, 30);
		cap.set(cv::CAP_PROP_FORMAT, CV_8UC3);

		if(!cap.isOpened()) {
			std::cout << "Not opened" << std::endl;
		}
		cap.grab();
		cap.retrieve(frame);
		cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
		cv::GaussianBlur(frame, frame, cv::Size(7, 7), 0, 0);
		cv::Mat black;
		cv::threshold(frame, black, 60, 255, 1);

		cv::imshow("B", black);
		cv::waitKey(200);
		if (cv::countNonZero(black) > 1500) {
			std::cout << "Found corner" << std::endl;
			robot->beep(400);
			robot->m(-100, -100, 1500);
			robot->turn(deg_to_rad(180));
			return;
		} 
		robot->m(-100, -100, 200);
		robot->turn(deg_to_rad(-135));
		robot->m(-100, -100, 1500);
	}
}