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
	2) drive forward and align using back cam
	3) save measurement from gyroscope
	4) search for victims, pick one up
	5) drive back to "homepos" and turn to saved value
	6) align using back cam
	7) drive backwards and rescue the victim

	jump to 2)

	*/

	robot->stop();
	robot->beep(100, BUZZER);

	find_black_corner(); // 1)
	
	robot->m(-100, -100, 400);
	float heading = robot->get_heading(); // 3)
	robot->turn(deg_to_rad(60));

	for (int rescued_victims_cnt = 0; rescued_victims_cnt < 3; rescued_victims_cnt++) {
		bool searching_victim = true;
		while (searching_victim) {
			if (find_victim()) {
				searching_victim = false;
			} else {
				std::cout << "looking for victim" << std::endl;
				robot->turn(deg_to_rad(-35));
			}
		}

		std::cout << "Rescuing victim..." << std::endl;

		// align with black corner
		robot->turn_to_heading(heading);
		robot->m(-100, -100, 600);
		robot->turn(deg_to_rad(180));
		robot->m(-100, -100, 1200);

		// unload victim
		robot->servo(SERVO_1, ARM_DROP, 500);
		robot->servo(SERVO_2, GRAB_OPEN, 500);
		robot->servo(SERVO_2, GRAB_CLOSED, 500);
		robot->servo(SERVO_1, ARM_UP, 500);

		robot->m(-100, -100, 250);
		robot->m(100, 100, 250);
		robot->turn(deg_to_rad(-135));
	}

	find_exit();
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
		while (1) {
			if (robot->single_distance(DIST_1) < 35 && robot->distance_avg(DIST_1, 10, 0.2f) < 35) break;
			robot->m(100, 100);	
		}

		// check for corner	using front camera
		robot->turn(deg_to_rad(-45));
		robot->m(100, 100, 400);
		robot->turn(deg_to_rad(90));
		robot->m(50, 50, 650);

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
			cap.release();
			return;
		} 
		robot->turn(deg_to_rad(45));
		robot->m(-100, -100, 300);
		robot->turn(deg_to_rad(-180));
		robot->m(-100, -100, 1500);

	}
}

bool Rescue::get_largest_circle(cv::Mat roi, cv::Vec3f& out) {
	cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
	cv::GaussianBlur(roi, roi, cv::Size(7, 7), 0, 0);
	std::vector<cv::Vec3f> circles;
	cv::HoughCircles(roi, circles, cv::HOUGH_GRADIENT, 1,
		60, // minDist
		34, // param1
		40, // param2
		2,  // minRadius
		300 // maxRadius
	);

	std::cout << circles.size() << std::endl;
	if(circles.size() == 0) return false; // No circles

	if(circles.size() == 1) {
		out = circles[0];
		return true;
	}

	// Select largest circle (maximum radius)
	float max_r = 0.0f;
	int max_index = 0;
	for(int i = 0; i < circles.size(); ++i) {
		cv::Vec3f c = circles[i];

		if(c[2] > max_r) {
			out = c;
			max_r = c[2];
		}
	}
	return true;
}

bool Rescue::find_victim() {
	// Capture one frame from camera
	cv::VideoCapture cap;

	cap.open("/dev/cams/back", cv::CAP_V4L2);
	if(!cap.isOpened()) {
		std::cout << "Back cam not opened" << std::endl;
	}
	cap.grab();
	cap.retrieve(frame);
	cap.release();

	cv::flip(frame, frame, -1);

	// Cut out horizontal region of interest
	cv::Rect rect_roi(ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT);
	cv::Mat roi = frame(rect_roi);

	cv::Vec3f victim;
	if(!get_largest_circle(roi, victim)) return false;
	int victim_x = victim[0] - 640 / 2;	

	// Turn to victim based on horizontal pixel coordinate
	const float pixel_angle = deg_to_rad(65.0f) / 640;
	float angle1 = pixel_angle * victim_x;
	robot->turn(angle1);
	delay(100);

	// Turn around and search with front camera
	robot->m(-30, -30, 300);
	robot->turn(RAD_180);
	delay(100);

	cv::VideoCapture cap2("/dev/cams/front", cv::CAP_V4L2);
	cap2.set(cv::CAP_PROP_FRAME_WIDTH, 160);
	cap2.set(cv::CAP_PROP_FRAME_HEIGHT, 96);
	cap2.set(cv::CAP_PROP_FPS, 30);
	cap2.set(cv::CAP_PROP_FORMAT, CV_8UC3);

	if(!cap2.isOpened()) {
		std::cout << "Front cam not opened" << std::endl;
	}

	delay(100);
	robot->m(15, 15);
	auto search_start_time = std::chrono::high_resolution_clock::now();
	while(1) {
		cap2.grab();
		cap2.retrieve(frame);

		cv::Vec3f c;
		if(get_largest_circle(frame, c)) {
			cap2.release();
			robot->stop();
			auto search_end_time = std::chrono::high_resolution_clock::now();

			// Turn to victim
			const float center_x = frame.cols / 2.0f;
			const float center_y = frame.rows + 20;
			float angle2 = std::atan2(c[1] - center_y, c[0] - center_x) +
				(PI / 2.0f);
			robot->turn(angle2);

			// Turn around, pick up and turn back
			robot->m(-60, -60, 600);
			delay(100);
			robot->turn(RAD_180);

			robot->servo(SERVO_2, GRAB_OPEN, 750);
			robot->servo(SERVO_1, ARM_DOWN, 750);
			robot->servo(SERVO_2, GRAB_CLOSED, 750);
			robot->servo(SERVO_1, ARM_UP, 750);
			robot->turn(RAD_180);
			delay(100);
			robot->m(30, 30, 500);
			delay(100);

			// Turn back
			robot->turn(-angle2);

			// Drive back
			uint32_t search_time = std::chrono::duration_cast<std::chrono::milliseconds>(
				search_end_time - search_start_time).count();
			robot->m(-15, search_time);

			// Turn initial angle
			robot->turn(-angle1);

			// Now the robot is back in the position it started when
			// this method was called, hand back to the rescue() method
			return true;
		}
	}
}

void Rescue::find_exit() {
	// finds exit of evacuation zone

	robot->m(100, 100, 250);
	robot->turn(deg_to_rad(90));
	while(robot->single_distance(DIST_1) > 10 && robot->distance_avg(DIST_1, 10, 0.2f) > 10) {
		robot->m(100, 100);
	}
	robot->turn(deg_to_rad(-45));
	while(robot->single_distance(DIST_1) > 7 && robot->distance_avg(DIST_1, 10, 0.2f) > 7) {
		robot->m(100, 100);
	}
	robot->turn(deg_to_rad(90));
	robot->beep(100);
	robot->m(100, 100, 100);	
	exit(0);

	// start line thread:
}
