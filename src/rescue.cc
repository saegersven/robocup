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

	robot->m(30, 30, 1000);
	robot->stop();
	robot->beep(100, BUZZER);

	find_black_corner(); // 1)
	
	robot->m(-100, -100, 250);
	robot->turn(deg_to_rad(35.0f));

	for (int rescued_victims_cnt = 0; rescued_victims_cnt < 3; rescued_victims_cnt++) {

		float heading = robot->get_heading(); // 3)

		bool searching_victim = true;
		while (searching_victim) {
			if (find_victim()) {
				std::cout << "Turning to heading" << std::endl;
				robot->turn_to_heading(heading);
				std::cout << "Turned to heading" << std::endl;

				searching_victim = false;
			} else {
				std::cout << "looking for victim" << std::endl;
				robot->turn(deg_to_rad(-45.0f));
			}
		}

		std::cout << "Rescuing victim..." << std::endl;


		// align with black corner
		drive_to_black_corner();

		// unload victim
		robot->servo(SERVO_1, ARM_DROP, 500);
		robot->servo(SERVO_2, GRAB_OPEN, 500);
		robot->servo(SERVO_2, GRAB_CLOSED, 500);
		robot->servo(SERVO_1, ARM_UP, 500);
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
		bool no_wall = true;
		while (no_wall) {
			if (robot->single_distance(DIST_1, 10) < 35 && robot->distance_avg(DIST_1, 10, 0.2f)) {
				no_wall = false;
			} else {
				robot->m(100, 100, 50);					
			}
		}

		// check for corner	using front camera
		robot->turn(deg_to_rad(-45));
		robot->m(100, 100, 500);
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
			cap.release();
			return;
		} 
		robot->m(-100, -100, 200);
		robot->turn(deg_to_rad(-135));
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

	std::string path = "/home/pi/Desktop/";
	std::string file_name = std::to_string (micros());
	std::string full_path = path + file_name + ".png";

	cv::imwrite(full_path, roi);
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
			robot->m(-60, -60, 520);
			robot->turn(RAD_180);

			robot->servo(SERVO_2, GRAB_OPEN, 750);
			robot->servo(SERVO_1, ARM_DOWN, 750);
			robot->servo(SERVO_2, GRAB_CLOSED, 750);
			robot->servo(SERVO_1, ARM_UP, 750);
			robot->turn(RAD_180);
			robot->m(30, 30, 500);

			// Turn back
			robot->turn(-angle2);

			// Drive back
			uint32_t search_time = std::chrono::duration_cast<std::chrono::milliseconds>(
				search_end_time - search_start_time).count();
			robot->m(-15, -15, search_time);

			// Turn initial angle
			robot->turn(-angle1);

			// Now the robot is back in the position it started when
			// this method was called, hand back to the rescue() method
			return true;
		}
	}
}

void Rescue::drive_to_black_corner() {
	// aligns robot with black corner using back cam
	cv::VideoCapture cap;
	cv::Mat frame;
	cap.open("/dev/cams/back", cv::CAP_V4L2);

	if(!cap.isOpened()) {
		std::cout << "Back cam not opened" << std::endl;
	}

	delay(100);
	const float pixel_angle = deg_to_rad(65.0f) / 640;

	while(1) {
		cap.open("/dev/cams/back", cv::CAP_V4L2);
		delay(200);
		cap.grab();
		cap.retrieve(frame);
		cap.release();
		cv::flip(frame, frame, -1);

		cv::Rect rect_roi(ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT);
		cv::Mat roi = frame(rect_roi);

		cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
		cv::GaussianBlur(roi, roi, cv::Size(7, 7), 0, 0);
		cv::Mat black;
		cv::threshold(roi, black, 50, 255, 1);

		// cv::imshow("B", black);
		// cv::waitKey(200);

		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(black, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

		std::vector<cv::Point> corner_contour;

		int x = 2000;
		for(std::vector<cv::Point> c : contours) {
			double A = cv::contourArea(c);
			cv::Rect bounding_rect = cv::boundingRect(c);
			int y = bounding_rect.y + bounding_rect.height / 2;
			// std::cout << "Contour " << A << " " << y << std::endl;
			if(A < 200) continue;
			if(y < 40 || y > 220) continue;
			if(bounding_rect.width / bounding_rect.height < 2) continue;
			x = bounding_rect.x + bounding_rect.width / 2 - frame.cols / 2;

			if(A > 100000) {
				robot->m(-100, -100, 1000);
				return;
			}
		}

		if(x == 2000) {
			// Corner was not found
			robot->turn(deg_to_rad(42.0f));
		} else {
			robot->turn(pixel_angle * x);
			robot->m(-80, -80, 600);
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
