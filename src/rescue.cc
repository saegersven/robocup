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
	std::cout << "Rescue, whoooooooo" << std::endl;
	//this->back_cam_id = robot->init_camera(0, false, 640, 480, 60);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	cap.set(cv::CAP_PROP_FPS, 30);
	cap.set(cv::CAP_PROP_FORMAT, CV_8UC3);
	
	// Gets called once inside the evacuation zone

	// Determine entrance position
	// float distance_right = robot->distance(DIST_1, 10);
	// float distance_front = robot->distance(DIST_2, 10);

	uint8_t corner_pos = 0x02;
	// corner_pos |= distance_front > 90.0f ? 0x02 : 0x00;
	// corner_pos |= distance_right < 40.0f ? 0x01 : 0x00;

	// Turn around and search for first victim
	robot->turn(WALL_RIGHT(corner_pos) ? RAD_180 : -RAD_180);

	float start_heading = robot->get_heading();

	bool ignore_dead = true;
	while(1) {
		const uint32_t num_steps = 4;
		const float total_angle = RAD_90;

		// Flip sign to avoid hitting wall
		float turn_sign = WALL_RIGHT(corner_pos) ? -1.0f : 1.0f;

		if(!ignore_dead) {
			// When no living victim was found, we are turned 90°
			// Flip sign to turn the other way
			turn_sign = -turn_sign;
		}

		bool found_victim = false;
		for(int i = 0; i < num_steps; ++i) {
			// Search for living victim
			// find_victim includes search and collection of the victim
			// When it returns true, start next turn and search
			if(find_victim(ignore_dead)) {
				found_victim = true;
				break;
			}
			robot->turn(turn_sign * total_angle / num_steps);
		}
		
		if(!found_victim) {
			if(ignore_dead) {
				// There are no living victims left
				ignore_dead = false;
			} else {
				// There are no victims left, we are done
				break;
			}
		}
	}

	// Search for exit*/
}

bool Rescue::find_victim(bool ignore_dead) {
	delay(250);
	// Capture from back camera
	cap.open(0);
	cap.grab();
	cap.retrieve(frame);
	cap.release();
	cv:flip(frame, frame, -1);
	debug_frame = frame.clone();

	cv::Rect rect_roi(ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT);
	cv::Mat roi = frame(rect_roi);

	cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
	cv::GaussianBlur(roi, roi, cv::Size(7, 7), 0, 0);

	std::vector<cv::Vec3f> circles;
	cv::HoughCircles(roi, circles, cv::HOUGH_GRADIENT, 1,
		60, // minDist
		34, // param1
		40, // param2
		2,	// minRadius
		300 // maxRadius
	);

	float max_r = 0.0f;
	int x_max = 0;
	for(int i = 0; i < circles.size(); ++i) {
		cv::Vec3i c = circles[i];
		cv::circle(debug_frame, cv::Point(c[0], c[1] + ROI_Y), c[2],
			cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
		if(c[2] > max_r) {
			x_max = c[0];
			max_r = c[2];
		}
	}
	cv::imshow("Frame", debug_frame);
	cv::waitKey(1000);

	if(circles.size() == 0) return false;

	x_max -= 640 / 2;
	const float pixel_angle = deg_to_rad(65.0f) / 640;

	robot->turn(pixel_angle * x_max);
	delay(100);

	robot->m(-30, -30, 300);
	robot->turn(RAD_180);
	delay(100);

	cv::VideoCapture cap2(2);
	cap2.set(cv::CAP_PROP_FRAME_WIDTH, 160);
	cap2.set(cv::CAP_PROP_FRAME_HEIGHT, 96);
	cap2.set(cv::CAP_PROP_FPS, 30);
	cap2.set(cv::CAP_PROP_FORMAT, CV_8UC3);

	if(!cap2.isOpened()) {
		std::cout << "Not opened" << std::endl;
	}

	delay(100);
	robot->m(15, 15);
	while(1) {
		cap2.grab();
		cap2.retrieve(frame);

		cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
		cv::GaussianBlur(frame, frame, cv::Size(7, 7), 0, 0);

		std::vector<cv::Vec3f> circles;
		cv::HoughCircles(frame, circles, cv::HOUGH_GRADIENT, 1,
			60, // minDist
			34, // param1
			40, // param2
			20,	// minRadius
			100 // maxRadius
		);

		if(circles.size() > 0) {
			cap2.release();
			cv::Vec3f c = circles[0];
			robot->stop();
			
			float center_x = frame.cols / 2.0f;
			float center_y = frame.rows + 20;
			float angle = std::atan2(c[1] - center_y, c[0] - center_x) + (PI / 2.0f);
			robot->turn(angle);

			robot->m(-60, -60, 520);
			robot->turn(RAD_180);

			robot->servo(SERVO_2, GRAB_OPEN, 750);
			robot->servo(SERVO_1, ARM_DOWN, 750);
			robot->servo(SERVO_2, GRAB_CLOSED, 750);
			robot->servo(SERVO_1, ARM_UP, 750);
			robot->turn(RAD_180);
			robot->m(30, 30, 500);

			// Search for corner
			while(1) {// Capture from back camera


				cap.open(0);
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

				cv::imshow("B", black);
				cv::waitKey(200);

				std::vector<std::vector<cv::Point>> contours;
				std::vector<cv::Vec4i> hierarchy;
				cv::findContours(black, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

				std::vector<cv::Point> corner_contour;

				int x = 2000;
				for(std::vector<cv::Point> c : contours) {
					double A = cv::contourArea(c);
					cv::Rect bounding_rect = cv::boundingRect(c);
					int y = bounding_rect.y + bounding_rect.height / 2;
					std::cout << "Contour " << A << " " << y << std::endl;
					if(A < 200) continue;
					if(y < 40 || y > 220) continue;
					if(bounding_rect.width / bounding_rect.height < 2) continue;
					x = bounding_rect.x + bounding_rect.width / 2 - frame.cols / 2;

					if(A > 100000) {
						robot->m(-100, -100, 700);
						robot->servo(SERVO_1, ARM_DROP, 750);
						robot->servo(SERVO_2, GRAB_OPEN, 750);
						robot->servo(SERVO_2, GRAB_CLOSED, 750);
						robot->servo(SERVO_1, ARM_UP, 750);

						robot->m(50, 50, 300);
						return true;
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

			break;
		}

		cv::imshow("f", frame);
		cv::waitKey(1);
	}
	cap2.release();

	return true;
}