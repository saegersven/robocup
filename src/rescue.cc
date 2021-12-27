#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "robot.h"
#include "rescue.h"
#include "utils.h"

Rescue::Rescue(Robot* robot) {
	this->robot = robot;
}

void Rescue::rescue() {
	this->back_cam_id = robot->init_camera(1, false, 640, 480, 60);

	while(1) {
		find_victim(false);
	}
	/*
	// Gets called once inside the evacuation zone

	// Determine entrance position
	float distance_right = robot->distance(DIST_1, 10);
	float distance_front = robot->distance(DIST_2, 10);

	uint8_t corner_pos = 0x00;
	corner_pos |= distance_front > 90.0f ? 0x02 : 0x00;
	corner_pos |= distance_right < 40.0f ? 0x01 : 0x00;

	// Turn around and search for first victim
	robot->turn(RIGHT_WALL(corner_pos) ? RAD_180 : -RAD_180);

	float start_heading = robot->get_heading();

	bool ignore_dead = true;
	while(1) {
		const uint32_t num_steps = 4;
		const float total_angle = RAD_90;

		// Flip sign to avoid hitting wall
		float turn_sign = RIGHT_WALL(corner_pos) ? -1.0f : 1.0f;

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
	// Capture from back camera
	cv::Mat frame = robot->capture(back_cam_id);
	cv::Mat debug_frame = frame.clone();

	cv::Rect rect_roi(ROI_MIN_X, ROI_MIN_Y, ROI_MAX_X, ROI_MAX_Y);
	cv::Mat roi = frame(rect_roi);

	cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
	cv::GaussianBlur(roi, roi, cv::Size(7, 7));

	std::vector<cv::Vec3f> circles;
	cv::HoughCircles(roi, circles, cv::HOUGH_GRADIENT, 1.2,
		50, // minDist
		23, // param1
		55, // param2
		1,	// minRadius
		300 // maxRadius
	);

	for(int i = 0; i < circles.size(); ++i) {
		cv::Vec3i c = circles[i];
		cv::circle(debug_frame, cv::Point(c[0], c[1]), c[2],
			cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
	}

	cv::imshow("find_victim Debug", debug_frame);
	cv::waitKey(1);

	return circles.size() > 0;
}