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

	victimML.init();

	robot->stop();
	robot->beep(100, BUZZER);
	robot->m(100, 100, 180);

	// Take time to align so that wall is on the right
	if(robot->distance_avg(DIST_SIDE_FRONT, 5, 0.2f) > 200) {
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

	// Drop Rescue Kit
	robot->servo(SERVO_1, ARM_DROP, 500);
	robot->servo(SERVO_2, GRAB_OPEN, 500);
	robot->servo(SERVO_2, GRAB_CLOSED, 500);
	robot->servo(SERVO_1, ARM_UP, 500);
	robot->m(100, 100, 550);
	robot->turn(RAD_180);

	std::cout << "END" << std::endl;
	exit(0);

	// Search for victims
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
			float distance_diff = (float)robot->distance_avg(DIST_FORWARD, 5, 0.2f) - CORNER_WALL_DISTANCE;
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

bool Rescue::rescue_victim(bool ignore_dead) {
	cv::Mat frame = capture(CAM_FAR);
	cv::Mat probability_map = victimML.invoke(frame);
	std::vector<Victim> victims = victimML.extract_victims(probability_map);

	const uint16_t WIDTH = 160;
	const uint16_t HEIGHT = 120;
	const float FAR_CAM_FOV = deg_to_rad(60.0f);

	bool victim_selected = false;
	Victim selected_victim;

	// Select lowest victim ignoring dead victims if ignore_dead is true
	for(int i = 0; i < victims.size(); ++i) {
		if(!ignore_dead || !victims[i].dead) {
			if(victims[i].y > selected_victim.y) {
				selected_victim = victims[i];
				victim_selected = true;
			}
		}
	}

	if(!victim_selected) return false;

	float angle = (selected_victim.x / WIDTH - 0.5) * FAR_CAM_FOV;
	robot->turn(angle);

	uint32_t num_steps = 0;
	const uint16_t APPROACH_STEP_SIZE = 300;
	while(selected_victim.y < HEIGHT * 0.7f) {
		++num_steps;
		robot->m(100, 100, APPROACH_STEP_SIZE);
		delay(200);

		frame = capture(CAM_FAR);
		probability_map = victimML.invoke(frame);
		victims = victimML.extract_victims(probability_map);

		victim_selected = false;
		selected_victim = {false, 0.0f, 0.0f};

		// Select victim that is closest to the center
		for(int i = 0; i < victims.size(); ++i) {
			if(abs(victims[i].x - WIDTH / 2) < abs(selected_victim.x - WIDTH / 2)) {
				selected_victim = victims[i];
				victim_selected = true;
			}
		}

		float angle2 = (selected_victim.x / WIDTH - 0.5) * FAR_CAM_FOV;
		robot->turn(angle2);
	}

	// Pick victim up
	robot->turn(RAD_180);
	robot->servo(SERVO_2, GRAB_OPEN, 750);
	robot->servo(SERVO_1, ARM_ALMOST_DOWN, 650);
	robot->m(-50, -50, 340);
	robot->servo(SERVO_1, ARM_DOWN, 250);
	robot->servo(SERVO_2, GRAB_CLOSED, 750);
	robot->m(50, 50, 100);
	robot->servo(SERVO_1, ARM_UP, 750);

	robot->m(100, 100, num_steps * APPROACH_STEP_SIZE);
	robot->turn(-angle - RAD_180);
	robot->m(-100, -100, 800);

	robot->servo(SERVO_1, ARM_DROP, 500);
	robot->servo(SERVO_2, GRAB_OPEN, 500);
	robot->servo(SERVO_2, GRAB_CLOSED, 500);
	robot->servo(SERVO_1, ARM_UP, 500);

	robot->m(100, 100, 550);
	robot->turn(RAD_180);

	return true;
}