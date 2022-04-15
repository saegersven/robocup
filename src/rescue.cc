#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <pthread.h>

#include "line.h"
#include "robot.h"
#include "rescue.h"
#include "utils.h"

Rescue::Rescue(std::shared_ptr<Robot> robot) : finished(false) {
	this->robot = robot;
}

void Rescue::start() {
	std::thread rescue_thread([this]() { this->rescue(); });
	this->native_handle = rescue_thread.native_handle();
	rescue_thread.detach();
	//rescue();
}

void Rescue::stop() {
	pthread_cancel(this->native_handle);
	//finished = true;
	cap.release();
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
	robot->m(100, 100, 100);
	find_black_corner(); // 1)
	
	robot->m(-100, -100, 600);
	//float heading = robot->get_heading(); // 3)
	robot->turn(deg_to_rad(60));
	uint8_t turn_counter = 0;

	for (int rescued_victims_cnt = 0; rescued_victims_cnt < 3; rescued_victims_cnt++) {
		bool searching_victim = true;
		while (searching_victim) {
			if (find_victim()) {
				searching_victim = false;
			} else {
				std::cout << "looking for victim" << std::endl;
				robot->turn(deg_to_rad(-30));
				delay(100);
				++turn_counter;
			}
		}

		std::cout << "Rescuing victim..." << std::endl;

		// align with black corner
		//robot->turn_to_heading(heading);
		robot->turn(turn_counter * deg_to_rad(30) - deg_to_rad(60));

		robot->m(100, 100, 1000);
		robot->m(-100, -100, 500);
		robot->turn(RAD_180);
		robot->m(-100, -100, 1400);

		// unload victim
		robot->servo(SERVO_1, ARM_DROP, 500);
		robot->servo(SERVO_2, GRAB_OPEN, 500);
		robot->servo(SERVO_2, GRAB_CLOSED, 500);
		robot->servo(SERVO_1, ARM_UP, 500);

		robot->m(100, 100, 130);
		robot->turn(deg_to_rad(-120));
		turn_counter = 0;
	}

	find_exit();
	finished = true;
}

// see 1)
void Rescue::find_black_corner() {
	// check if there's a wall next to the robot (right side):
	if (robot->distance_avg(DIST_2, 10, 0.2f) > 10.0f) {
		// if not drive so that there is one:
		robot->turn(-RAD_90);
		robot->m(-100, -100, 400);
		robot->turn(-RAD_180);
		robot->m(-100, -100, 800);
	}
	while (1) { 
		bool isWall = false; // is robot < 35cm away from front wall
		bool isGreen = false; // is there the exit?

		// repeat until black corner is found:
		while (!isWall && !isGreen) {
			float dist = robot->single_distance(DIST_1);
			if (dist < 35.0f && robot->distance_avg(DIST_1, 10, 0.2f) < 35.0f) {
				isWall = true; 
				break;
			} else {
				// unlikely, but there could be no front wall due to exit ahead				
				cv::VideoCapture cap;

				cap.set(cv::CAP_PROP_FRAME_WIDTH, 160);
				cap.set(cv::CAP_PROP_FRAME_HEIGHT, 96);
				cap.set(cv::CAP_PROP_FPS, 30);
				cap.set(cv::CAP_PROP_FORMAT, CV_8UC3);

				cap.open("/dev/cams/front", cv::CAP_V4L2);
				if(!cap.isOpened()) {
					std::cout << "Front cam not opened" << std::endl;
				}
				cap.grab();
				cap.retrieve(frame);
				cap.release();
				cv::Mat roi = frame(cv::Range(24, 43), cv::Range(15, 67));

				uint32_t num_green_pixels = 0;
				in_range(roi, &is_green, &num_green_pixels);

				if(num_green_pixels > 500) {
					robot->beep(1000);
					isGreen = true;
					break;
				}
			}
			if (dist < 120) {
				robot->m(100, 100, (pow((dist - 33.0f), 1.6f) + 10.0f)); // checking intervals increase non linear depending on distance to front wall
			} else { // there must be the exit since there is no wall. Robot must drive slowly to be able to detect green
				robot->m(100, 100, 25);
			}
			
		}

		// check for corner	using front camera
		if (isWall) {
			robot->turn(-RAD_45);
			robot->m(100, 100, 400);
			robot->turn(RAD_90);
			robot->m(50, 50, 650);

			cv::VideoCapture cap;
			cap.set(cv::CAP_PROP_FRAME_WIDTH, 160);
			cap.set(cv::CAP_PROP_FRAME_HEIGHT, 96);
			cap.set(cv::CAP_PROP_FPS, 30);
			cap.set(cv::CAP_PROP_FORMAT, CV_8UC3);

			cap.open("/dev/cams/front", cv::CAP_V4L2);
			if(!cap.isOpened()) {
				std::cout << "Front cam not opened" << std::endl;
			}
			cap.grab();
			cap.retrieve(frame);
			cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
			cv::GaussianBlur(frame, frame, cv::Size(7, 7), 0, 0);
			cv::Mat black;
			cv::threshold(frame, black, 60, 255, 1);
			cv::imshow("B", black);
			cv::waitKey(1000);
			std::cout << "black Pixels: " << cv::countNonZero(black) << std::endl;
			if (cv::countNonZero(black) > 15000) {
				std::cout << "Found corner" << std::endl;
				robot->beep(400);
				cap.release();
				save_img("/home/pi/Desktop/black_corner_images/", frame);
				return;
			} 
			robot->turn(RAD_45);
			robot->m(-100, -100, 300);
			robot->turn(deg_to_rad(-185));
			robot->m(-100, -100, 1500);
		} 
		// no need to check for corner since there is the exit
		else if (isGreen) {
			robot->stop();
			robot->beep(3000);
			delay(100000);
		}
	}
	/*
	auto start = micros();
	float min_dist = 99999999.9f;
	while (micros() - start < 2500000) {
		robot->m(100, 100, 100);
		float dist = robot->distance_avg(DIST_2, 10, 0.2f);

		// save min dist
		if (dist < min_dist) min_dist = dist;
	}

	float error = min_dist - 60.0f;
	if(error < 0) {
		robot->turn(-RAD_90);
	} else {
		robot->turn(RAD_90);
	}
	robot->m(100, 100, error * 50);
	*/

	/*
	// drive ~ 52.5cm forward (approximately half of the rescue area no matter where the entry is)
	robot->beep(3000);
	robot->m(100, 100, 1600);

	// now turn a bit left/right (depending on where is a sidewall)

	if (robot->distance_avg(DIST_2, 10, 0.2f) > 40.0f) {
		robot->turn(-RAD_90);
		robot->m(-100, -100, 700);
	} else {		
		robot->turn(RAD_90);
		robot->m(-100, -100, 700);
	}

	robot->stop();
	cap.release();
	std::cout << "In middle of rescue area" << std::endl;

	cap.open("/dev/cams/back", cv::CAP_V4L2);
	if(!cap.isOpened()) {
		std::cout << "Back cam not opened" << std::endl;
	}

	uint32_t max_black_pixels = 0;
	float max_heading = 0.0f;

	for (int i = 0; i < 20; ++i) {
		robot->turn(deg_to_rad(18.0f));
		delay(500);

		cv::Mat image;
		cap.grab();
		cap.retrieve(image);

		cv::imshow("Frame", image);
		
		// just look at bottom half of image
		cv::Mat roi = image(cv::Range(24, 43), cv::Range(15, 67));
		// count black pixels, if > max_black_pixels -> save heading and update max_black_pixels
		

	}
	cap.release();

	robot->turn_to_heading(max_heading - RAD_180);
	*/
	
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
	cap.release();
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
	save_img("/home/pi/Desktop/victims_images_back_cam/", frame);


	// Turn around and search with front camera
	robot->m(-30, -30, 300);
	robot->turn(RAD_180);
	delay(100);

	cap.open("/dev/cams/front", cv::CAP_V4L2);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 160);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 96);
	cap.set(cv::CAP_PROP_FPS, 30);
	cap.set(cv::CAP_PROP_FORMAT, CV_8UC3);

	if(!cap.isOpened()) {
		std::cout << "Front cam not opened" << std::endl;
	}

	delay(100);
	robot->m(35, 35);
	auto search_start_time = std::chrono::high_resolution_clock::now();
	while(1) {
		cap.grab();
		cap.retrieve(frame);

		cv::Vec3f c;
		if(get_largest_circle(frame, c)) {
			save_img("/home/pi/Desktop/victims_images_front_cam/", frame);
			cap.release();
			robot->stop();
			auto search_end_time = std::chrono::high_resolution_clock::now();

			// Turn to victim
			const float center_x = frame.cols / 2.0f;
			const float center_y = frame.rows + 20;
			float angle2 = std::atan2(c[1] - center_y, c[0] - center_x) +
				(PI / 2.0f);
			robot->turn(angle2);

			// Turn around, pick up and turn back
			robot->m(-60, -60, 650);
			delay(100);
			robot->turn(RAD_180);

			robot->servo(SERVO_2, GRAB_OPEN, 750);
			robot->servo(SERVO_1, ARM_DOWN, 670);
			robot->servo(SERVO_2, GRAB_CLOSED, 750);
			robot->servo(SERVO_1, ARM_UP, 750);
			//robot->turn(RAD_180);
			delay(100);
			robot->m(-60, -60, 600);
			delay(100);

			// Turn back
			robot->turn(-angle2);

			// Drive back
			uint32_t search_time = std::chrono::duration_cast<std::chrono::milliseconds>(
				search_end_time - search_start_time).count();

			std::cout << search_time << std::endl;

			robot->m(35, 35, search_time);

			// Turn initial angle
			robot->turn(-angle1);
			delay(150);

			// Now the robot is back in the position it started when
			// this method was called, hand back to the rescue() method
			return true;
		}
	}
}

void Rescue::find_exit() {
	robot->m(-100, -100, 650);
	robot->turn(-RAD_90);
	robot->m(100, 100, 1400);
	robot->m(100, 100, -300);
	robot->turn(-RAD_90);
	robot->beep(100);

	// Drive while there is a side wall
	while(robot->single_distance(DIST_2) < 30.0f || robot->distance_avg(DIST_2, 10, 0.2f) < 30.0f) {
		if(robot->single_distance(DIST_1) < 10.0f && robot->distance_avg(DIST_1, 10, 0.2f) < 10.0f) {
			robot->turn(-RAD_45);
			robot->m(100, 100, 500);
			robot->turn(-RAD_45);
			robot->m(-100, -100, 500);
		}
		robot->m(100, 100);
	}
	robot->m(100, 100, 150);

	std::cout << "Check for green" << std::endl;

	// Turn right and check for green strip
	robot->stop();

	robot->turn(RAD_90);
	robot->m(100, 100, 350);

	cv::VideoCapture cap;

	cap.set(cv::CAP_PROP_FRAME_WIDTH, 160);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 96);
	cap.set(cv::CAP_PROP_FPS, 30);
	cap.set(cv::CAP_PROP_FORMAT, CV_8UC3);

	cap.open("/dev/cams/front", cv::CAP_V4L2);
	if(!cap.isOpened()) {
		std::cout << "Front cam not opened" << std::endl;
	}
	cap.grab();
	cap.retrieve(frame);
	cap.release();

	uint32_t num_green_pixels = 0;
	in_range(frame, &is_green, &num_green_pixels);

	if(num_green_pixels > 1500) {
		// Found exit
		robot->m(100, 100, 500);
		std::cout << "Found exit" << std::endl;
		save_img("/home/pi/Desktop/exit_images/", frame);
		return;
	}
}
