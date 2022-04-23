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
	robot->m(100, 100, 180);
	find_black_corner(); // 1)

	// drop rescue kit:		
	robot->m(-100, -100, 600);
	robot->turn(RAD_180);
	robot->m(-100, -100, 800);
	robot->servo(SERVO_1, ARM_DROP, 500);
	robot->servo(SERVO_2, GRAB_OPEN, 500);
	robot->servo(SERVO_2, GRAB_CLOSED, 500);
	robot->servo(SERVO_1, ARM_UP, 500);
	robot->m(100, 100, 550);
	robot->turn(RAD_180);

	//float heading = robot->get_heading(); // 3)
	uint8_t turn_counter = 0;

	const int num_victims = 3;

	for (int rescued_victims_cnt = 0; rescued_victims_cnt < num_victims; rescued_victims_cnt++) {
		bool searching_victim = true;
		bool is_in_center = false;
		bool abort = false;

		while (searching_victim) {
			if(turn_counter == 12) {
				if(is_in_center) {
					robot->m(100, 100, 1200);
					abort = true;
					break;
				} else {
					robot->m(-100, -100, 1200);
					is_in_center = true;
					//robot->turn(-RAD_90);
					robot->beep(100);
					turn_counter = 0;
				}
			}

			if (find_victim(rescued_victims_cnt < 2)) {
				searching_victim = false;
			} else {
				std::cout << "looking for victim" << std::endl;
				robot->turn(deg_to_rad(-30));
				delay(50);
				++turn_counter;
			}
		}
		if(abort) break;

		std::cout << "Rescuing victim..." << std::endl;

		// align with black corner
		//robot->turn_to_heading(heading);
		robot->turn(turn_counter * deg_to_rad(30) - RAD_180 + deg_to_rad(4.2f));

		robot->m(100, 100, 1000);
		robot->m(-100, -100, 500);
		robot->turn(RAD_180);
		robot->m(-100, -100, is_in_center ? 2600 : 1500);

		// unload victim
		robot->servo(SERVO_1, ARM_DROP, 500);
		robot->servo(SERVO_2, GRAB_OPEN, 500);
		robot->servo(SERVO_2, GRAB_CLOSED, 500);
		robot->servo(SERVO_1, ARM_UP, 500);

		robot->m(100, 100, 450);

		is_in_center = false;

		if(rescued_victims_cnt == num_victims - 1) {
			robot->turn(-RAD_180);
		} else {
			robot->turn(-RAD_180);
			turn_counter = 0;
		}
	}

	find_exit();
	finished = true;
}

bool check_green_stripe(cv::Mat frame) {
	uint32_t num_green_pixels = 0;
	in_range(frame, &is_green, &num_green_pixels);

	/*if(num_green_pixels > 1500) {
		// Found exit
		robot->m(100, 100, 500);
		std::cout << "Found exit" << std::endl;
		save_img("/home/pi/Desktop/exit_images/", frame);
		return;
	}*/
	//std::cout << num_green_pixels << std::endl;
	return num_green_pixels > 300;
}

// see 1)
void Rescue::find_black_corner() {
	// check if there's a wall next to the robot (right side):
	if (robot->distance_avg(DIST_2, 10, 0.2f) > 20.0f) {
		// if not drive so that there is one:
		robot->turn(-RAD_90);
		robot->m(-100, -100, 350);
		robot->turn(-RAD_180);
		robot->m(-100, -100, 650);
	} else {
		robot->turn(RAD_90);
		robot->m(50, 50, 900);
		robot->m(-50, -50, 480);
		robot->turn(-RAD_90);
	}


	cv::VideoCapture cap;
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 160);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 96);
	cap.set(cv::CAP_PROP_FPS, 30);
	cap.set(cv::CAP_PROP_FORMAT, CV_8UC3);

	cap.open("/dev/cams/front", cv::CAP_V4L2);
	if(!cap.isOpened()) {
		std::cout << "Front cam not opened" << std::endl;
	}

	while (1) { 
		bool isWall = false; // is robot < 35cm away from front wall
		bool isGreen = false; // is there the exit?

		// repeat until black corner is found:
		while (!isWall && !isGreen) {

			cap.grab();
			cap.retrieve(frame);

			/*if(check_green_stripe(frame)) {
				std::cout << "Green stripe" << std::endl;
				isGreen = true;
				break;
			}*/

			float dist = robot->single_distance(DIST_1);
			if (dist < 33.0f && robot->distance_avg(DIST_1, 10, 0.2f) < 33.0f) {
				std::cout << "Wall" << std::endl;
				isWall = true; 
				break;
			}
			if (dist < 120.0f) {
				robot->m(100, 100);//, (pow((dist - 34.0f), 1.7f) + 20.0f)); // checking intervals increase non linear depending on distance to front wall
			}
			// else { // in case of wrong measurement:
			//	robot->m(100, 100, 25);
			// }
			
		}

		// check for corner	using front camera
		if (isWall) {
			robot->turn(-RAD_45);
			robot->m(100, 100, 400);
			robot->turn(RAD_90);
			robot->m(50, 50, 650);

			cap.release();

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
			//cv::imshow("B", black);
			//cv::waitKey(1000);
			std::cout << "Black Pixels: " << cv::countNonZero(black) << std::endl;
			if (cv::countNonZero(black) > 20000) {
				std::cout << "Found corner" << std::endl;
				robot->beep(400);
				cap.release();
				save_img("/home/pi/Desktop/black_corner_images/", frame);
				return;
			} 
			robot->turn(RAD_45);
			robot->m(-100, -100, 200);
			robot->turn(-RAD_90);
			robot->m(100, 100, 800);
			robot->m(-100, -100, 480);
			robot->turn(-RAD_90);
			//robot->m(-100, -100, 1100);
		} 
		// no need to check for corner since there is the exit
		else if (isGreen) {
			cap.release();
			robot->m(-100, -100, 200);
			robot->turn(RAD_90);
			robot->m(-100, -100, 500);
			robot->turn(-RAD_180);
			robot->m(-100, -100, 1000);
			cap.open("/dev/cams/front", cv::CAP_V4L2);
		}
	}
	cap.release();
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

bool Rescue::get_largest_circle(cv::Mat roi, cv::Vec3f& out, bool ignore_dead) {
	cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
	cv::GaussianBlur(roi, roi, cv::Size(3, 3), 0, 0);
	std::vector<cv::Vec3f> circles;

	// for finding perfect params refer to /scripts/ml/find_perfect_houghCircles_params.py
	/*cv::HoughCircles(roi, circles, cv::HOUGH_GRADIENT, 0.8f,
		60, // minDist
		37, // param1
		30, // param2
		10,  // minRadius
		300 // maxRadius
	);*/

	cv::HoughCircles(roi, circles, cv::HOUGH_GRADIENT, 1,
		60, // minDist
		34, // param1
		40, // param2
		10,  // minRadius
		300 // maxRadius
	);

#ifdef DEBUG
	cv::Mat debug_roi = roi.clone();
	for(int i = 0; i < circles.size(); ++i) {
		cv::circle(debug_roi, cv::Point2f(circles[i][0], circles[i][1]), circles[i][2], cv::Scalar(0, 0, 0), 4);
	}
	//cv::imshow("d", debug_roi);
	//cv::waitKey(500);
#endif

	//std::cout << circles.size() << std::endl;
	if(circles.size() == 0) return false; // No circles
	if(circles.size() > 3) {
		std::cout << "More than 3" << std::endl;
		return false;
	}
	/*if(circles.size() == 1) {
		out = circles[0];
#ifdef DEBUG
	save_img("/home/pi/Desktop/runtime_debug/", debug_roi);
#endif
		return true;
	}*/

	// Select largest circle (maximum radius)
	float max_r = 0.0f;
	int max_index = 0;
	for(int i = 0; i < circles.size(); ++i) {
		cv::Vec3f c = circles[i];

		float average_victim_color = average_circle_color(roi, c[0], c[1], c[2]);
		bool black = average_victim_color < 45.0f;

		std::cout << average_victim_color << std::endl;

		if(ignore_dead && black) {
			std::cout << "Dead victim" << std::endl;
			if(circles.size() == 1) return false;
			else continue;
		}

		if(c[2] > max_r) {
			out = c;
			max_r = c[2];
		}
	}
#ifdef DEBUG
		save_img("/home/pi/Desktop/runtime_debug/", debug_roi);
#endif
	return true;
}

bool Rescue::find_victim(bool ignore_dead) {
	// Capture one frame from camera
	cap.release();
	delay(100);
	cap.open("/dev/cams/back", cv::CAP_V4L2);
	if(!cap.isOpened()) {
		std::cout << "Back cam not opened" << std::endl;
	}
	cap.grab();
	cap.retrieve(frame);
	delay(100);
	cap.release();

	// Cut out horizontal region of interest
	cv::Rect rect_roi(ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT);
	cv::Mat roi = frame(rect_roi);

	cv::Vec3f victim;
	if(!get_largest_circle(roi.clone(), victim, ignore_dead)) return false;

	/*float area = victim[2] * victim[2] * PI;
	uint32_t num_circle_pixels = count_circle_in_range(roi, victim[0], victim[1], victim[2], &is_black);
	float black_fraction = (float)num_circle_pixels / area;

	bool black = black_fraction > 0.2f;
	std::cout << victim[0] << "\t" << victim[1] << "\t" << victim[2] << std::endl;
	cv::imshow("CIRCLE_ROI", roi);
	cv::waitKey(500);
	std::cout << num_circle_pixels << "\t" << black_fraction << std::endl;*/

/*
	// Check if background around circle is very white
	uint16_t roi_circle_y = victim[1] >= 40 ? victim[1]-40 : 0;
	uint16_t roi_circle_x = victim[0] >= 40 ? victim[0]-40 : 0;
	uint16_t roi_circle_height = roi_circle_y + 80 >= ROI_HEIGHT ? ROI_HEIGHT - roi_circle_y - 1 : 80;
	uint16_t roi_circle_width = roi_circle_x + 80 >= ROI_WIDTH ? ROI_WIDTH - roi_circle_y - 1 : 80;
	std::cout << roi_circle_y << "\n" << roi_circle_x << "\n" << roi_circle_height << "\n" << roi_circle_width << "\n";
	cv::Rect rect_roi_circle(roi_circle_x, roi_circle_y, roi_circle_width, roi_circle_height);
	std::cout << roi.cols << "\n" << roi.rows << "\n";
	cv::Mat roi_circle = roi(rect_roi_circle);
	cv::Vec3b avg_background = average_color(roi_circle);
	uint16_t background_color_gray = avg_background[0] + avg_background[1] + avg_background[2];
	if((black && background_color_gray < 150*3) || (!black && background_color_gray < 180*3)) {
		std::cout << "Ignoring because of dark background (" << background_color_gray << ")" << std::endl;
		return false;
	}*/

	int victim_x = victim[0] - 640 / 2;	

	// Turn to victim based on horizontal pixel coordinate
	const float pixel_angle = deg_to_rad(65.0f) / 640;
	float angle1 = pixel_angle * victim_x;
	robot->turn(angle1);

	float dist = robot->distance_avg(DIST_1, 20, 0.4f);
	if(dist > 100.0f) {
		std::cout << "Hopefully just silver" << std::endl;
		robot->turn(-angle1);
		return false;
	}

	delay(100);
	save_img("/home/pi/Desktop/victims_images_back_cam/", frame);


	// Turn around and search with frofnt camera
	//robot->m(-30, -30, 300);
	//robot->turn(RAD_180);
	//delay(100);

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
		if(get_largest_circle(frame, c, ignore_dead)) {
			save_img("/home/pi/Desktop/victims_images_front_cam/", frame);
			cap.release();
			robot->stop();
			auto search_end_time = std::chrono::high_resolution_clock::now();

			// Turn to victim
			const float center_x = frame.cols / 2.0f;
			const float center_y = frame.rows + 20;
			float angle2 = std::atan2(c[1] - center_y, c[0] - center_x) + (PI / 2.0f);
			robot->turn(angle2);

			//bool near_wall = robot->distance_avg(DIST_1, 0.2f, 10) < 15.0f;
			
			//robot->m(-60, -60, 400);
			

			// Turn around, pick up and turn back
			robot->m(-50, -50, 1000);
			delay(100);
			robot->turn(RAD_180);

			robot->servo(SERVO_2, GRAB_OPEN, 750);
			robot->servo(SERVO_1, ARM_ALMOST_DOWN, 650);
			robot->m(-50, -50, 340);
			robot->servo(SERVO_1, ARM_DOWN, 250);
			robot->servo(SERVO_2, GRAB_CLOSED, 750);
			robot->m(50, 50, 100);
			robot->servo(SERVO_1, ARM_UP, 750);

			//robot->m(-50, -50, 630);

			// Turn back
			robot->turn(-angle2);

			// Drive back
			uint32_t search_time = std::chrono::duration_cast<std::chrono::milliseconds>(
				search_end_time - search_start_time).count() - 900;

			//std::cout << search_time << std::endl;

			robot->m(35, 35, search_time);

			// Turn initial angle
			robot->turn(-angle1);
			delay(150);
			return true;
		}
	}
}


void Rescue::find_exit() {
	robot->m(100, 100, 1500);
	robot->m(-100, -100, 450);
	robot->turn(RAD_45);
	robot->m(-100, -100, 550);
	robot->turn(-RAD_90);
	robot->m(100, 100, 1500);
	robot->m(-100, -100, 320);
	robot->turn(-RAD_90);
	robot->beep(100);

	cv::VideoCapture cap;

	cap.set(cv::CAP_PROP_FRAME_WIDTH, 160);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 96);
	cap.set(cv::CAP_PROP_FPS, 30);
	cap.set(cv::CAP_PROP_FORMAT, CV_8UC3);

	cap.open("/dev/cams/front", cv::CAP_V4L2);
	if(!cap.isOpened()) {
		std::cout << "Front cam not opened" << std::endl;
	}

	SilverML s;
	s.start();

	bool disable_side = false;

	while(1) {
		float last_side_distance = 0.0f;
		// Drive while there is a side wall
		float side_distance = robot->single_distance(DIST_2);
		while(disable_side || side_distance < 30.0f || robot->distance_avg(DIST_2, 10, 0.2f) < 30.0f) {
			last_side_distance = side_distance;
			std::cout << side_distance << std::endl;
			if(side_distance < 30.0f) {
				std::cout << "disable_side = false" << std::endl;
				disable_side = false;
			}
			cap.grab();
			cap.retrieve(frame);
			cv::Mat silver_roi = frame(cv::Range(28, 38), cv::Range(20, 62));
			bool silver = false;//s.predict_silver(silver_roi);
			if(silver) {
				std::cout << "Detected silver";
				cap.release();
				robot->m(-100, -100, 200);
			}
			if(silver || (robot->single_distance(DIST_1) < 10.0f && robot->distance_avg(DIST_1, 10, 0.2f) < 10.0f)) {
				float s_dist = robot->distance_avg(DIST_2, 10, 0.2f);
				if(s_dist > 30.0f) break;
				if(!silver) cap.release();
				std::cout << "Corner, turning" << std::endl;
				robot->m(-100, -100, silver ? 200 : 150);
				robot->turn(RAD_90);
				robot->m(-100, -100, 500);
				robot->turn(-RAD_180);
				robot->m(-100, -100, 1200);
				cap.open("/dev/cams/front", cv::CAP_V4L2);
				disable_side = true;
			}

			if(check_green_stripe(frame)) {
				std::cout << "Detected green stripe, turning" << std::endl;
				cap.release();
				robot->m(100, 100, 300);
				s.stop();
				return;
			}

			robot->m(100, 100);

			side_distance = robot->single_distance(DIST_2);
		}
		robot->m(100, 100, 150);

		std::cout << "Check for green" << std::endl;

		// Turn right and check for green strip
		robot->stop();
		robot->m(-50, -50, 150);

		robot->turn(RAD_90);
		robot->m(100, 100, 30 * last_side_distance + 100);
		
		cap.release();
		delay(100);

		cap.open("/dev/cams/front", cv::CAP_V4L2);
		delay(100);
		cap.grab();
		cap.retrieve(frame);
		if(check_green_stripe(frame)) {
			robot->m(100, 100, 500);
			//s.stop();
			cap.release();
			return;
		}
		robot->m(-100, -100, 200);
		robot->turn(-RAD_90);
		robot->m(50, 50, 150);
		disable_side = true;
	}
}

void Rescue::to_wall(uint16_t dist) {
	// aligns robot sidewards to dist
	robot->turn(RAD_90);
	while(1) {
		float curr_dist = robot->distance_avg(DIST_1, 5, 0.2f);
		if(dist + 1.0f > curr_dist) {
			robot->m(-100, -100, 20)
		} 
		else if (dist - 1.0f < curr_dist) {
			robot->m(100, 100, 20);
		} else {
			robot->turn(-RAD_90);
			return;
		}
	}
}