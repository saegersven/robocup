#include "rescue.h"

#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <pthread.h>

#include "line.h"
#include "robot.h"
#include "utils.h"

Rescue::Rescue(std::shared_ptr<Robot> robot) : finished(false) {
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

void Rescue::open_camera(uint8_t cam_id) {
	std::cout << "Opening camera" << std::endl;
	if(cam_id == CAM_SHORT) {
		caps[cam_id].set(cv::CAP_PROP_FRAME_WIDTH, 160);
		caps[cam_id].set(cv::CAP_PROP_FRAME_HEIGHT, 96);
		caps[cam_id].set(cv::CAP_PROP_FPS, 30);
		caps[cam_id].set(cv::CAP_PROP_FORMAT, CV_8UC3);
	}
	caps[cam_id].open(cam_filenames[cam_id], cv::CAP_V4L2);
	if(!caps[cam_id].isOpened()) {
		std::cout << "Failed opening camera " << std::to_string(cam_id) << std::endl;
	}
}

void Rescue::close_camera(uint8_t cam_id) {
	caps[cam_id].release();
}

cv::Mat Rescue::capture(uint8_t cam_id) {
	bool close = false;
	if(!caps[cam_id].isOpened()) {
		open_camera(cam_id);
		close = true;
	}
	cv::Mat frame;
	caps[cam_id].grab();
	caps[cam_id].retrieve(frame);
	if(close) close_camera(cam_id);

	return frame;
}

void Rescue::rescue() {
	caps.push_back(cv::VideoCapture());
	caps.push_back(cv::VideoCapture());

	victimML.init();

	robot->stop();
	robot->beep(100, BUZZER);
	robot->m(100, 100, 350);

	// Take time to align so that wall is on the right
	if(robot->distance_avg(DIST_SIDE_FRONT, 5, 0.2f) > 250) {
		// No wall on the right, so turn 90° clockwise and align
		robot->turn(-RAD_90);
		robot->m(-100, -100, 300);
		robot->turn(-RAD_180);
		robot->m(100, 100, 250);

		/*if(robot->distance(DIST_SIDE_FRONT) < 200) {
			// There is a wall right now, align
			robot->turn(RAD_90);
			robot->m(100, 100, 800);
			robot->m(-100, -100, 450);
			robot->turn(-RAD_90);
		}*/
	} else {
		if(robot->distance(DIST_SIDE_FRONT) < 120) {
			robot->turn(RAD_90);
			robot->m(-100, -100, 100);
			robot->turn(-RAD_90);
		}
	}
	robot->turn(-get_angle_to_right_wall());

	find_black_corner();

	// Drop Rescue Kit
	robot->servo(SERVO_1, ARM_DROP, 500);
	robot->servo(SERVO_2, GRAB_OPEN, 500);
	robot->servo(SERVO_2, GRAB_CLOSED, 500);
	robot->servo(SERVO_1, ARM_UP, 500);


	robot->m(100, 100, 800);
	robot->turn(RAD_180 - deg_to_rad(60.0f));

	// Search for victims
	uint8_t turn_counter = 0;
	uint8_t rescued_victims_cnt = 0;
	const int num_victims = 3;
	bool searching_dead_victim = false;

	while(true) {
		bool searching_victim = true;
		bool is_in_center = false;
		bool center_long = false;
		bool abort = false;

		while (true) {
			if(turn_counter == 12) {
				if(is_in_center) {
					if(is_in_center) {
						//robot->turn(deg_to_rad(20.0f));
						//robot->m(100, 100, 1000);
						robot->m(-100, -100, 1350);
						if(center_long) {
							robot->turn(deg_to_rad(-135.0f));
						} else {
							robot->turn(deg_to_rad(135.0f));
						}
					}
					//robot->m(-100, -100, 100);
					align_with_corner();
					robot->m(100, 100, 1400);

					//robot->turn(deg_to_rad(60.0f));
					//robot->m(100, 100, 800);
					//align_with_corner();
					//robot->m(100, 100, 1500);
					if(searching_dead_victim) {
						abort = true;
						break;
					}
					searching_dead_victim = true;

					robot->m(-100, -100, 850);
					robot->turn(deg_to_rad(-60.0f));
					turn_counter = 0;
					is_in_center = false;
				} else {
					//robot->turn(-deg_to_rad(120.0f));
					// Move to center
					robot->turn(deg_to_rad(60.0f));
					robot->m(100, 100, 1450);

					robot->m(-100, -100, 1050);
					robot->turn(deg_to_rad(-135.0f));
					uint16_t dist1 = robot->distance_avg(DIST_FORWARD, 10, 0.2f);
					uint16_t dist2 = robot->distance_avg(DIST_SIDE_FRONT, 10, 0.2f);
					std::cout << dist1 << " : " << dist2 << std::endl;
					if(dist1 > dist2) {
						center_long = false;
					} else {
						robot->turn(-RAD_90);
						center_long = true;
					}
					robot->m(100, 100, 1350);

					//robot->m(-100, -100, 1200);
					//robot->turn(-deg_to_rad(60.0f));
					is_in_center = true;
					//robot->turn(-RAD_90);
					robot->beep(100);
					turn_counter = 0;
				}
			}

			if (rescue_victim(!searching_dead_victim, turn_counter * deg_to_rad(-30.0f))) {
				// Rescue victim
				robot->turn(-RAD_180 + turn_counter * deg_to_rad(30.0f) + (!is_in_center ? deg_to_rad(60.0f) : 0));

				if(is_in_center) {
					//robot->turn(deg_to_rad(20.0f));
					//robot->m(100, 100, 1000);
					robot->m(-100, -100, 1350);
					if(center_long) {
						robot->turn(deg_to_rad(-135.0f));
					} else {
						robot->turn(deg_to_rad(135.0f));
					}
				}
				//robot->m(-100, -100, 100);
				align_with_corner();
				robot->m(100, 100, 1450);

			    robot->m(-100, -100, 600);
				robot->turn(RAD_180);

				robot->m(-30, -30, 1200);

				// unload victim
				robot->servo(SERVO_1, ARM_DROP, 500);
				robot->servo(SERVO_2, GRAB_OPEN, 500);
				robot->servo(SERVO_2, GRAB_CLOSED, 500);
				robot->servo(SERVO_1, ARM_UP, 500);

				++rescued_victims_cnt;
				turn_counter = 0;
				if(searching_dead_victim) {
					robot->m(100, 100, 150);
					robot->turn(RAD_180);
					robot->m(100, 100, 750);
					abort = true;
				}
				if(rescued_victims_cnt == 2) searching_dead_victim = true;
				break;
			} else {
				std::cout << "looking for victim" << std::endl;
				robot->turn(deg_to_rad(-30.0f));
				delay(50);
				++turn_counter;
			}
		}
		if(abort) break;

		is_in_center = false;

		/*if(rescued_victims_cnt == num_victims - 1) {
			robot->m(100, 100, 200);
			robot->turn(RAD_180);
			robot->m(100, 100, 700);
		} else {*/
		robot->m(100, 100, 750);
		robot->turn(RAD_180 - deg_to_rad(60.0f));
		turn_counter = 0;
		//}
	}

	find_exit();

	finished = true;
}

void Rescue::align_with_corner() {
	cv::Mat frame = capture(CAM_FAR);
	
	float x = 0.0f;
	float y = 0.0f;
	uint32_t num_black_pixels = 0;
	for(int i = 0; i < frame.rows; ++i) {
		cv::Vec3b* p = frame.ptr<cv::Vec3b>(i);
		for(int j = 0; j < frame.cols; ++j) {
			if(is_black(p[j][0], p[j][1], p[j][2])) {
				x += (float)j;
				//y += (float)i;
				++num_black_pixels;
			}
		}
	}
	x /= num_black_pixels;

	const float FAR_CAM_FOV = deg_to_rad(65.0f);
	float angle = (x / frame.cols - 0.5f) * FAR_CAM_FOV;

	robot->turn(angle);
}

float Rescue::get_angle_to_right_wall() {
	float dist_front = robot->distance(DIST_SIDE_FRONT);
	float dist_back = robot->distance(DIST_SIDE_BACK);
	if(dist_front > 400 || dist_back > 400) return 0.0f;
	float angle = std::atan((dist_back - dist_front) / 145.0f);
	if(std::abs(angle) > deg_to_rad(10.0f)) {
		dist_front = robot->distance_avg(DIST_SIDE_FRONT, 10, 0.2f);
		dist_back = robot->distance_avg(DIST_SIDE_BACK, 10, 0.2f);
		angle = std::atan((dist_back - dist_front) / 145.0f);
	}
	return angle;
}

void Rescue::turn_90_wall() {
	robot->turn(-get_angle_to_right_wall());
	robot->turn(RAD_90);
	robot->m(-100, -100, 200);
	robot->turn(-RAD_180);
	robot->turn(-get_angle_to_right_wall());
}

void Rescue::find_black_corner() {
	std::cout << "Searching for corner" << std::endl;
	while(robot->distance_avg(DIST_FORWARD, 10, 0.2f) < 300) {
		robot->m(-100, -100, 200);
	}
	robot->stop();
	const float DISTANCE_PER_STEP = 200.0f; // Approximate distance driven each step [mm]
	const float GOAL_DISTANCE = 60.0f;
	uint64_t last_turn = micros();
	while(true) {
		// Align with right wall
		if(micros() - last_turn >= 500000) {
			uint16_t side_dist = robot->distance(DIST_SIDE_FRONT);
			if(side_dist < 450) {
				if(side_dist > 130) {
					robot->turn(RAD_90);
					while(robot->distance(DIST_FORWARD) > 155) {
						robot->m(50, 50);
					}
					robot->stop();
					robot->turn(-RAD_90);
				}
			}
			robot->turn(-get_angle_to_right_wall());
			last_turn = micros();
			//std::cout << "Turning done" << std::endl;
		}

		robot->m(100, 100);

		// Check for exit ahead
		if(robot->distance(DIST_SIDE_FRONT) > 450) {
			robot->stop();
			delay(100);
			robot->m(-100, -100, 300);
			cv::Mat short_frame = capture(CAM_SHORT);
			if(check_for_green(short_frame)) {
				// Wall
				std::cout << "Exit, turning" << std::endl;
				robot->m(-100, -100, 400);
				turn_90_wall();
			} else {
				robot->m(100, 100, 380);
			}
		}

		if(robot->distance(DIST_FORWARD) < 460 && robot->distance_avg(DIST_FORWARD, 10, 0.2f) < 470) {// Check for corner
			robot->stop();
			delay(200);

			cv::Mat frame = capture(CAM_FAR);
			cv::Mat roi = frame(cv::Range(CORNER_ROI_Y_MIN, CORNER_ROI_Y_MAX),
				cv::Range(CORNER_ROI_X_MIN, CORNER_ROI_X_MAX));
			cv::Mat black;
			inRange(roi, cv::Scalar(0, 0, 0), cv::Scalar(100, 100, 100), black);
			uint32_t num_pixels = cv::countNonZero(black);

			cv::imshow("Corner ROI", roi);
			cv::imshow("Corner threshold", black);
			cv::waitKey(1);

			float pixel_fraction = (float)num_pixels / roi.cols / roi.rows;
			std::cout << num_pixels << " black pixels (" << pixel_fraction * 100.0f << " %)" << std::endl;
		
			if(pixel_fraction > 0.20) {
				std::cout << "Found corner. Aligning to wall" << std::endl;
				// Over 30% of the screen is black -> corner
				robot->turn(-get_angle_to_right_wall());
				std::cout << "Adjusting distance" << std::endl;

				const uint16_t CORNER_WALL_DISTANCE = 350;
				float distance_diff = (float)robot->distance_avg(DIST_FORWARD, 5, 0.2f) - CORNER_WALL_DISTANCE;
				robot->m(100, 100, distance_diff * DISTANCE_FACTOR);

				std::cout << "Aligning with corner" << std::endl;
				robot->turn(deg_to_rad(120.0f));
				robot->m(-100, -100, 600);

				//robot->turn(-RAD_90);
				//robot->m(100, 100, 500);
				//robot->m(-100, -100, 150);
				robot->turn(deg_to_rad(95.0f));

				robot->m(-30, -30, 800);

				return;
			} else {
				std::cout << "Wall" << std::endl;
				robot->m(100, 100, 500);
				turn_90_wall();
			}
		}
	}
}

bool Rescue::rescue_victim(bool ignore_dead, float angle_offset) {
	victimML = VictimML();
	victimML.init();

	cv::Mat frame = capture(CAM_FAR);
	cv::Mat probability_map = victimML.invoke(frame);
	std::vector<Victim> victims = victimML.extract_victims(probability_map);

	std::cout << victims.size() << " victims found." << std::endl;

	cv::Mat debug_map = two_channel_to_three_channel(probability_map);
	cv::resize(debug_map, debug_map, cv::Size(320, 240));
	cv::imshow("lol", debug_map);
	cv::imshow("Frame", frame);
	save_img("home/pi/Desktop/potential_victims_far_cam", frame);
	cv::waitKey(1);

	const uint16_t WIDTH = 160;
	const uint16_t HEIGHT = 120;
	const float FAR_CAM_FOV = deg_to_rad(65.0f);

	bool victim_selected = false;
	Victim selected_victim = {false, 0.0f, 0.0f};

	// Select lowest victim ignoring dead victims if ignore_dead is true
	for(int i = 0; i < victims.size(); ++i) {
		if(!victims[i].dead || !ignore_dead) {
			if(victims[i].y > selected_victim.y) {
				selected_victim = victims[i];
				victim_selected = true;
			}
		}
	}

	if(!victim_selected) return false;

	float angle = (selected_victim.x / WIDTH - 0.5) * FAR_CAM_FOV;
	robot->turn(angle);

	// save image for debugging and ml purposes:
	uint32_t num_steps = 0;
	uint32_t ms_travelled = 0;
	while(selected_victim.y < HEIGHT * 0.7f) {
		uint16_t approach_step_size = 400 - 350 * selected_victim.y / HEIGHT;
		++num_steps;
		robot->m(100, 100, approach_step_size);
		ms_travelled += approach_step_size;
		delay(200);

		frame = capture(CAM_FAR);
		if(frame.empty()) {
			std::cout << "Empty frame" << std::endl;
			robot->m(-100, -100, approach_step_size);
			continue;
		}
		probability_map = victimML.invoke(frame);
		victims = victimML.extract_victims(probability_map);

		cv::Mat debug_map = two_channel_to_three_channel(probability_map);
		cv::resize(debug_map, debug_map, cv::Size(320, 240));
		cv::imshow("lol", debug_map);
		cv::imshow("Frame", frame);
		save_img("home/pi/Desktop/potential_victims_far_cam", frame);

		cv::waitKey(1);

		victim_selected = false;
		selected_victim = {false, 0.0f, 0.0f};

		// Select victim that is closest to the center
		for(int i = 0; i < victims.size(); ++i) {
			if(abs(victims[i].x - WIDTH / 2) < abs(selected_victim.x - WIDTH / 2)) {
				selected_victim = victims[i];
				victim_selected = true;
			}
		}
		std::cout << selected_victim.y << std::endl;

		float angle2 = (selected_victim.x / WIDTH - 0.5) * FAR_CAM_FOV;
		robot->turn(angle2);
	}

	// Pick up victim
	robot->m(-100, -100, 530);
	robot->turn(RAD_180);
	robot->servo(SERVO_2, GRAB_OPEN, 750);
	robot->servo(SERVO_1, ARM_ALMOST_DOWN, 650);
	robot->m(-50, -50, 340);
	robot->servo(SERVO_1, ARM_DOWN, 30);
	robot->servo(SERVO_2, GRAB_CLOSED, 750);
	robot->m(50, 50, 180);
	robot->servo(SERVO_1, ARM_UP, 750);

	robot->m(100, 100, ms_travelled - 400);

	robot->turn(-angle);

	/*frame = capture(CAM_FAR);
	cv::Mat thresh;
	cv::inRange(frame, cv::Scalar(0, 0, 0), cv::Scalar(100, 100, 100), thresh);

	std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    float max_area = 0.0f;
    std::vector<cv::Point> largest_contour;
    for(int i = 0; i < contours.size(); ++i) {
    	float area = cv::contourArea(contours[i]);
    	if(area > max_area) {
    		max_area = area;
    		largest_contour = contours[i];
    	}
    }

    cv::Rect contour_rect = cv::boundingRect(largest_contour);
    std::cout << "Contour pos: " << contour_rect.x << std::endl;
    float angle_to_corner = ((contour_rect.x + contour_rect.width / 2) / thresh.cols - 0.5) * FAR_CAM_FOV;

    robot->turn(angle_to_corner);*/

	

	//robot->m(100, 100, 450);

	return true;
}

bool Rescue::check_for_green(cv::Mat frame) {
	uint32_t num_green_pixels = 0;
	cv::Mat green = in_range(frame, &is_green, &num_green_pixels);

	float green_percentage = (float)num_green_pixels / frame.cols / frame.rows;
	std::cout << "Green: " << green_percentage << std::endl;
	return green_percentage > 0.1f;
}

void Rescue::find_exit() {
	robot->m(-100, -100, 300);
	robot->turn(RAD_45);
	robot->m(-100, -100, 700);
	robot->turn(-RAD_90);

	// TODO: Check for green

	while(robot->distance(DIST_FORWARD) > 200) {
		robot->m(60, 60);
	}
	robot->stop();
	robot->turn(-RAD_90);
	robot->turn(-get_angle_to_right_wall());

	bool disable_side = false;
	uint16_t last_side_distance = 0;
	while(true) {
		uint16_t dist = robot->distance(DIST_SIDE_FRONT);
		// Start searching for green
		uint64_t last_turn = micros();
		bool short_cam_open = false;

		while(disable_side || dist < 350) {
			if(micros() - last_turn >= 500000) {
				if(dist < 450) {
					if(dist > 130) {
						robot->turn(RAD_90);
						while(robot->distance(DIST_FORWARD) > 155) {
							robot->m(50, 50);
						}
						robot->stop();
						robot->turn(-RAD_90);
						robot->m(-100, -100, 200);
					}
				}
				robot->turn(-get_angle_to_right_wall());
				last_turn = micros();
			}

			//std::cout << dist << std::endl;

			if(dist < 350) {
				disable_side = false;
			}

			last_side_distance = dist;
			robot->m(100, 100);

			uint16_t dist_forward = robot->distance(DIST_FORWARD);
			std::cout << "Forward dist: " << dist_forward << std::endl;

			if(dist_forward < 120) {
				std::cout << "Wall" << std::endl;
				robot->turn(-get_angle_to_right_wall());
				robot->m(-100, -100, 150);
				robot->turn(RAD_90);
				robot->m(-100, -100, 200);
				robot->turn(-RAD_180);
				robot->turn(-get_angle_to_right_wall());
			}/* else if(dist_forward > 1300) {
				std::cout << "Exit ahead" << std::endl;
				if(!short_cam_open) {
					open_camera(CAM_SHORT);
					short_cam_open = true;
				}
				uint64_t start = micros();
				std::cout << "Reading frame" << std::endl;
				cv::Mat frame = capture(CAM_SHORT);
				std::cout << "Frame took: " << (micros() - start) / 1000 << std::endl;

				if(check_for_green(frame)) {
					// Found exit
					robot->m(100, 100, 300);
					return;
				} else {
					uint32_t num_black_pixels = 0;
					in_range(frame, &is_black, &num_black_pixels);
					float black = (float)num_black_pixels / frame.cols / frame.rows;

					std::cout << "Black: " << black << std::endl;

					if(black > 0.06f) {
						// Silver
						std::cout << "Silver" << std::endl;
						robot->turn(-get_angle_to_right_wall());
						robot->m(-100, -100, 300);
						robot->turn(RAD_90);
						robot->m(-100, -100, 200);
						robot->turn(-RAD_180);
						robot->turn(-get_angle_to_right_wall());
					}
				}
			} else if(short_cam_open) {
				close_camera(CAM_SHORT);
				short_cam_open = false;
			}*/
			dist = robot->distance(DIST_SIDE_FRONT);
		}
		robot->stop();
		robot->m(-100, -100, 400);
		cv::Mat frame = capture(CAM_SHORT);

		if(check_for_green(frame)) {
			// Found exit
			robot->m(100, 100, 300);
			return;
		}

		robot->m(100, 100, 670);

		// Turn right and check for green
		robot->turn(RAD_90);
		uint16_t time = 1 * last_side_distance + 150;
		robot->m(100, 100, time);
		frame = capture(CAM_SHORT);

		if(check_for_green(frame)) {
			// Found exit
			robot->m(100, 100, 500);
			return;
		}

		robot->m(-100, -100, time);
		robot->turn(-RAD_90);
		disable_side = true;
	}
}