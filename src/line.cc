#include "line.h"

#include <algorithm>
#include <vector>
#include <limits>
#include <cmath>
#include <future>

#include <opencv2/opencv.hpp>

#include "robot.h"
#include "utils.h"
#include "rescue.h"

bool is_black(uint8_t b, uint8_t g, uint8_t r) {
	return (uint16_t)b + (uint16_t)g + (uint16_t)r < BLACK_MAX_SUM;
}

bool is_green(uint8_t b, uint8_t g, uint8_t r) {
	return 1.0f / GREEN_RATIO_THRESHOLD * g > b + r && g > GREEN_MIN_VALUE;
}

bool is_blue(uint8_t b, uint8_t g, uint8_t r) {
	return 1.0f / BLUE_RATIO_THRESHOLD * b > g + r && b > BLUE_MIN_VALUE;
}

Line::Line(int front_cam_id, std::shared_ptr<Robot> robot)
	: obstacle_active(0), running(false), last_frame_t(std::chrono::high_resolution_clock::now()) {
	this->front_cam_id = front_cam_id;
	this->robot = robot;

	this->average_silver = cv::imread(RUNTIME_AVERAGE_SILVER_PATH);
	this->distance_weight_map = cv::imread(RUNTIME_DISTANCE_WEIGHT_MAP_PATH, cv::IMREAD_GRAYSCALE);
}

void Line::start() {
	// Start video feed from front camera
	robot->start_video(front_cam_id);

	running = true;

	line_angle_integral = 0.0f;
	last_update = std::chrono::high_resolution_clock::now();

	std::thread obstacle_thread([this]{obstacle();});
	obstacle_thread.detach();
	obstacle_active = 0;
	micros_start = micros();
}

void Line::stop() {
	// Reset if running
	if(!running) return;

	robot->stop_video(front_cam_id);
	obstacle_active = 0;
	// Setting running to false will notify obstacle thread to stop
	running = false;
}

bool Line::check_silver_distance() {
	/*
	if (90.00 < robot->single_distance(DIST_1, 20, 0.1f, 5000) && robot->distance(DIST_1, 20, 0.1f, 5000) < 130.00) {
		return true;
	}
	*/
	return false;
}

bool Line::check_silver(cv::Mat& frame) {
	if(micros() - micros_start < 60000000) return false; 

	const float MINIMUM_RATIO = 0.52; // Ratio of red to sum of blue and green
	const float MINIMUM_VALUE = 200; // Note: This is the total, not the average
	const float CENTER_MINIMUM_VALUE = 220;

	// Left red dot
	cv::Mat roi_l = frame(cv::Range(30, 37), cv::Range(23, 30));
	cv::Vec3b col_l = average_color(roi_l);
	float r_l = (float)col_l[2] / (col_l[1] + col_l[0]);
	float v_l = (float)col_l[2] + col_l[1] + col_l[0];

	if(r_l < MINIMUM_RATIO || v_l < MINIMUM_VALUE) return false;

	std::cout << "Right dot" << std::endl;
	// Right red dot
	cv::Mat roi_r = frame(cv::Range(30, 37), cv::Range(52, 59));
	cv::Vec3b col_r = average_color(roi_r);
	float r_r = (float)col_r[2] / (col_r[1] + col_r[0]);
	float v_r = (float)col_r[2] + col_r[1] + col_r[0];

	cv::imshow("roi_r", roi_r);

	if(r_r < MINIMUM_RATIO || v_r < MINIMUM_VALUE) return false;

	/*std::cout << "Distance" << std::endl;
	// Distance
	float dist = robot->distance(DIST_1, 2, 500);
	std::cout << dist << std::endl;
	if(dist < 90.0f || dist > 130.0f) return false;*/

	std::cout << "Center" << std::endl;
	// Center
	cv::Mat roi_c = frame(cv::Range(30, 37), cv::Range(25, 50));
	cv::Vec3b col_c = average_color(roi_c);
	float v_c = (float)col_c[2] + col_c[1] + col_c[0];

	//std::cout << r_l << "\t" << r_r << std::endl;

	return v_c > CENTER_MINIMUM_VALUE;
}

bool Line::abort_obstacle(cv::Mat frame) {
#ifdef DEBUG
	cv::Mat debug = frame.clone();
	cv::rectangle(debug, cv::Point(OBSTACLE_X_LOWER, OBSTACLE_Y_LOWER),
		cv::Point(OBSTACLE_X_UPPER, OBSTACLE_Y_UPPER), cv::Scalar(0, 255, 0), 2);
	cv::imshow("Debug", debug);
	cv::waitKey(1);
#endif

	cv::Mat cut = frame(cv::Range(OBSTACLE_Y_LOWER, OBSTACLE_Y_UPPER),
		cv::Range(OBSTACLE_X_LOWER, OBSTACLE_X_UPPER));
	cv::Mat black = in_range(cut, &is_black);
	uint32_t non_zero = cv::countNonZero(black);
	std::cout << non_zero << std::endl;
	return non_zero > 200;
}

// ASYNC
void Line::obstacle() {
	while(running) {
		if(obstacle_active != 1) continue;
		if(robot->single_distance(DIST_1, 1000) < 9.0f) {
			robot->set_gpio(LED_1, true);
			robot->stop();
			robot->block();
			delay(10);
			if(robot->distance_avg(DIST_1, 10, 0.2f, 500, 3000) < 9.5f) {
				if(robot->distance_avg(DIST_1, 30, 0.2f, 500, 10000) < 9.5f) {				
					std::cout << "Obstacle" << std::endl;
					obstacle_active = 2;
				}
			}
			robot->set_gpio(LED_1, false);
			robot->block(false);
		}
	}
}

bool Line::obstacle_straight_line(uint32_t duration) {
	auto start_t = std::chrono::high_resolution_clock::now();

	robot->stop_video(front_cam_id);
	robot->start_video(front_cam_id);

	while(1) {
		robot->m(80, 80);

		cv::Mat frame = robot->capture(front_cam_id);
		cv::imshow("Debug", frame);
		cv::waitKey(1);
		cv::Mat roi = frame(cv::Range(26, 47), cv::Range(26, 55));
		uint32_t roi_size = roi.cols * roi.rows;

		uint32_t num_black = 0;
		in_range(frame, &is_black, &num_black);

		float p = (float)num_black / roi_size;
		std::cout << p << std::endl;

		if(p > 0.8f) {
			// Abort
			robot->stop();
			return true;
		}

		auto now = std::chrono::high_resolution_clock::now();
		uint32_t elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_t).count();
		if(elapsed >= duration) return false;
	}
}

bool Line::line(cv::Mat& frame) {
	// Check if obstacle thread has notified main thread
	if(obstacle_active == 2) {
		robot->stop();
		if(robot->distance_avg(DIST_1, 10, 0.2f, 500, 5000) < 9.0f) {
			std::cout << "Obstacle!" << std::endl;
			robot->set_gpio(LED_2, true);				
			robot->m(-80, -80, 100);

			robot->turn(RAD_90);
			robot->m(80, 80, 200);

			robot->stop_video(front_cam_id);
			delay(50);
			robot->start_video(front_cam_id);

			const uint32_t durations[] = {550, 1450, 1250, 1250, 450};

			for(int i = 0; i < 42; ++i) {
				if(obstacle_straight_line(durations[i])) break;

				robot->stop_video(front_cam_id);
				robot->turn(-RAD_90);
				robot->start_video(front_cam_id);
			}

			robot->set_gpio(LED_2, false);
			robot->m(-40, -40, 150);
			robot->m(-40, 40, 250);
			robot->m(80, 80, 350);
			robot->m(-80, 80, 250);
		}

		obstacle_active = 0;
	} else {
		obstacle_active = 1;
		auto start_t = std::chrono::high_resolution_clock::now();
#ifdef DEBUG
		debug_frame = frame.clone();
#endif

		cv::Mat black = in_range(frame, &is_black);

		follow(frame, black);

		//rescue_kit(frame);

		green(frame, black);

		if(check_silver(frame)) {
			std::cout << "cam detected silver!\nchecking distance..." << std::endl;
			robot->m(100, 100, 1400);
			robot->stop();
			delay(100);

			float dist_front = robot->distance_avg(DIST_1, 100, 0.3f);
			float dist_side = robot->distance_avg(DIST_2, 100, 0.3f);

			std::cout << "Front distance: " << dist_front << std::endl;
			std::cout << "Side distance: " << dist_side << std::endl;


			// check front distance < foo and side distance < foo and black pixels in frame < foo
			// increase rescue_cnt for each
			// #redundancy
			int rescue_cnt = 0;
			if (dist_front > 50.0f && dist_front < 130.0f) rescue_cnt++;
			if (dist_side > 3.0f && dist_side < 120.0f) rescue_cnt++;
			if (black_pixel_threshold_under(1000)) rescue_cnt++;
			
			std::cout << "Rescue_cnt: " << rescue_cnt << std::endl;

			if (rescue_cnt >= 2) {
				return true;
			} else {
				robot->m(-100, -100, 1350); // return to previous position (a bit further to avoid another false positive)
			}
		}
	}
#ifdef DEBUG
#ifdef DEBUG_RESIZE
	cv::resize(debug_frame, debug_frame, cv::Size(), 4.00, 4.00);
#endif
	cv::imshow("Debug", debug_frame);
	cv::waitKey(1);
#endif

	cv::imshow("Frame", frame);
	cv::waitKey(1);

#ifdef FPS_COUNTER
	auto end_t = std::chrono::high_resolution_clock::now();
	uint32_t us = std::chrono::duration_cast<std::chrono::microseconds>(end_t - last_frame_t).count();
	float l_fps = 1.0 / ((float)us / 1'000'000);
	fps = fps * 0.9f + l_fps * 0.1f;
	std::cout << fps << " fps, " << us << "us" << std::endl;
	last_frame_t = end_t;
#endif
	return false;
}

float Line::difference_weight(float x) {
	return 0.25f + 0.75f * std::pow(2, -std::pow(x * 5, 2));
}

float Line::distance_weight(float x) {
	return std::pow(2, -std::pow(((x - 0.65) * 4), 2));
}

float Line::circular_line(cv::Mat& in) {
	float weighted_line_angle = 0.0f;
	float total_weight = 0.0f;

	uint32_t num_angles = 0;

	//std::vector<std::pair<float, float>> line_angles; // Map of angle and distance

	uint8_t* p;
	uint8_t* p_dw;

	//cv::Point2f center(in.cols / 2, in.rows); // Bottom center
	float center_x = in.cols / 2.0f;
	float center_y = in.rows;

	int i, j;
	for(i = 0; i < in.rows; ++i) {
		p = in.ptr<uint8_t>(i);
		p_dw = distance_weight_map.ptr<uint8_t>(i);
		for(j = 0; j < in.cols; ++j) {
			if(p[j]) {
				float x = (float)j;
				float y = (float)i;

				uint8_t dw = p_dw[j];
				if(dw) {
					++num_angles;

					float pixel_distance_weight = dw / 255.0f;
					float angle = std::atan2(y - center_y, x - center_x) + (PI / 2.0f);
					float angle_difference_weight = difference_weight((angle - last_line_angle) / PI * 2.0f);
					
					weighted_line_angle += angle_difference_weight * pixel_distance_weight * angle;
					total_weight += angle_difference_weight * pixel_distance_weight;
				}
			}
		}
	}

	if(num_angles < 40) return 0.0f;

	weighted_line_angle /= total_weight;
	//average_difference_weight /= num_angles;
	//average_line_angle /= average_difference_weight;

	return weighted_line_angle;
}

void Line::follow(cv::Mat& frame, cv::Mat black) {
	//std::cout << "Follow" << std::endl;

	//cv::Mat black = in_range_black(frame);

	float line_angle = circular_line(black);

	if(std::isnan(line_angle)) line_angle = 0.0f;

	last_line_angle = line_angle;

#ifdef DEBUG
	cv::Point center(debug_frame.cols / 2, debug_frame.rows);

	cv::circle(debug_frame, center, MINIMUM_DISTANCE, cv::Scalar(0, 255, 0), 2);
	cv::circle(debug_frame, center, MAXIMUM_DISTANCE, cv::Scalar(0, 255, 0), 2);

	cv::line(debug_frame,
		cv::Point(std::sin(line_angle) * MINIMUM_DISTANCE, -std::cos(line_angle) * MINIMUM_DISTANCE) + center,
		cv::Point(std::sin(line_angle) * MAXIMUM_DISTANCE, -std::cos(line_angle) * MAXIMUM_DISTANCE) + center,
		cv::Scalar(0, 255, 0), 2
		);
#endif

	// auto now = std::chrono::high_resolution_clock::now();
	// float dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update).count() / 1000000.0f;
	// last_update = now;

	// this->line_angle_integral = line_angle_integral * FOLLOW_LAST_I_FACTOR + line_angle * dt;

	int16_t error = line_angle * FOLLOW_P_FACTOR;
	/* + line_angle_integral * FOLLOW_I_FACTOR; */

#ifndef MOVEMENT_OFF
	robot->m(FOLLOW_MOTOR_SPEED - error, FOLLOW_MOTOR_SPEED + error);
#endif
}

void Line::add_to_group_center(int x_pos, int y_pos, cv::Mat ir, uint32_t& num_pixels, float& center_x, float& center_y) {
	int col_limit = ir.cols - 1;
	int row_limit = ir.rows - 1;

	for(int y = -1; y <= 1; ++y) {
		int y_index = y_pos + y;

		uint8_t* p = ir.ptr<uint8_t>(y_index);

		for(int x = -1; x <= 1; ++x) {
			if(y == 0 && x == 0) continue;

			int x_index = x_pos + x;

			if(p[x_index] == 0xFF) {
				p[x_index] = 0x7F;
				center_x += (float)x_index;
				center_y += (float)y_index;
				++num_pixels;

				if(x_index > 0 && x_index < col_limit && y_index > 0 && y_index < row_limit)
					add_to_group_center(x_index, y_index, ir, num_pixels, center_x, center_y);
			}
		}
	}
}

std::vector<Group> Line::find_groups(cv::Mat frame, cv::Mat& ir, std::function<bool (uint8_t, uint8_t, uint8_t)> f) {
	std::vector<Group> groups;

	uint32_t num_pixels = 0;
	ir = in_range(frame, f, &num_pixels);

	if(num_pixels < 50) return groups;

	for(int y = 0; y < ir.rows; ++y) {
		uint8_t* p = ir.ptr<uint8_t>(y);
		for(int x = 0; x < ir.cols; ++x) {
			// Don't check for non-zero, as found pixels are set to 0xFE instead
			// so information does not get lost, when using matrix later,
			// check for non-zero pixels
			if(p[x] == 0xFF) {
				//std::cout << "Creating group at " << x << ", " << y << std::endl;
				uint32_t num_group_pixels = 0;
				float group_center_x = 0.0f;
				float group_center_y = 0.0f;
				add_to_group_center(x, y, ir, num_group_pixels, group_center_x, group_center_y);

				if(num_group_pixels > 100) {
					group_center_x /= num_group_pixels;
					group_center_y /= num_group_pixels;
					groups.push_back({group_center_x, group_center_y, num_group_pixels});
				}
			}
		}
	}

	return groups;
}

uint8_t Line::green_direction(cv::Mat& frame, cv::Mat& black, float& global_average_x, float& global_average_y) {
	uint8_t green_mask = 0;

	cv::Mat green;
	std::vector<Group> groups = find_groups(frame, green, &is_green);

	const int cut_width = 30;
	const int cut_height = 30;

	// Check if all of the groups are between certain y values to prevent premature evaluation
	// of a dead-end or late evaluation of green points behind a line, when the lower line is
	// already out of the frame
	for(int i = 0; i < groups.size(); ++i) {
		if(groups[i].y < 10) return 0;
		if(groups[i].y > 35) return 0;
		//if(groups[i].x < 8) return 0;
		//if(groups[i].x > frame.cols - 8) return 0;
	}

	global_average_x = 0.0f;
	global_average_y = 0.0f;
	uint8_t num_green_points = 0; // Amount of green points below the line

	if(groups.size() > 0) {
		std::cout << std::to_string(groups.size()) << " ---" << std::endl;
	}

	std::vector<uint8_t> decisions(groups.size());

	// Cut out part of the black matrix around the group centers
	for(int i = 0; i < groups.size(); ++i) {
		// Horizontal range
		int x_start = groups[i].x - cut_width / 2;
		int x_end = groups[i].x + cut_width / 2;
		if(x_start < 0) x_start = 0;
		if(x_end > black.cols) x_end = black.cols;

		// Vertical range
		int y_start = groups[i].y - cut_height / 2;
		int y_end = groups[i].y + cut_height / 2;
		if(y_start < 0) y_start = 0;
		if(y_end > black.rows) y_end = black.rows;

		// Calculate average black pixel in the cut
		float average_x = 0.0f;
		float average_y = 0.0f;

		uint32_t num_pixels = 0;

		uint8_t* p;
		uint8_t* p_grn;
		int y, x;
		for(y = y_start; y < y_end; ++y) {
			p = black.ptr<uint8_t>(y);
			p_grn = green.ptr<uint8_t>(y);
			for(x = x_start; x < x_end; ++x) {
				if(p[x] && !p_grn[x]) {
					average_x += (float)x;
					average_y += (float)y;
					++num_pixels;
				}
			}
		}
		average_x /= num_pixels;
		average_y /= num_pixels;

		// Check quadrant of the average pixel to determine location of green point relative to line
		if(average_y < groups[i].y) {
			// Only consider point if average is above

			// Convert local average point and add to global average
			global_average_x += average_x;
			global_average_y += average_y;
			++num_green_points;


			uint8_t dir_bit = average_x < groups[i].x ? 0x02 : 0x01;
			decisions[i] = dir_bit;

			std::cout << "Dir: " << std::to_string(dir_bit) << std::endl;
			green_mask |= dir_bit;
		}
	}
	if(groups.size() > 0) {
		std::cout << "----" << std::endl;
		for(int i = 0; i < groups.size(); ++i) {
			cv::circle(debug_frame, cv::Point(groups[i].x, groups[i].y), 2, cv::Scalar(255, 100, 100), 2);
			cv::putText(debug_frame, std::to_string(decisions[i]), cv::Point(groups[i].x + 3, groups[i].y + 8), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0), 2);
		}
	}
	global_average_x /= num_green_points;
	global_average_y /= num_green_points;

	return green_mask;
}

void Line::green(cv::Mat& frame, cv::Mat& black) {
	float global_average_x, global_average_y;
	uint8_t green_result = green_direction(frame, black, global_average_x, global_average_y);


#ifndef MOVEMENT_OFF
	if(green_result != 0) {
		cv::circle(debug_frame, cv::Point(global_average_x, global_average_y), 2, cv::Scalar(0, 255, 255), 2);

		cv::imshow("Green_debug", debug_frame);
		cv::waitKey(1);
		// The global average point roughly represents the center of the intersection
		// When traversing, move to that point first, then rotate

		float center_x = frame.cols / 2.0f;
		float center_y = frame.rows + 20.0f;
		float angle = std::atan2(global_average_y - center_y, global_average_x - center_x) + (PI / 2.0f);
		float distance = std::sqrt(std::pow(global_average_y - center_y, 2) + std::pow(global_average_x - center_x, 2));

		// std::cout << angle << std::endl;
		// std::cout << distance << std::endl;

		// Stop video to keep camera from freezing
		robot->stop_video(front_cam_id);

		robot->stop();

		robot->turn(angle);
		delay(300);

		robot->m(100, 100, DISTANCE_FACTOR * (distance - 45));
		delay(200);

		// Take another picture
		frame = robot->capture(front_cam_id);
		black = in_range(frame, &is_black);

		auto millisecondsUTC = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		cv::imwrite("/home/pi/Desktop/green_images/" + std::to_string(millisecondsUTC) + ".png", frame);

		cv::imshow("Green frame", frame);
		cv::waitKey(100);

		// Re-determine green result
		uint8_t new_green_result = green_direction(frame, black, global_average_x, global_average_y);

		robot->m(60, 60, 250);

		if(new_green_result == GREEN_RESULT_DEAD_END ||
			green_result == GREEN_RESULT_DEAD_END) {
			std::cout << "DEAD-END" << std::endl;
			robot->turn(deg_to_rad(180.0f));
			robot->m(60, 60, 150);
		} else if(green_result == GREEN_RESULT_LEFT) {
			std::cout << "LEFT" << std::endl;
			robot->turn(deg_to_rad(-70.0f));
		} else if(green_result == GREEN_RESULT_RIGHT) {
			std::cout << "RIGHT" << std::endl;
			robot->turn(deg_to_rad(70.0f));
		}
		delay(100);
		robot->m(60, 60, 130);
		robot->start_video(front_cam_id);
	}
#endif
}

void Line::rescue_kit(cv::Mat& frame) {
	cv::Mat blue;
	std::vector<Group> groups = find_groups(frame, blue, &is_blue);

#ifdef DEBUG
	cv::imshow("Blue", blue);
#endif

	if(groups.size() > 0) {
		Group group = groups[0];
		// If there is more than one group, select the one with most pixels,
		// as the smaller ones are likely noise
		if(groups.size() > 1) {
			for(int i = 1; i < groups.size(); ++i) {
				if(groups[i].num_pixels > group.num_pixels) {
					group = groups[i];
				}
			}
		}
		// Position robot
		float center_x = frame.cols / 2.0f;
		float center_y = frame.rows + 20;
		float angle = std::atan2(group.y - center_y, group.x - center_x) + (PI / 2.0f);
		float distance = std::sqrt(std::pow(group.y - center_y, 2) + std::pow(group.x - center_x, 2));

		// Stop video so camera doesn't freeze
		robot->stop_video(front_cam_id);

		// m() can handle negative durations
		//robot->m(60, -60, 300 * angle);
		robot->turn(angle);

		stop();
		delay(500);

		// Maximum distance is 68, so 44 seems like a good value
		if(distance < 44) {
			robot->m(-60, -60, DISTANCE_FACTOR * (distance - 44));
		} else {
			robot->m(60, 60, DISTANCE_FACTOR * (distance - 44));
		}

		delay(1000);

		robot->m(-60, -60, 720);
		delay(200);
		robot->turn(RAD_180);

		//delay(1000);
		robot->servo(SERVO_2, GRAB_OPEN, 750);
		robot->servo(SERVO_1, ARM_DOWN, 750);

		robot->attach_servo(SERVO_2);
		robot->write_servo(SERVO_2, GRAB_CLOSED);
		delay(500);
		robot->servo(SERVO_1, ARM_UP, 750);
		robot->write_servo(SERVO_2, GRAB_OPEN);
		delay(200);
		robot->write_servo(SERVO_2, GRAB_CLOSED);
		delay(200);
		robot->release_servo(SERVO_2);

		delay(200);
		// Reposition to continue
		robot->turn(deg_to_rad(180.0f));
		delay(200);
		robot->m(60, 60, 500);
		delay(1000);

		// Restart video to continue
		robot->start_video(front_cam_id);
	}
}

bool Line::black_pixel_threshold_under(int threshold) {
	return true;
}