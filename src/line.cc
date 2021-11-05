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

#define BLACK_MIN_SUM 300

bool is_black(uint8_t b, uint8_t g, uint8_t r) {
	return (uint16_t)b + (uint16_t)g + (uint16_t)r < BLACK_MIN_SUM;
}

#define GREEN_RATIO_THRESHOLD 0.6f
#define GREEN_MIN_VALUE 20

bool is_green(uint8_t b, uint8_t g, uint8_t r) {
	return 1.0f / GREEN_RATIO_THRESHOLD * g > b + r && g > GREEN_MIN_VALUE;
}

#define BLUE_RATIO_THRESHOLD 1.7f
#define BLUE_MIN_VALUE 50

bool is_blue(uint8_t b, uint8_t g, uint8_t r) {
	return 1.0f / BLUE_RATIO_THRESHOLD * b > g + r && b > BLUE_MIN_VALUE;
}

Line::Line(int front_cam_id, std::shared_ptr<Robot> robot) : obstacle_active(0), running(false) {
	this->front_cam_id = front_cam_id;
	this->robot = robot;

	this->average_silver = cv::imread(RUNTIME_AVERAGE_SILVER_PATH);
	this->distance_weight_map = cv::imread(RUNTIME_DISTANCE_WEIGHT_MAP_PATH, cv::IMREAD_GRAYSCALE);
}

void Line::start() {
	// Start video feed from front camera
	robot->start_video(front_cam_id);

	running = true;

	std::thread obstacle_thread([this]{obstacle();});
	obstacle_thread.detach();
}

void Line::stop() {
	// Reset if running
	if(!running) return;

	robot->stop_video(front_cam_id);
	obstacle_active = 0;
	// Setting running to false will notify obstacle thread to stop
	running = false;
}

bool Line::check_silver(cv::Mat& frame) {
	cv::Mat a = frame(cv::Range(SILVER_Y), cv::Range(SILVER_X));

	// Calculate difference between frame cutout
	int rows = a.rows;
	int cols = a.cols;

	uint32_t total_difference = 0;

	uint8_t* p;
	uint8_t* p_b;

	int i, j;
	for(i = 0; i < rows; ++i) {
		p = a.ptr<uint8_t>(i);
		p_b = average_silver.ptr<uint8_t>(i);
		for(j = 0; j < cols; ++j) {
			total_difference += std::abs((int16_t)p[j + 0] - p_b[j + 0])
				+ std::abs((int16_t)p[j + 1] - p_b[j + 2])
				+ std::abs((int16_t)p[j + 1] - p_b[j + 2]);
		}
	}

	return total_difference < 25'000;
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
		if(robot->distance(DIST_1, 1) < 7.0f && robot->distance(DIST_1, 20) < 9.0f) {
			std::cout << "Obstacle" << std::endl;
			obstacle_active = 1;
		}
	}
}

bool Line::line(cv::Mat& frame) {
	// Check if obstacle thread has notified main thread
	if(obstacle_active == 1) {
		robot->beep(300, LED_1);
			
		robot->m(-80, -80, 150);

		robot->m(-80, 80, 270);
		robot->m(80, 80, 200);

		obstacle_active = 2;
		robot->stop_video(front_cam_id);
		robot->start_video(front_cam_id);
		delay(500);
	} else if(obstacle_active == 2) {
		if(!abort_obstacle(frame)) {
			robot->m(40, 15, 30);
			robot->m(50, -20, 30);
			delay(50);
		} else {
			robot->stop();
			robot->m(20, 20, 450);
			robot->m(-20, 20, 500);
			obstacle_active = 0;
		}
	} else {
#ifdef DEBUG
		debug_frame = frame.clone();
#endif

		cv::Mat black = in_range(frame, &is_black);

		follow(frame, black);

		rescue_kit(frame);

		uint8_t green_result = green(frame, black);

		switch(green_result) {
			default:
				break;
			case GREEN_RESULT_LEFT:
				std::cout << "LEFT" << std::endl;
#ifndef MOVEMENT_OFF
				// Stop video before doing anything that causes a delay in frame retreival
				robot->stop_video(front_cam_id);
				robot->m(50, 50, 150);
				robot->m(-50, 50, 300);
				delay(210);
				robot->m(50, 50, 100);
				robot->start_video(front_cam_id);
#endif
				break;
			case GREEN_RESULT_RIGHT:
				std::cout << "RIGHT" << std::endl;
#ifndef MOVEMENT_OFF
				robot->stop_video(front_cam_id);
				robot->m(50, 50, 150);
				robot->m(50, -50, 300);
				delay(210);
				robot->m(50, 50, 100);
				robot->start_video(front_cam_id);
#endif
				break;
			case GREEN_RESULT_DEAD_END:
				std::cout << "DEAD-END" << std::endl;
#ifndef MOVEMENT_OFF
				robot->stop_video(front_cam_id);
				robot->m(50, 50, 150);
				robot->m(50, -50, 600);
				delay(200);
				robot->m(50, 50, 150);
				robot->start_video(front_cam_id);
#endif
				break;
		}

		/*if(check_silver(frame)) {
			robot->stop();
			std::cout << "SILVER" << std::endl;
			robot->stop_video(front_cam_id);
			robot->m(60, -60, 600);
			delay(200);
			robot->m(20, 20, 300);
			robot->start_video(front_cam_id);
			delay(200);
			return true; // Return true when silver is detected
		}*/
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
				// Put point into array based on distance to center
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

	int16_t error = line_angle * FOLLOW_HORIZONTAL_SENSITIVITY;

#ifndef MOVEMENT_OFF
	robot->m(FOLLOW_MOTOR_SPEED + error, FOLLOW_MOTOR_SPEED - error);
#endif
}

// std::vector<cv::Point> Line::find_green_group_centers_old(cv::Mat frame, cv::Mat& green) {
// 	std::vector<cv::Point> groups;

// 	uint32_t num_pixels = 0;
// 	green = in_range(frame, &is_green, &num_pixels);
// #ifdef DEBUG
// 	cv::imshow("Green", green);
// #endif

// 	if(num_pixels < 50) return groups; // Save some time by not calculating contours

// 	std::vector<std::vector<cv::Point>> contours;
// 	std::vector<cv::Vec4i> hierarchy;
// 	cv::findContours(green, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

// 	for(int i = 0; i < contours.size(); i++) {
// 		cv::RotatedRect bounding_rect = cv::minAreaRect(contours[i]);

// 		float size = bounding_rect.size.width * bounding_rect.size.height;
// 		if(size > 100.0f) {
// 			groups.push_back(bounding_rect.center);
// 		}
// 	}

// 	return groups;
// }

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

uint8_t Line::green(cv::Mat& frame, cv::Mat& black) {
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
		if(groups[i].x < 8) return 0;
		if(groups[i].x > 80-8) return 0;
	}

	// Cut out part of the black matrix around the group centers
	for(int i = 0; i < groups.size(); ++i) {
		cv::Range x_range = cv::Range(groups[i].x - cut_width / 2, groups[i].x + cut_width / 2);
		if(x_range.start < 0) x_range.start = 0;
		if(x_range.end > black.size[1]) x_range.end = black.cols;

		cv::Range y_range = cv::Range(groups[i].y - cut_height / 2, groups[i].y + cut_height / 2);
		if(y_range.start < 0) y_range.start = 0;
		if(y_range.end > black.size[0]) y_range.end = black.rows;

		cv::Mat cut = black(y_range, x_range);
		cv::Mat green_cut = green(y_range, x_range); // TODO

		// Calculate average black pixel in the cut
		float average_x = 0.0f;
		float average_y = 0.0f;

		uint32_t num_pixels = 0;

		uint8_t* p;
		uint8_t* p_grn;
		int y, x;
		for(y = 0; y < cut.rows; ++y) {
			p = cut.ptr<uint8_t>(y);
			p_grn = green_cut.ptr<uint8_t>(y);
			for(x = 0; x < cut.cols; ++x) {
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
		if(average_y < cut.rows / 2) {
			// Only consider point if average is above
			green_mask |= average_x < cut.cols / 2 ? 0x02 : 0x01;
		}
	}
	// If a green point was to the bottom left of the corner/black line,
	// the first bit will be set. If a green point was to the bottom right of the
	// corner/black line, the second bit will be set.
	// left -> 1
	// right -> 2
	// both -> 3
	//std::cout << std::to_string(green_mask) << std::endl;
	return green_mask;
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

		robot->stop_video(front_cam_id);
		if(angle < 0) {
			robot->m(-60, 60, 300 * -angle);
		} else {
			robot->m(60, -60, 300 * angle);
		}
		std::cout << "Angle:    " << rad_to_deg(angle) << std::endl;
		std::cout << "Distance: " << distance << std::endl;
		stop();
		delay(500);

		if(distance < 24) {
			robot->m(-60, -60, 1.5f * (distance - 24));
		} else {
			robot->m(60, 60, 1.5f * (distance - 24));
		}

		delay(1000);

		robot->m(-60, -60, 420);
		delay(200);
		robot->m(100, -100, TURN_100_90 * 2);

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
		robot->m(-100, 100, TURN_100_90 * 2);
		delay(200);
		robot->m(60, 60, 200);
		delay(1000);
		robot->start_video(front_cam_id);
	}
}