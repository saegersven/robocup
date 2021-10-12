#include "line.h"

#include <algorithm>
#include <vector>
#include <limits>
#include <cmath>
#include <future>

#include <opencv2/opencv.hpp>

#include "robot.h"
#include "utils.h"
//#include "neural_networks.h"

bool is_black(uint8_t b, uint8_t g, uint8_t r) {
	return (uint16_t)b + (uint16_t)g + (uint16_t)r < 300;
}

bool is_green(uint8_t b, uint8_t g, uint8_t r) {
	return false; // TODO
}

Line::Line(int front_cam_id, std::shared_ptr<Robot> robot) : obstacle_active(0), running(false) {
	this->front_cam_id = front_cam_id;
	this->robot = robot;
	//this->neural_networks = neural_networks;

	this->average_silver = cv::imread(RUNTIME_AVERAGE_SILVER_PATH);
}

void Line::start() {
	// Start video feed from front camera
	robot->start_video(front_cam_id);

	running = true;

	std::cout << "Begin line" << std::endl;
	std::thread obstacle_thread([this]{obstacle();});
	obstacle_thread.detach();
	std::cout << "Obstacle thread started" << std::endl;
}

void Line::stop() {
	// Reset if running
	if(!running) return;

	robot->stop_video(front_cam_id);
	obstacle_active = 0;
	// Setting running to false should notify obstacle thread
	running = false;
}

bool Line::check_silver(cv::Mat& frame) {
	cv::Mat a = frame(cv::Range(SILVER_Y), cv::Range(SILVER_X));

	//cv::rectangle(frame, cv::Rect(30, 36, 27, 54), cv::Scalar(0, 255, 0), 2);

	//cv::Mat a_resized;
	//cv::resize(a, a_resized, cv::Size(),10.0, 10.0);
	//cv::imshow("Silver Cut", a_resized);
	//cv::imwrite(RUNTIME_AVERAGE_SILVER_PATH, a);

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
				+ std::abs((int16_t)p[j + 1] - p_b[j + 2]) * 2;
		}
	}

	//std::cout << std::to_string(total_difference) << std::endl;

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
		if(robot->distance(DIST_1, 1) < 7.0f) {
			if(robot->distance(DIST_1, 20) < 9.0f) {
				std::cout << "Obstacle" << std::endl;
				obstacle_active = 1;
				/*robot->beep(300, LED_1);
				
				robot->m(-80, -80, 150);

				robot->m(-80, 80, 270);
				robot->m(80, 80, 200);

				obstacle_active = true;
				robot->stop_video(front_cam_id);
				robot->start_video(front_cam_id);
				delay(500);*/
			}
		}
	}
}

void Line::line(cv::Mat& frame) {
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
		//std::async(std::launch::async, [this]{ obstacle(); });
		cv::Mat black = in_range(frame, &is_black);

		follow(frame, black);

		green(frame, black);

		if(check_silver(frame)) {
			robot->stop();
			std::cout << "DETECTED SILVER" << std::endl;
			robot->m(60, -60, 600);
			delay(200);
			robot->m(20, 20, 300);
			robot->stop_video(front_cam_id);
			robot->start_video(front_cam_id);
			delay(200);
		}
	}

	cv::imshow("Frame", frame);
	cv::waitKey(1);

	//cv::Mat frame = robot->capture(front_cam_id); // Retrieve video frame
	/*cv::Mat green_cut = frame(cv::Range(10, 70), cv::Range(8, 40));

	// TODO: Green color values and threshold
	if(pixel_count_over_threshold_primary_color(green_cut, 1, 0.85f, 40, 1000)) {
		//float confidence = 0.0f;
		//switch(neural_networks.infere(GREEN_NN_ID, frame, confidence)) {
		switch(green(frame)) {
			case GREEN_RESULT_LEFT:
				// TODO: Go left
				std::cout << "Left" << std::endl;
				break;
			case GREEN_RESULT_RIGHT:
				// TODO: Go right
				std::cout << "Right" << std::endl;
				break;
			case GREEN_RESULT_DEAD_END:
				// TODO: Turn around
				std::cout << "Dead End" << std::endl;
				break;
		}
	}*/
}

/*cv::Mat Line::in_range_black(cv::Mat& in) {
	CV_Assert(in.channels() == 3);
	CV_Assert(in.depth() == CV_8U);

	int rows = in.rows;
	int cols = in.cols;

	cv::Vec3b* p;
	uint8_t* p_out;
	cv::Mat out(rows, cols, CV_8UC1);

	int i, j;
	for(i = 0; i < rows; ++i) {
		p = in.ptr<cv::Vec3b>(i);
		p_out = out.ptr<uint8_t>(i);
		for(j = 0; j < cols; j++) {
			p_out[j] = is_black(p[j][0], p[j][1], p[j][2]) ? 0xFF : 0x00;
		}
	}
	return out;
}*/

float Line::difference_weight(float x) {
	return 0.25f + 0.75f * std::pow(2, -std::pow(x * 5, 2));
}

float Line::distance_weight(float x) {
	return std::pow(2, -std::pow(((x - 0.65) * 4), 2));
}

float Line::circular_line(cv::Mat& in) {
	float average_line_angle;
	std::vector<std::pair<float, float>> line_angles; // Map of angle and distance

	uint8_t* p;

	cv::Point2f center(in.cols / 2, in.rows); // Bottom center
	float center_x = in.cols / 2.0f;
	float center_y = in.rows;

	int i, j;
	for(i = 0; i < in.rows; ++i) {
		p = in.ptr<uint8_t>(i);
		for(j = 0; j < in.cols; ++j) {
			if(p[j] != 0) {
				// Put point into array based on distance to center
				float x = (float)j;
				float y = (float)i;
				float distance = std::sqrt(std::pow(y - center_y, 2) + std::pow(x - center_x, 2));
				if(distance > MINIMUM_DISTANCE && distance < MAXIMUM_DISTANCE) {
					float angle = std::atan2(y - center_y, x - center_x) + (PI / 2.0f);
					line_angles.push_back(std::pair<float, float>(angle, distance));
				}
			}
		}
	}

	if(line_angles.size() < 40) return 0.0f;

	float average_difference_weight = 0.0f;
	for(std::pair<float, float> angle : line_angles) {
		float angle_difference_weight = difference_weight((angle.first - last_line_angle) / PI * 2.0f);
		float angle_distance_weight = distance_weight(map(angle.second, MINIMUM_DISTANCE, MAXIMUM_DISTANCE, 0.0f, 1.0f));

		average_line_angle += angle_difference_weight * angle_distance_weight * angle.first;
		average_difference_weight += angle_difference_weight;
	}

	average_line_angle /= (float)line_angles.size();
	average_line_angle /= average_difference_weight / (float)line_angles.size();

	return average_line_angle;
}

void Line::follow(cv::Mat& frame, cv::Mat black) {
	auto start_time = std::chrono::high_resolution_clock::now();
#ifdef DEBUG
	cv::Mat debug = frame.clone();
#endif
	//std::cout << "Follow" << std::endl;

	//cv::Mat black = in_range_black(frame);

	float line_angle = circular_line(black);

	if(std::isnan(line_angle)) line_angle = 0.0f;

	last_line_angle = line_angle;

#ifdef DEBUG
	cv::Point center(debug.cols / 2, debug.rows);

	cv::circle(debug, center, MINIMUM_DISTANCE, cv::Scalar(0, 255, 0), 2);
	cv::circle(debug, center, MAXIMUM_DISTANCE, cv::Scalar(0, 255, 0), 2);

	cv::line(debug,
		cv::Point(std::sin(line_angle) * MINIMUM_DISTANCE, -std::cos(line_angle) * MINIMUM_DISTANCE) + center,
		cv::Point(std::sin(line_angle) * MAXIMUM_DISTANCE, -std::cos(line_angle) * MAXIMUM_DISTANCE) + center,
		cv::Scalar(0, 255, 0), 2
		);

	//std::cout << "Pos:\t" << std::to_string(line_pos) << "\tAngle:\t" << std::to_string(line_angle) << "\r" << std::endl;

	cv::imshow("Debug", debug);
	cv::waitKey(1);
#endif

	//int16_t error = line_pos * FOLLOW_HORIZONTAL_SENSITIVITY + line_angle * FOLLOW_ANGLE_SENSITIVITY;

	int16_t error = line_angle * FOLLOW_HORIZONTAL_SENSITIVITY;

	robot->m(FOLLOW_MOTOR_SPEED + error, FOLLOW_MOTOR_SPEED - error);

	//std::cout << std::to_string(error) << std::endl;

	auto end_time = std::chrono::high_resolution_clock::now();
	uint16_t us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
	//std::cout << "Took: " << std::to_string(us) << "μs" << std::endl;
}

std::vector<cv::Point> Line::find_green_group_centers(cv::Mat frame, cv::Mat& green) {
	std::vector<cv::Point> groups;

	green = in_range(frame, &is_green);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(green, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	for(int i = 0; i < contours.size(); i++) {
		cv::RotatedRect bounding_rect = cv::minAreaRect(contours[i]);

		float size = bounding_rect.size.width * bounding_rect.size.height;
		if(size > 100.0f) {
			groups.push_back(bounding_rect.center);
		}
	}

	return groups;
}

uint8_t Line::green(cv::Mat& frame, cv::Mat& black) {
	uint8_t green_mask = 0;

	cv::Mat green;
	std::vector<cv::Point> groups = find_green_group_centers(frame, green);

	const int cut_width = 15;
	const int cut_height = 15;

	// Cut out part of the black matrix around the group centers
	for(int i = 0; i < groups.size(); ++i) {
		cv::Range x_range = cv::Range(groups[i].x - cut_width / 2, groups[i].x + cut_width / 2);
		if(x_range.start < 0) x_range.start = 0;
		if(x_range.end > black.size[0]) x_range.end = black.cols;

		cv::Range y_range = cv::Range(groups[i].y - cut_height / 2, groups[i].y + cut_height / 2);
		if(y_range.start < 0) y_range.start = 0;
		if(y_range.end > black.size[1]) y_range.end = black.rows;

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
			green_mask |= average_x < cut.cols / 2 ? 0x01 : 0x02;
		}
	}
	// If a green point was to the bottom left of the corner/black line,
	// the first bit will be set. If a green point was to the bottom right of the
	// corner/black line, the second bit will be set.
	// left -> 1
	// right -> 2
	// both -> 3
	return green_mask;
}