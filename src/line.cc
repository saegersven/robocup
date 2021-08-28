#include "line.h"

#include <algorithm>
#include <vector>

#include <opencv2/opencv.hpp>

#include "robot.h"
#include "utils.h"
#include "neural_networks.h"

Line::Line(int front_cam_id, Robot* robot, NeuralNetworks neural_networks) {
	this->front_cam_id = front_cam_id;
	this->robot = robot;
	this->neural_networks = neural_networks;
}

void Line::start() {
	// Start video feed from front camera
	robot->start_video(front_cam_id);

	running = true;
}

void Line::stop() {
	// Stop video if running
	if(!running) return;

	robot->stop_video(front_cam_id);
	running = false;
}

void Line::line(cv::Mat& frame) {
	follow(frame);

	//cv::Mat frame = robot->capture(front_cam_id); // Retrieve video frame
	cv::Mat green_cut = frame(cv::Range(10, 70), cv::Range(8, 40));

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
	}
}

bool Line::is_black(cv::Mat& in, uint8_t x, uint8_t y) {
	cv::Vec3b pix = in.at<cv::Vec3b>(y, x);
	float lightness = ((float)pix[0] + (float)pix[1] + (float)pix[2]) / 3.0f;
	float saturation = std::max(pix[0], pix[1], pix[2]) - std::min(pix[0], pix[1], pix[2]);
	return lightness <= 20.0f && saturation <= 20.0f;
}

void Line::follow(cv::Mat& frame) {
	const int8_t check_y = 40;
	const int8_t padding = 10;
	const int8_t width = frame.cols - padding * 2;
	
	const uint8_t min_line_width = 2;

	uint8_t line_x = last_line_x;
	uint8_t line_check_left = last_line_x - 1;
	uint8_t line_check_right = last_line_x + 1;

	// 0 -> No line yet
	// 1 -> On line
	// 1 + line width -> No line again
	bool flag_left = 0;
	bool flag_right = 0;

	uint8_t line_left = 200;
	uint8_t line_right = 200;

	bool line_not_centered = false;

	bool first = true;
	while(true) {
		if(flag_left == 0 && is_black(frame, line_check_left, check_y)) {
			flag_left = 1;
			// If we haven't started on the line (line_not_centered),
			// both edges of the line will be found in the same direction.
			if(line_not_centered) line_right = line_check_left;
		}
		if(flag_right == 0 && is_black(frame, line_check_right, check_y)) {
			flag_right = 1;
			if(line_not_centered) line_left = line_check_right;
		}

		if(first && (flag_left == 0 || flag_right == 0)) line_not_centered = true;

		if(flag_left <= min_line_width && !is_black(frame, line_check_left, check_y)) {
			++flag_left;
			line_left = line_check_left;
		}
		if(flag_right <= 1 + min_line_width && !is_black(frame, line_check_right, check_y)) {
			++flag_right;
			line_right = line_check_right;
		}

		if(line_right - line_left >= min_line_width) {
			// Minimum line width is reached
			if(line_not_centered) {
				if((flag_right == 2 || flag_left == 2)) {
					break;
				}
			} else {
				if(flag_right == 2 && flag_left == 2) {
					break;
				}
			}
		}

		if(line_check_left == padding) {
			// We have reached the border. One of the line edges is not set, set it to the border.
			if(line_left == 200) line_left = padding;
			if(line_right == 200) line_right = frame.cols - padding;
			break;
		}

		--line_check_left;
		++line_check_right;

		first = false;
	}

	line_x = (uint8_t)((uint16_t)line_left + (uint16_t)line_right) >> 1;

	last_line_x = line_x;

	float angle = 0.0f;

	const float line_x_f = (float)line_x;
	const float radius = 15.0f; // in pixels
	const uint8_t res = 15;
	// We have the horizontal position of the line, now get the angle
	float angle_offset = 0.0f;
	float sin, cos, cx_l, cx_r, cy;
	for(uint8_t i = 0; i <= res; ++i) {
		sin = std::sin(angle_offset);
		cos = std::cos(angle_offset);

		cx_l = line_x_f + sin * radius;
		cx_r = line_x_f - sin * radius;

		cy = check_y + cos * radius;

		if(is_black(frame, (uint8_t)cx_l, (uint8_t)cy)) {
			angle = -angle_offset;
			break;
		} else if(is_black(frame, (uint8_t)cx_r, (uint8_t)cy)) {
			angle = angle_offset;
			break;
		}

		angle_offset += deg_to_rad(90.0f / res);
	}

	// Now we have the line x-position at the bottom and the angle of the line
	float error = (line_x_f - frame.cols / 2.0f) * FOLLOW_HORIZONTAL_SENSITIVITY;
	error += angle * FOLLOW_ANGLE_SENSITIVITY;

	// Finally set motor speed to follow line
	robot->m_asc(FOLLOW_MOTOR_SPEED + error, FOLLOW_MOTOR_SPEED - error);
}

// TODO: Find better values
uint8_t Line::green(cv::Mat& frame) {
	uint8_t green_points = 0;

	cv::Mat green = in_range_primary_color(frame, 1, 0.85f, 40);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(green, contours, cv::RETR_TREE, cv::CV_CHAIN_APPROX_SIMPLE);

	for(int i = 0; i < contours.size(); ++i) {
		cv::RotatedRect r = cv::minAreaRect(contours[i]);
		// Only consider contour if it is big enough
		if(r.size.width * r.size.height < 40.0f) continue;

		float initial_angle = deg_to_rad(r.angle);
		if(initial_angle > deg_to_rad(45.0f)) initial_angle -= deg_to_rad(360.0f);
		float angle = initial_angle;

		std::vector<cv::Point2f> black_points;
		black_points.reserve(30);

		for(int x = 0; x < 30; ++x) {
			angle = deg_to_rad(360.0f / 30.0f * x);

			float sin = std::sin(angle);
			float cos = std::cos(angle);

			float cx = r.center.x + sin * (r.size.width * 0.75f);
			float cy = r.center.y - cos * (r.size.height * 0.75f);

			for(int y = 0; y < 4; ++y) {
				cx += sin * y * 0.5f;
				cy -= cos * y * 0.5f;

				if(is_black(frame, (uint8_t)cx, (uint8_t)cy)) {
					black_points.push_back(cv::Point2f(cx, cy));
					break;
				}
			}
		}

		if(black_points.size() > 5) {
			float mean_x, mean_y;
			for(cv::Point2f p : black_points) {
				mean_x += p.x;
				mean_y += p.y;
			}

			mean_x /= black_points.size();
			mean_y /= black_points.size();

			float relative_black_angle = std::atan2(r.center.y - mean_y, r.center.x - mean_x);
			relative_black_angle -= initial_angle;

			if(relative_black_angle > 90.0f && relative_black_angle < 90.0f) {
				// If angle is positive, green is left
				green_points |= relative_black_angle > 0.0f ? 0x01 : 0x02;
			}
		}
	}
	// If both left and right green is there, return value is 3
	// If only left or right, return value is 1 or 2
	// If no contours were at the bottom, return value is 0
	return green_points;
}
