#include "line.h"

#include <algorithm>
#include <vector>

#include <opencv2/opencv.hpp>

#include "robot.h"
#include "utils.h"
//#include "neural_networks.h"

Line::Line(int front_cam_id, std::shared_ptr<Robot> robot) {
	this->front_cam_id = front_cam_id;
	this->robot = robot;
	//this->neural_networks = neural_networks;

	this->average_silver = cv::imread(RUNTIME_AVERAGE_SILVER_PATH);
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

	return total_difference < 20'000;
}

void Line::line(cv::Mat& frame) {
	if(check_silver(frame)) {
		robot->stop();
		std::cout << "DETECTED SILVER" << std::endl;
		robot->m(60, -60, 600);
		delay(500);
		robot->m(20, 20, 300);
		robot->stop_video(front_cam_id);
		robot->start_video(front_cam_id);
		delay(500);
	} else {
		follow(frame);
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

bool Line::is_black(uint8_t b, uint8_t g, uint8_t r) {
	float lightness = ((float)b + g + r) / 3.0f;
	//float saturation = std::max(b, std::max(g, r)) - std::min(b, std::min(g, r));
	return lightness < 100.0f;
}

float Line::line_weight(float distance) {
	distance = std::abs(distance);
	return 0.85f * std::pow(50, -std::pow((distance - 0.6f), 2)) + 0.25f * distance;
	//return 0.7f * std::pow(50, -std::pow((distance - 0.6f), 2)) + 0.5f * distance;
	//return 1.0f;
	//return 3*distance*distance - 2*distance*distance*distance;
}

float Line::motor_weight(float line_pos) {
	//return std::pow(2, -10.0f * line_pos * line_pos);
	return 1.0f;
}

float Line::pixel_weight(float distance) {
	return 0.2f + 0.8f * std::pow(2, -std::pow(distance * 8, 2));
}

float Line::average_black(cv::Mat in, uint32_t& num_pixels) {
	float v = 0.0f;
	num_pixels = 0;

	float average_pixel_weight = 1.0f;

	int rows = in.rows;
	int cols = in.cols;

	cv::Vec3b* p;
	int i, j;
	for(i = 0; i < rows; ++i) {
		p = in.ptr<cv::Vec3b>(i);
		for(j = 0; j < cols; ++j) {
			if(is_black(p[j][0], p[j][1], p[j][2])) {
				++num_pixels;
				float dist_to_center = (float)j / in.cols * 2.0f - 1.0f;
				float dist_to_last = dist_to_center - last_line_pos;
				float p_weight = pixel_weight(dist_to_last);
				v += p_weight * dist_to_center;
				average_pixel_weight += p_weight;
			}
		}
	}
	average_pixel_weight /= (float)num_pixels;
	v /= (float)num_pixels;
	v /= average_pixel_weight;
	return v;
}

cv::Mat Line::in_range_black(cv::Mat& in) {
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
			p_out[j] = ((float)p[j][0] + p[j][1] + p[j][2]) / 3 < 100 ? 0xFF : 0x00;
		}
	}
	return out;
}

void Line::follow(cv::Mat& frame) {
	auto start_time = std::chrono::high_resolution_clock::now();
	cv::Mat debug = frame.clone();
	//std::cout << "Follow" << std::endl;

	int16_t line_angle = 0;

	float line_pos = 0.0f;
	float line_pos_weighted = 0.0f;

	uint8_t line_search_offset = 0;

	bool line_found = false;

	while(!line_found && (line_search_offset + BLACK_Y_BOTTOM_OFFSET < frame.size[0])) {
		cv::Mat black_cut = frame(
			cv::Range(BLACK_Y_TOP_OFFSET + line_search_offset, BLACK_Y_BOTTOM_OFFSET + line_search_offset),
			cv::Range(0, frame.cols)
			);

		uint32_t num_pixels = 0;
		line_pos = average_black(black_cut, num_pixels);

		if(num_pixels > 30) {
			line_found = true;

			cv::line(debug, cv::Point((line_pos + 1.0f) * frame.size[1] / 2, line_search_offset + BLACK_Y_TOP_OFFSET),
				cv::Point((line_pos + 1.0f) * frame.size[1] / 2, line_search_offset + BLACK_Y_BOTTOM_OFFSET),
				cv::Scalar(0, 0, 255),
				2);

			line_pos_weighted = line_weight(line_pos) * line_pos;
			last_line_pos = line_pos;
			std::cout << std::to_string(line_pos_weighted) << "\t";
			std::cout << std::to_string(line_pos) << std::endl;
		}

		/*cv::Mat black_out = in_range_black(black_cut);
		cv::imshow("Black out", black_out);
		
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(black_out, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

		if(contours.size() > 0) {
			int distance = 128;
			cv::RotatedRect rect;
			int index = -1;
			for(int i = 0; i < contours.size(); i++) {
				cv::RotatedRect r = cv::minAreaRect(contours[i]);
				int d = last_line_pos + frame.size[1] - r.center.x;
				float size = r.size.width * r.size.height;
				if(d < distance && size > 20.0f) {
					index = i;
					rect = r;
					distance = d;
				}
			}

			if(index != -1) {
				line_found = true;
				// rect is now the minarearect of the contour with the largest area
				draw_rotated_rect(debug, rect, cv::Scalar(255, 0, 0), 2);

				line_angle = (int16_t)rect.angle;

				// Normalize angle between -90° and 90°
				while(line_angle > 45) {
					line_angle -= 90;
				}

				while(line_angle < -45) {
					line_angle += 90;
				}

				line_pos = (int8_t)(rect.center.x - frame.size[1] / 2);
				last_line_pos = line_pos;
			}
		}*/
		line_search_offset += 2;
	}

	//std::cout << "Pos:\t" << std::to_string(line_pos) << "\tAngle:\t" << std::to_string(line_angle) << "\r" << std::endl;

	cv::imshow("Debug", debug);
	cv::waitKey(1);

	//int16_t error = line_pos * FOLLOW_HORIZONTAL_SENSITIVITY + line_angle * FOLLOW_ANGLE_SENSITIVITY;

	int16_t error = line_pos_weighted * FOLLOW_HORIZONTAL_SENSITIVITY;
	float mw = motor_weight(line_pos_weighted);

	robot->m(FOLLOW_MOTOR_SPEED * mw + error, FOLLOW_MOTOR_SPEED * mw - error);

	std::cout << std::to_string(error) << std::endl;

	auto end_time = std::chrono::high_resolution_clock::now();
	uint16_t us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
	//std::cout << "Took: " << std::to_string(us) << "μs" << std::endl;
}

// TODO: Find better values
uint8_t Line::green(cv::Mat& frame) {
	return 0;
	/*uint8_t green_points = 0;

	cv::Mat green = in_range_primary_color(frame, 1, 0.85f, 40);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(green, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	for(int i = 0; i < contours.size(); ++i) {
		cv::RotatedRect r = cv::minAreaRect(contours[i]);
		// Only consider contour if it is big enough
		if(r.size.width * r.size.height < 40.0f) continue;

		float initial_angle = deg_to_rad(r.angle);
		if(initial_angle > deg_to_rad(45.0f)) initial_angle -= deg_to_rad(360.0f);
		float angle = initial_angle;

		std::vector<cv::Point2f> black_points;
		black_points.reserve(30);

		const int res = 30;
		const float step = deg_to_rad(360.0f / res);

		float sin, cos, cx, cy, step_out_x, step_out_y;
		for(int i = 0; i < res; ++i) {
			angle = deg_to_rad(step * i);

			sin = std::sin(angle);
			cos = std::cos(angle);

			cx = r.center.x + sin * (r.size.width * 0.75f);
			cy = r.center.y - cos * (r.size.height * 0.75f);

			step_out_x = sin * 0.5f;
			step_out_y = cos * 0.5f;

			for(int y = 0; y < 4; ++y) {
				cx += step_out_x;
				cy -= step_out_y;

				if(is_black(frame, (uint8_t)cx, (uint8_t)cy)) {
					black_points.push_back(cv::Point2f(cx, cy));
					break;
				}
			}
		}

		if(black_points.size() > 8) {
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
	return green_points;*/
}