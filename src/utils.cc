#include "utils.h"

#include <algorithm>

#include <cmath>

#include <opencv2/opencv.hpp>
#include <string>

float clip(float n, float lower, float upper) {
	return std::max(lower, std::min(n, upper));
}

float point_distance(cv::Point a, cv::Point b) {
	return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

float map(float s, float a1, float a2, float b1, float b2) {
	return b1 + (s - a1) * (b2 - b1) / (a2 - a1);
}

void draw_rotated_rect(cv::Mat out, cv::RotatedRect r, cv::Scalar color, int thickness) {
	cv::Point2f vertices[4];
	r.points(vertices);
	for(int i = 0; i < 4; i++) {
		cv::line(out, vertices[i], vertices[(i + 1) % 4], color, thickness);
	}
}

cv::Mat in_range(cv::Mat& in, std::function<bool (uint8_t, uint8_t, uint8_t)> f, uint32_t* num_pixels) {
	CV_Assert(in.channels() == 3);
	CV_Assert(in.depth() == CV_8U);

	uint32_t num_p = 0;

	int rows = in.rows;
	int cols = in.cols;

	cv::Mat out(rows, cols, CV_8UC1);

	int i, j;
	for(i = 0; i < rows; ++i) {
		cv::Vec3b* p = in.ptr<cv::Vec3b>(i);
		uint8_t* p_out = out.ptr<uint8_t>(i);
		for(j = 0; j < cols; j++) {
			if(f(p[j][0], p[j][1], p[j][2])) {
				p_out[j] = 0xFF;
				++num_p;
			} else {
				p_out[j] = 0x00;
			}
		}
	}
	if(num_pixels != nullptr) {
		*num_pixels = num_p;
	}
	return out;
}

void clipped_difference(cv::Mat a, cv::Mat b, cv::Mat out) {
	CV_Assert(a.depth() == CV_8U);
	CV_Assert(b.depth() == CV_8U);
	CV_Assert(a.cols == b.cols);
	CV_Assert(a.rows == b.rows);

	int i, j;
	for(i = 0; i < a.rows; ++i) {
		uint8_t* a_ptr = a.ptr<uint8_t>(i);
		uint8_t* b_ptr = b.ptr<uint8_t>(i);
		uint8_t* out_ptr = out.ptr<uint8_t>(i);
		for(j = 0; j < a.cols * a.channels(); j++) {
			int16_t diff = a_ptr[j] - b_ptr[j];
			if(diff < 0) diff = 0;
			out_ptr[j] = (uint8_t)diff;
		}
	}
}

float average_difference(cv::Mat a, cv::Mat b) {
	CV_Assert(a.depth() == CV_8U);
	CV_Assert(b.depth() == CV_8U);
	CV_Assert(a.cols == b.cols);
	CV_Assert(a.rows == b.rows);
	CV_Assert(a.channels() == b.channels());

	float difference = 0.0f;

	int i, j;
	for(i = 0; i < a.rows; ++i) {
		uint8_t* a_ptr = a.ptr<uint8_t>(i);
		uint8_t* b_ptr = b.ptr<uint8_t>(i);
		for(j = 0; j < a.cols * a.channels(); j++) {
			int16_t diff = a_ptr[j] - b_ptr[j];
			difference += std::abs(diff);
		}
	}
	return difference / (a.cols * a.rows * a.channels());
}

cv::Vec3b average_color(cv::Mat in) {
	CV_Assert(in.depth() == CV_8U);
	float total_b = 0;
	float total_g = 0;
	float total_r = 0;

	int i, j;
	for(i = 0; i < in.rows; ++i) {
		uint8_t* ptr = in.ptr<uint8_t>(i);
		for(j = 0; j < in.cols; ++j) {
			total_b += (float)ptr[j];
			total_g += (float)ptr[j + 1];
			total_r += (float)ptr[j + 2];
		}
	}
	float size = in.rows * in.cols;
	total_b /= size;
	total_g /= size;
	total_r /= size;

	return cv::Vec3b((uint8_t)total_b, (uint8_t)total_g, (uint8_t)total_r);
}

float average_circle_color(cv::Mat in, float center_x, float center_y, float radius) {
	CV_Assert(in.depth() == CV_8U);
	float total = 0.0f;

	float r2 = radius * radius;
	uint32_t num_circle_pixels = 0;

	int i, j;
	for(i = 0; i < in.rows; ++i) {
		float y = (float)i - center_y;
		float y2 = y * y;
		uint8_t* ptr = in.ptr<uint8_t>(i);
		//uint8_t* debug_ptr = debug.ptr<uint8_t>(i);
		for(j = 0; j < in.cols; ++j) {
			float x = (float)j - center_x;
			if(y2 + x*x > r2) {
				continue;
			}
			for(int k = 0; k < in.channels(); ++k) {
				total += (float)ptr[j*in.channels() + k];
			}
			++num_circle_pixels;
		}
	}
	total /= num_circle_pixels * in.channels();

	return total;
}

void save_img(const std::string& path, cv::Mat frame) {
	auto millisecondsUTC = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	cv::imwrite(path + std::to_string(millisecondsUTC) + ".png", frame);
}

uint32_t count_circle_in_range(cv::Mat in, float center_x, float center_y, float radius, std::function<bool (uint8_t, uint8_t, uint8_t)> f) {
	CV_Assert(in.depth() == CV_8U);
	uint8_t* ptr;

	float r2 = radius * radius;
	uint32_t num_circle_pixels_in_range = 0;

	int i, j;
	for(i = 0; i < in.rows; ++i) {
		float y = (float)i - center_y;
		float y2 = y * y;
		ptr = in.ptr<uint8_t>(i);
		//uint8_t* debug_ptr = debug.ptr<uint8_t>(i);
		for(j = 0; j < in.cols; ++j) {
			float x = (float)j - center_x;
			if(y2 + x*x > r2) {
				continue;
			}
			int k = j*in.channels();
			uint16_t sum = ptr[j*3] + ptr[j*3 + 1] + ptr[j*3 + 2];
			if(sum < 100) ++num_circle_pixels_in_range;
			//if(f(ptr[k + 0], ptr[k + 1], ptr[k + 2])) ++num_circle_pixels_in_range;
		}
	}

	return num_circle_pixels_in_range;
}

cv::Mat two_channel_to_three_channel(cv::Mat in) {
	cv::Mat out(in.rows, in.cols, CV_32FC3);
	for(int i = 0; i < in.rows; ++i) {
		float* p_in = in.ptr<float>(i);
		float* p_out = out.ptr<float>(i);
		for(int j = 0; j < in.cols; ++j) {
			p_out[j * 3] = p_in[j * 2];
			p_out[j * 3 + 1] = p_in[j * 2 + 1];
			p_out[j * 3 + 2] = 0.0f;
		}
	}
	return out;
}