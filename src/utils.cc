#include "utils.h"

#include <algorithm>

#include <cmath>

#include <opencv2/opencv.hpp>

float clip(float n, float lower, float upper) {
	return std::max(lower, std::min(n, upper));
}

float point_distance(cv::Point a, cv::Point b) {
	return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

inline float deg_to_rad(float deg) {
	return deg * 0.01745329251f;
}

uint8_t bgr_to_lightness(cv::Vec3b in) {
	return bgr_to_lightness(in[0], in[1], in[2]);
}

uint8_t bgr_to_lightness(uint8_t b, uint8_t g, uint8_t r) {
	return (b + g + r) / 3;
}

uint8_t bgr_to_hue(cv::Vec3b in) {
	return bgr_to_hue(in[0], in[1], in[2]);
}

uint8_t bgr_to_hue(uint8_t b, uint8_t g, uint8_t r, uint8_t& value) {
	float c_max = (float)std::max(b, g, r);
	float c_min = (float)std::min(b, g, r);
	float delta = c_max - c_min;

	value = c_max;

	// If delta is 0, the color is a shade of gray
	if(delta == 0.0f) return (uint8_t) 0;

	if(c_max == r)
		return (uint8_t)(42.5f * ((g - b) / delta));
	if(c_max == g)
		return (uint8_t)(42.5f * ((b - r) / delta + 2.0f));
	return (uint8_t)(42.5f * ((r - g) / delta + 4.0f));
}

bool detect_primary_color(uint8_t b, uint8_t g, uint8_t r, uint8_t channel_index, float min_ratio, uint8_t min_value) {
	switch(channel_index) {
		case 0:
			return detect_primary_color(b, g, r, min_ratio, min_value);
		case 1:
			return detect_primary_color(g, b, r, min_ratio, min_value);
		case 2:
			return detect_primary_color(r, b, g, min_ratio, min_value);
	}
}

bool detect_primary_color(uint8_t c, uint8_t c1, uint8_t c2, float min_ratio, uint8_t min_value) {
	float sum = c1 + c2;
	if(sum == 0.0f) return false;
	ratio = (float)c / sum;

	return ratio > min_ratio && c > min_value;
}

cv::Mat in_range_primary_color(cv::Mat& in, uint8_t channel_index, float min_ratio, uint8_t min_value) {
	cv::CV_Assert(in.channels() == 3);
	cv::CV_Assert(in.depth() == cv::CV_8U);

	int rows = in.rows;
	int cols = in.cols;

	uint8_t* p;
	cv::Mat out(rows, cols, cv::CV_8UC1);

	int i, j;
	for(i = 0; i < rows; ++i) {
		p = in.ptr<uint8_t>(i);
		p_out = out.ptr<uint8_t>(i);
		for(j = 0; j < cols; ++j) {
			p_out[j] = detect_primary_color(p[j][0], p[j][1], p[j][2], channel_index, min_ratio, min_value) ? 0xFF : 0x00;
		}
	}
	return out;
}

cv::Mat in_range_hue(cv::Mat& in, uint8_t lower, uint8_t upper, uint8_t min_value) {
	int rows = in.rows;
	int cols = in.cols;

	uint8_t* p;

	cv::Mat out(rows, cols, cv::CV_8UC1);

	int i, j;
	for(i = 0; i < rows; ++i) {
		p = in.ptr<uint8_t>(i);
		p_out = out.ptr<uint8_t>(i);
		for(j = 0; j < cols; ++j) {
			uint8_t value;
			uint8_t hue = bgr_to_hue(p[j * 3], p[j * 3 + 1], p[j * 3 + 2], value);
			if(value >= min_value && hue >= lower && hue <= upper) {
				p_out[j] = 0x01;
			} else {
				p_out[j] = 0x00;
			}
		}
	}

	return out;
}

bool pixel_count_over_threshold(cv::Mat& in, cv::Vec3b lower, cv::Vec3b upper, uint32_t num_pixels) {
	cv::CV_Assert(in.depth() == cv::CV_8U);
	cv::CV_Assert(in.channels() == 3); // TODO: Support for more channels

	int rows = in.rows;
	int cols = in.cols;

	// If the matrix is continuous, we don't have to get a new pointer for
	// every row. Treating the matrix as having one row is faster.
	if(in.isContinuous()) {
		cols *= rows;
		rows = 1;
	}

	uint32_t counter = 0;

	int i, j;
	uint8_t* p;
	for(i = 0; i < rows; ++i) {
		p = in.ptr<uint8_t>(i);
		for(j = 0; j < cols; ++j) {
			if(p[j * 3][0] >= lower[0] && p[j * 3][0] <= upper[0] &&
				p[j * 3][1] >= lower[1] && p[j * 3][1] <= upper[1] &&
				p[j * 3][2] >= lower[2] && p[j * 3][2] <= upper[2]) {
				++counter;
				if(counter == num_pixels) return true;
			}
		}
	}

	return false;
}

bool pixel_count_over_threshold_hue(cv::Mat& in, uint8_t lower, uint8_t upper, uint8_t min_value, uint32_t num_pixels) {
	cv::CV_Assert(in.depth() == cv::CV_8U);
	cv::CV_Assert(in.channels() == 3);

	int rows = in.rows;
	int cols = in.cols;

	// If the matrix is continuous, we don't have to get a new pointer for
	// every row. Treating the matrix as having one row is faster.
	if(in.isContinuous()) {
		cols *= rows;
		rows = 1;
	}

	uint32_t counter = 0;

	int i, j;
	uint8_t* p;
	for(i = 0; i < rows; ++i) {
		p = in.ptr<uint8_t>(i);
		for(j = 0; j < cols; ++j) {
			uint8_t value;
			uint8_t hue = bgr_to_hue(p[j * 3], p[j * 3 + 1], p[j * 3 + 2], value);
			if(value >= min_value && hue >= lower && hue <= upper) {
				++counter;
				if(counter == num_pixels) return true;
			}
		}
	}

	return false;
}

bool pixel_count_over_threshold_primary_color(cv::Mat& in, uint8_t channel, float min_ratio, uint8_t min_value, uint32_t num_pixels) {
	cv::CV_Assert(in.depth() == cv::CV_8U);
	cv::CV_Assert(in.channels() == 3);

	int rows = in.rows;
	int cols = in.cols;

	// If the matrix is continuous, we don't have to get a new pointer for
	// every row. Treating the matrix as having one row is faster.
	if(in.isContinuous()) {
		cols *= rows;
		rows = 1;
	}

	uint32_t counter = 0;

	int i, j;
	uint8_t* p;
	for(i = 0; i < rows; ++i) {
		p = in.ptr<uint8_t>(i);
		for(j = 0; j < cols; ++j) {
			if(detect_primary_color(b, g, r, channel, min_ratio, min_value)) {
				++counter;
				if(counter == num_pixels) return true;
			}
		}
	}

	return false;
}

float map(float s, float a1, float a2, float b1, float b2) {
	return b1 + (s - a1) * (b2 - b1) / (a2 - a1);
}