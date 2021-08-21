#include "utils.h"

float clip(float n, float lower, float upper) {
	return std::max(lower, std::min(n, upper));
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

bool pixel_count_over_threshold(cv::Mat& in, cv::Vec3b lower, cv::Vec3b upper, uint32_t num_pixels) {
	CV_Assert(in.depth() == cv::CV_8U);
	CV_Assert(in.channels() == 3); // TODO: Support for more channels

	int rows = in.rows;
	int cols = in.cols * 3;

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
			if(p[j][0] >= lower[0] && p[j][0] <= upper[0] &&
				p[j][1] >= lower[1] && p[j][1] <= upper[1] &&
				p[j][2] >= lower[2] && p[j][2] <= upper[2]) {
				++counter;
				if(counter == num_pixels) return true;
			}
		}
	}

	return false;
}

bool pixel_count_over_threshold_hue(cv::Mat& in, uint8_t lower, uint8_t upper, uint8_t min_value, uint32_t num_pixels) {
	CV_Assert(in.depth() == cv::CV_8U);
	CV_Assert(in.channels() == 3); // TODO: Support for more channels

	int rows = in.rows;
	int cols = in.cols * 3;

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