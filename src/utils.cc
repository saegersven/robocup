#include "utils.h"

float clip(float n, float lower, float upper) {
	return std::max(lower, std::min(n, upper));
}

bool pixel_count_over_threshold(cv::Mat& in, cv::Scalar lower, cv::Scalar upper, uint32_t num_pixels) {
	float* in_ptr = in.ptr<float>(0);

	uint32_t counter = 0;

	// Assuming three channels
	for(uint32_t i = 0; i < (in.cols * in.rows); i++) {
		if(in_ptr[i * 3 + 0] >= lower[0] && in_ptr[i * 3 + 0] <= upper[0] &&
			in_ptr[i * 3 + 1] >= lower[1] && in_ptr[i * 3 + 1] <= upper[1] &&
			in_ptr[i * 3 + 2] >= lower[2] && in_ptr[i * 3 + 2] <= upper[2]) {
			counter++;
			if(counter == num_pixels) return true;
		}
	}
	return false;
}