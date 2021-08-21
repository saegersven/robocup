#include "utils.h"

float clip(float n, float lower, float upper) {
	return std::max(lower, std::min(n, upper));
}

bool pixel_count_over_threshold(cv::Mat& in, cv::Scalar lower, cv::Scalar upper, uint32_t num_pixels) {
	CV_Assert(in.depth() == cv::CV_32F);
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
	float* p;
	for(i = 0; i < rows; ++i) {
		p = in.ptr<float>(i);
		for(j = 0; j < cols; ++j) {
			if(p[j][0] >= lower[0] && p[j][0] <= upper[0] &&
				p[j][1] >= lower[1] && p[j][1] <= upper[1] &&
				p[j][2] >= lower[2] && p[j][2] <= upper[2]) {
				++counter;
				if(counter == num_pixels) return true;
			}
		}
	}

	/*
	// SAFE WAY
	cv::MatIterator_<Vec3f> it, end;
	for(it = in.begin<Vec3f>(), end = in.end<Vec3f>(); it != end; ++it) {
		if((*it)[0] >= lower[0] && (*it)[0] <= upper[0] &&
			(*it)[1] >= lower[1] && (*it)[1] <= upper[1] &&
			(*it)[2] >= lower[2] && (*it)[2] <= upper[2]) {
			++counter;
			if(counter == num_pixels) return true;
		}
	}
	*/

	return false;
}