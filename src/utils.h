#pragma once

float clip(float n, float lower, float upper);

bool pixel_count_over_threshold(cv::Mat& in, cv::Scalar lower, cv::Scalar upper, uint32_t num_pixels);