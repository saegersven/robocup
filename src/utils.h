#pragma once
#include <opencv2/opencv.hpp>

float clip(float n, float lower, float upper);

float point_distance(cv::Point a, cv::Point b);

inline float deg_to_rad(float deg);

cv::Mat inRange_hue(cv::Mat& in);

uint8_t bgr_to_gray(cv::Vec3b in);
uint8_t bgr_to_gray(uint8_t b, uint8_t g, uint8_t r);

uint8_t bgr_to_hue(cv::Vec3b in);
uint8_t bgr_to_hue(uint8_t b, uint8_t g, uint8_t r);

bool pixel_count_over_threshold(cv::Mat& in, cv::Vec3b lower, cv::Vec3b upper, uint32_t num_pixels);

bool pixel_count_over_threshold_hue(cv::Mat& in, uint8_t lower, uint8_t upper, uint32_t num_pixels);