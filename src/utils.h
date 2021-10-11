#pragma once

#include <opencv2/opencv.hpp>

#define delay(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

float clip(float n, float lower, float upper);

float point_distance(cv::Point a, cv::Point b);

inline float deg_to_rad(float deg) {
	return deg * 0.01745329251f;
}

bool detect_primary_color(uint8_t b, uint8_t g, uint8_t r, uint8_t channel_index, float min_ratio, uint8_t min_value);
bool detect_primary_color(uint8_t c, uint8_t c1, uint8_t c2, float min_ratio, uint8_t min_value);

cv::Mat in_range_hue(cv::Mat& in);

cv::Mat in_range_primary_color(cv::Mat& in, uint8_t channel_index, float min_ratio, uint8_t min_value);

uint8_t bgr_to_lightness(uint8_t b, uint8_t g, uint8_t r);
uint8_t bgr_to_lightness(cv::Vec3b in);

uint8_t bgr_to_hue(cv::Vec3b in);
uint8_t bgr_to_hue(uint8_t b, uint8_t g, uint8_t r, uint8_t& value);

bool pixel_count_over_threshold(cv::Mat& in, cv::Vec3b lower, cv::Vec3b upper, uint32_t num_pixels);

bool pixel_count_over_threshold_hue(cv::Mat& in, uint8_t lower, uint8_t upper, uint32_t num_pixels);

bool pixel_count_over_threshold_primary_color(cv::Mat& in, uint8_t channel, float min_ratio, uint8_t min_value, uint32_t num_pixels);

float map(float s, float a1, float a2, float b1, float b2);

void draw_rotated_rect(cv::Mat out, cv::RotatedRect r, cv::Scalar color, int thickness);