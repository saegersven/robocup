#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#define PI (double) 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679 // should be accurate enough XD
#define RAD_360 (2.0 * PI)
#define RAD_180 PI
#define RAD_90 (0.5 * PI)
#define RAD_45 (0.25 * PI)

#define delay(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

float clip(float n, float lower, float upper);

float point_distance(cv::Point a, cv::Point b);

inline float deg_to_rad(float deg) {
	return deg * PI / 180.0f;
}

inline float rad_to_deg(float rad) {
	return rad / PI * 180.0f;
}

float map(float s, float a1, float a2, float b1, float b2);

void draw_rotated_rect(cv::Mat out, cv::RotatedRect r, cv::Scalar color, int thickness);

cv::Mat in_range(cv::Mat& in, std::function<bool (uint8_t, uint8_t, uint8_t)> f, uint32_t* num_pixels = nullptr);

void clipped_difference(cv::Mat a, cv::Mat b, cv::Mat out);
void save_img(std::string path, cv::Mat frame);

cv::Vec3b average_color(cv::Mat in);