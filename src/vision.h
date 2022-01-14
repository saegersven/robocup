#pragma once

#include <opencv2/opencv.hpp>

// Filename macro for camera calibration files
#define CALIBRATION_FILE_NAME(id) "cc_" + std::to_string(id) + ".bin"

struct Camera {
private:
	cv::Mat undistort(cv::Mat in);

public:
	const std::string& hardware_id;
	cv::VideoCapture cap;
	cv::Size image_size;
	int fps;
	bool calibrated = false;
	cv::Mat camera_matrix;
	cv::Mat new_camera_matrix;
	cv::Mat distortion_matrix;

	bool has_subtractive_mask = false;
	cv::Mat subtractive_mask;

	Camera(const std::string& hardware_id, bool calibrated, int width, int height, int fps);

	void calibration_from_file(const std::string& file_name);
	void load_subtractive_mask(const std::string& file_name);
	void open_video();
	void stop_video();
	cv::Mat retrieve_video_frame(bool undistort = false);
	cv::Mat single_capture(bool undistort = false);
};

struct CameraPropertiesSerialized {
	float fx, fy, cx, cy;
	float k1, k2, p1, p2, k3;
};