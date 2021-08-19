#pragma once
#include <opencv2/opencv.hpp>
#include "errcodes.hpp"

struct Camera {
	int hardware_id;
	cv::VideoCapture cap;
	cv::Size image_size;
	int fps;
	bool calibrated = false;
	cv::Mat camera_matrix;
	cv::Mat new_camera_matrix;
	cv::Mat distortion_matrix;

	Camera(int hardware_id, bool calibrated, int width, int height, int fps);

	void calibration_from_file(std::string& file_name);
	void open_video();
	void stop_video();
	cv::Mat retrieve_video_frame(bool undistort = false);
	cv::Mat single_capture(bool undistort = false);
}

struct CameraPropertiesSerialized {
	float fx, fy, cx, cy;
	float k1, k2, p1, p2, k3;
}

cv::Mat undistort(cv::Mat in, CameraMatrix cam, DistortionMatrix dist);