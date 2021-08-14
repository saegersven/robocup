#include <opencv2/opencv.hpp>
#include "errcodes.hpp"

struct CameraProperties {
	cv::VideoCapture cap;
	cv::Size image_size;
	bool calibrated = false;
	cv::Mat camera_matrix;
	cv::Mat new_camera_matrix;
	cv::Mat distortion_matrix;

	void calibration_from_file(std::string& file_name);
}

cv::Mat undistort(cv::Mat in, CameraMatrix cam, DistortionMatrix dist);