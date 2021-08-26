#include "vision.h"

#include <opencv2/opencv.hpp>

#include "errcodes.hpp"

Camera::Camera(int hardware_id, bool calibrated, int width, int height, int fps) {
	this->hardware_id = hardware_id;
	this->fps = fps;
	this->image_size = cv::Size(width, height);

	if(calibrated) {
		calibration_from_file(CALIBRATION_FILE_NAME(id));
		this->new_camera_matrix = cv::getOptimalNewCameraMatrix(
			this->camera_matrix, this->distortion_matrix, this->image_size, 1.0);
		this->calibrated = true;
	}
}

void Camera::calibration_from_file(std::string& file_name) {
	FILE* file = fopen(file_name.c_str(), "rb");
	if(!file) {
		std::cout << "Could not open camera calibration file '" <<
			file_name << "'" << std::endl;
		exit(ERRCODE_FILE_OPEN);
	}

	CameraPropertiesSerialized c;
	fread(&c, sizeof(CameraPropertiesSerialized), 1, file);

	this->camera_matrix = cv::Mat(3, 3, cv::CV_32F, {
		c.fx, 0.0f, 0.0f,
		0.0f, c.fy, 0.0f,
		c.cx, c.cy, 0.0f
	});

	this->distortion_matrix = cv::Mat(1, 5, cv::CV_32F, {
		c.k1, c.k2, c.p1, c.p2, c.k3
	});

	fclose(file);
}

void Camera::open_video() {
	this->cap.open(hardware_id);
	this->cap.set(cv::CAP_PROP_FRAME_WIDTH, this->image_size.width);
	this->cap.set(cv::CAP_PROP_FRAME_HEIGHT, this->image_size.height);
	this->cap.set(cv::CAP_PROP_FPS, this->fps);

	if(!this->cap.isOpened()) {
		std::cerr << "Could not open camera " << std::to_string(this->hardware_id) << std::endl;
		exit(ERRCODE_CAM_SETUP);
	}
}

void Camera::stop_video() {
	cap.release();
}

cv::Mat Camera::retrieve_video_frame(bool undistort = false) {
	cv::Mat frame;

	this->cap.grab();
	this->cap.retrieve(frame);

	if(frame.empty()) {
		std::cerr << "Error grabbing frame from main camera" << std::endl;
		exit(ERRCODE_CAM_GRAB_FRAME);
	}

	if(undistort && this->calibrated) {
		frame = undistort(frame);
	}

	return frame;
}

cv::Mat Camera::single_capture(bool undistort = false) {
	open_video(hardware_id);
	cv::Mat image = retrieve_video_frame(undistort);
	stop_video();
	return image;
}

cv::Mat Camera::undistort(cv::Mat in) {
	cv::Mat out;
	cv::undistort(in, out, this->camera_matrix, this->distortion_matrix, this->new_camera_matrix);
	return out;
}