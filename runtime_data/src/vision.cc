#include "vision.h"

#include <opencv2/opencv.hpp>

#include "errcodes.h"
#include "utils.h"

Camera::Camera(int hardware_id, bool calibrated, int width, int height, int fps) {
	this->hardware_id = hardware_id;
	this->fps = fps;
	this->image_size = cv::Size(width, height);

	if(calibrated) {
		calibration_from_file("cc_" + std::to_string(hardware_id) + ".bin");
		this->new_camera_matrix = cv::getOptimalNewCameraMatrix(
			this->camera_matrix, this->distortion_matrix, this->image_size, 1.0);
		this->calibrated = true;
	}
}

void Camera::calibration_from_file(const std::string& file_name) {
	FILE* file = fopen(file_name.c_str(), "rb");
	if(!file) {
		std::cout << "Could not open camera calibration file '" <<
			file_name << "'" << std::endl;
		exit(ERRCODE_FILE_OPEN);
	}

	CameraPropertiesSerialized c;
	fread(&c, sizeof(CameraPropertiesSerialized), 1, file);

	float camera_matrix_data[3][3] = {
		{c.fx, 0.0f, 0.0f},
		{0.0f, c.fy, 0.0f},
		{c.cx, c.cy, 0.0f}
	};

	this->camera_matrix = cv::Mat(3, 3, CV_32F, &camera_matrix_data);

	float distortion_matrix_data[5] = {
		c.k1, c.k2, c.p1, c.p2, c.k3
	};

	this->distortion_matrix = cv::Mat(1, 5, CV_32F, &distortion_matrix_data);

	fclose(file);
}

void Camera::load_subtractive_mask(const std::string& file_name) {
	this->subtractive_mask = cv::imread(file_name);
	has_subtractive_mask = true;
}

void Camera::open_video() {
	this->cap.open(hardware_id);
	this->cap.set(cv::CAP_PROP_FRAME_WIDTH, std::max(320, this->image_size.width));
	this->cap.set(cv::CAP_PROP_FRAME_HEIGHT, std::max(192, this->image_size.height));
	this->cap.set(cv::CAP_PROP_FPS, this->fps);
	this->cap.set(cv::CAP_PROP_FORMAT, CV_8UC3);

	if(!this->cap.isOpened()) {
		std::cerr << "Could not open camera " << std::to_string(this->hardware_id) << std::endl;
		exit(ERRCODE_CAM_SETUP);
	}
}

void Camera::stop_video() {
	cap.release();
}

cv::Mat Camera::retrieve_video_frame(bool undist) {
	cv::Mat frame;

	this->cap.grab();
	this->cap.retrieve(frame);

	if(frame.empty()) {
		std::cerr << "Error grabbing frame from main camera" << std::endl;
		exit(ERRCODE_CAM_GRAB_FRAME);
	}

	//frame = quarter_image(frame);

	cv::resize(frame, frame, this->image_size);

	if(has_subtractive_mask) {
		clipped_difference(frame, subtractive_mask, frame);
	}

	if(undist && this->calibrated) {
		frame = undistort(frame);
	}

	return frame;
}

cv::Mat Camera::single_capture(bool undist) {
	open_video();
	cv::Mat image = retrieve_video_frame(undist);
	stop_video();
	return image;
}

cv::Mat Camera::undistort(cv::Mat in) {
	cv::Mat out;
	cv::undistort(in, out, this->camera_matrix, this->distortion_matrix, this->new_camera_matrix);
	return out;
}