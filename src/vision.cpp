#include "vision.hpp"

void CameraProperties::calibration_from_file(std::string& file_name) {
	FILE* file = fopen(file_name.c_str(), "rb");
	if(!file) {
		std::cout << "Could not open camera calibration file '" <<
			file_name << "'" << std::endl;
		exit(ERRCODE_FILE_OPEN);
	}

	camera_matrix(3, 3, cv::CV_32F, 0.0f);
	fread(&camera_matrix.at<float>(0, 0), sizeof(float), 1, file); // fx
	fread(&camera_matrix.at<float>(1, 1), sizeof(float), 1, file); // fy
	fread(&camera_matrix.at<float>(2, 0), sizeof(float), 1, file); // cx
	fread(&camera_matrix.at<float>(2, 1), sizeof(float), 1, file); // cy
	camera_matrix.at<float>(2, 2) = 1.0f;

	distortion_matrix(1, 5, cv::CV_32F);
	fread(&distortion_matrix.at<float>(0, 0), sizeof(float), 1, file); // k1
	fread(&distortion_matrix.at<float>(0, 1), sizeof(float), 1, file); // k2
	fread(&distortion_matrix.at<float>(0, 2), sizeof(float), 1, file); // p1
	fread(&distortion_matrix.at<float>(0, 3), sizeof(float), 1, file); // p2
	fread(&distortion_matrix.at<float>(0, 4), sizeof(float), 1, file); // k3

	fclose(file);
}

cv::Mat undistort(cv::Mat in, CameraProperties cam) {
	cv::Mat out;
	cv::undistort(in, out, cam.camera_matrix, cam.distortion_matrix, cam.new_camera_matrix);
	return out;
}