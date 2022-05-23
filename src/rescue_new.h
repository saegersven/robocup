#pragma once

#define CAM_FAR 0
#define CAM_SHORT 1

class Rescue {
private:
	std::shared_ptr<Robot> robot;
	VictimML victimML;

	std::thread::native_handle_type native_handle;

	const std::string[2] cam_filenames = {
		"/dev/cams/back",
		"/dev/cams/front"
	};
	cv::VideoCapture caps;

	cv::Mat capture(uint8_t cam_id);
	void rescue();
	float get_angle_to_right_wall();
	void find_black_corner();
	bool rescue_victim(bool ignore_dead);

public:
	Rescue(std::shared_ptr<Robot> robot);
	void start();
	void stop();
}