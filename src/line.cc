#include "line.h"

Line::Line(int front_cam_id, Robot* robot, NeuralNetworks neural_networks) {
	this->front_cam_id = front_cam_id;
	this->robot = robot;
	this->neural_networks = neural_networks

	reset();
}

void Line::start() {
	// Start video feed from front camera
	robot->start_video(front_cam_id);

	running = true;
}

void Line::stop() {
	// Stop video if running
	if(!running) return;

	robot->stop_video(front_cam_id);
	running = false;
}

void Line::line(cv::Mat& frame) {
	//cv::Mat frame = robot->capture(front_cam_id); // Retrieve video frame
	cv::Mat green_cut = frame(cv::Range(10, 70), cv::Range(8, 40));

	// TODO: Green color values and threshold
	if(pixel_count_over_threshold_hue(green_cut, 110, 140, 40, 1000)) {
		// Consult the mighty AI
		float confidence = 0.0f;
		switch(neural_networks.infere(GREEN_NN_ID, frame, confidence)) {
			case GREEN_RESULT_LEFT:
				// TODO: Go left
				std::cout << "Left (" << std::to_string(confidence) << ")" << std::endl;
				break;
			case GREEN_RESULT_RIGHT:
				// TODO: Go right
				std::cout << "Right (" << std::to_string(confidence) << ")" << std::endl;
				break;
			case GREEN_RESULT_DEAD_END:
				// TODO: Turn around
				std::cout << "Dead End (" << std::to_string(confidence) << ")" << std::endl;
				break;
		}
	}
}

// TODO: Find better values
uint8_t Line::green(cv::Mat& frame) {
	cv::Mat green = inRange_hue(frame, 110, 140, 40);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(green, contours, cv::RETR_TREE, cv::CV_CHAIN_APPROX_SIMPLE);

	// Four positions are possible:
	//	  4|2
	//	------
	//    0|1
	// green_points is a bitmask
	// We don't account for the top two
	uint8_t green_points;

	cv::Point bottom_center(frame.cols / 2, frame.rows - 1);

	const float check_distance_factor = 0.1f; // Go a tenth of the size of the rect to check

	for(int i = 0; i < contours.size(); ++i) {
		cv::RotatedRect r = cv::minAreaRect(contours[i]);
		// Only consider contour if it is near the bottom center of the image
		if(point_distance(bottom_center, r.center) > 25.0f)
			continue;
		// For each contour, compute the position relative to the nearest line
		// by checking the colors at the edges of the min area rects
		uint8_t black;
		float angle, cx, cy;
		int i = 0;
		for(float angle_offset = 0.0f; angle_offset < 360.0f; angle_offset += 90.0f) {
			angle = r.angle + angle_offset;
			cx = r.center.x + std::cos(angle) * r.size.width * check_distance_factor;
			cy = r.center.y + std::sin(angle) * r.size.height * check_distance_factor;
			black |= (bgr_to_lightness(frame.at<cv::Vec3b>((int)cx, (int)cy)) < 30 ? 0x01 : 0x00) << i;
			++i;
		}

		switch(black) {
			case 0b00001001:
				// Bottom left, bit for line right (1st) and line top (4th) are set
				green_points |= 0x01; // Set first bit
				break;
			case 0b00001100:
				// Bottom right, bit for line left (3rd) and line top (4th) are set
				green_points |= 0x02; // Set second bit
				break;
			default:
				// Ignore green points at the top
		}
	}
	// If both left and right green is there, return value is 3
	// If only left or right, return value is 1 or 2
	// If no contours were at the bottom, return value is 0
	return green_points;
}