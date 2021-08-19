#include "line.h"

Line::Line(int front_cam_id, Robot* robot) {
	this->front_cam_id = front_cam_id;
	this->robot = robot;

	reset();
}

Line::start() {
	// Start video feed from front camera
	robot->start_video(front_cam_id);

	running = true;
}

Line::stop() {
	// Stop video if running
	if(!running) return;

	robot->stop_video(front_cam_id);
	running = false;
}

Line::line() {
	cv::Mat frame = robot->capture(front_cam_id); // Retrieve video frame
	cv::Mat green_cut = frame(cv::Range(10, 70), cv::Range(8, 40));

	// TODO: Green color values and threshold
	if(pixel_count_over_threshold(green_cut, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0), 10)) {
		// Consult the mighty AI
		float confidence = 0.0f;
		switch(green_model.evaluate(frame, confidence)) {
			case GreenResult::left:
				// TODO: Go left
				std::cout << "Left (" << std::to_string(confidence) << ")" << std::endl;
				break;
			case GreenResult::right:
				// TODO: Go right
				std::cout << "Right (" << std::to_string(confidence) << ")" << std::endl;
				break;
			case GreenResult::dead_end:
				// TODO: Turn around
				std::cout << "Dead End (" << std::to_string(confidence) << ")" << std::endl;
				break;
		}
	}
}