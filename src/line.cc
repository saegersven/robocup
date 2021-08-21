#include "line.h"

Line::Line(int front_cam_id, Robot* robot, NeuralNetworks neural_networks) {
	this->front_cam_id = front_cam_id;
	this->robot = robot;
	this->neural_networks = neural_networks

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

Line::line(cv::Mat& frame) {
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