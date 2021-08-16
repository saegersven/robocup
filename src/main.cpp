#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "line.hpp"
#include "robot.hpp"
#include "utils.hpp"
//#include "rescue.hpp"

enum class State {
	line,
	rescue
}

int main() {
	// DEBUG SETUP
	cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);

	State state = State::line;
	Robot robot();

	auto time = std::chrono::system_clock::now();

	// CAMERA SETUP
	const int FRONT_CAM = robot.init_camera(0, false); // Front camera
	const int BACK_LEFT_CAM = robot.init_camera(1, true); // Back left camera
	const int BACK_RIGHT_CAM = robot.init_camera(2, true); // Back right camera

	if(state == State::line) {
		// Start video feed from front camera
		robot.start_video(FRONT_CAM);
	}

	// MAIN LOOP
	while(1) {
		if(robot.button(BTN_RESTART)) {
			robot.button_wait(BTN_RESTART, false);
			// Wait for button to be pressed and released to resume
			bool p = false; // Is button pressed down
			while(1) {
				// TODO: Show debug camera views while waiting

				if(robot.button(BTN_RESTART) && !p) p == true;
				else if(!robot.button(BTN_RESTART) && p) break; // Button released after pressed down
			}
		}

		// Normal operation
		switch(state) {
			case State::line:
				break;
			case State::rescue:
				break;
		}
	}

	return 0;
}