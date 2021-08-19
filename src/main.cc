#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "line.h"
#include "robot.h"
#include "utils.h"
#include "green_model.h"
//#include "rescue.h"

enum class State {
	line,
	rescue
}

int main() {
	// DEBUG SETUP
	cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);

	State state = State::line;
	Robot* robot = new Robot();

	auto time = std::chrono::system_clock::now();

	// CAMERA SETUP
	const int FRONT_CAM = robot->init_camera(0, false, 80, 48, 60); // Front camera
	const int BACK_LEFT_CAM = robot->init_camera(1, true); // Back left camera
	const int BACK_RIGHT_CAM = robot->init_camera(2, true); // Back right camera

	Line line(FRONT_CAM, robot);
	line.start();

	Rescue rescue();

	// MAIN LOOP
	while(1) {
		if(robot->button(BTN_RESTART)) {
			robot->button_wait(BTN_RESTART, false);
			// Wait for button to be pressed and released to resume
			bool p = false; // Is button pressed down
			while(1) {
				// TODO: Show debug camera views while waiting

				if(robot->button(BTN_RESTART) && !p) p == true;
				else if(!robot->button(BTN_RESTART) && p) break; // Button released after press
			}
			// Reset
			state = State::line;
			rescue.stop();

			line.stop();
			line.start();
		}

		switch(state) {
			case State::line:
				// Line is synchronised
				line.line();
				break;
			case State::rescue:
				// Monitor rescue thread

				break;
		}
	}

	return 0;
}