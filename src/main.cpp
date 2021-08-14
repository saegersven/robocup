#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "line.hpp"
//#include "rescue.hpp"

enum class State {
	line,
	rescue
}

int main() {
	cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);

	State state = State::line;
	Robot robot();

	auto time = std::chrono::system_clock::now();
	// Manage states as well as video capture here
	while(1) {
		if(robot.button(BTN_RESTART)) {
			robot.button_wait(BTN_RESTART, false);
			// Wait for button to be pressed and released to resume
			bool p = false;
			while(1) {
				// TODO: Show debug camera views while waiting

				if(robot.button(BTN_RESTART) && !p) p == true;
				else if(!robot.button(BTN_RESTART) && p) break;
			}
		}

		switch(state) {
			case State::line:

				break;
			case State::rescue:

				break;
		}
	}

	return 0;
}