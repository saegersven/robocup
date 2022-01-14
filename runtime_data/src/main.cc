#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "line.h"
#include "robot.h"
#include "rescue.h"
#include "utils.h"

#define SILVER_NN_ID 1
#define SILVER_RESULT_NEGATIVE 0
#define SILVER_RESULT_POSITIVE 1

enum class State {
	line,
	rescue
};

int main() {
	std::cout << "Init" << std::endl;

	// ../ to get out of build directory
	const std::string GREEN_MODEL_PATH = "../ml/green/model.tflite";
	const std::string SILVER_MODEL_PATH = "../ml/silver/model.tflite";

	State state = State::line;
	std::shared_ptr<Robot> robot = std::make_shared<Robot>();

	const auto start_time = std::chrono::system_clock::now();

	const std::string SUB_MASK_PATH = "../runtime_data/front_sub_mask.png";

	// CAMERA SETUP
	const int FRONT_CAM = robot->init_camera(0, false, 80, 48, 60, SUB_MASK_PATH);	// Front camera

	Line line(FRONT_CAM, robot);
	line.start();

	while(!robot->button(BTN_DEBUG));
	while(robot->button(BTN_DEBUG));
	delay(50);

	//Rescue rescue();

	// MAIN LOOP
	while(1) {
		if(robot->button(BTN_DEBUG)) {
			while(robot->button(BTN_DEBUG));
			switch(state) {
				case State::line:
					line.stop();
					break;
				case State::rescue:
					//rescue.stop();
					state = State::line;
					break;
			}
			line.stop();
			robot->stop();
			delay(50);
			while(!robot->button(BTN_DEBUG));
			delay(50);
			while(robot->button(BTN_DEBUG));
			delay(100);

			line.start();
		}

		switch(state) {
			case State::line:
			{
				cv::Mat frame = robot->capture(FRONT_CAM);

				bool line_result = line.line(frame);

				// Check for silver
				if(line_result) {
					line.stop();

					state = State::rescue;
					//rescue.start();
				}
				break;
			}
			case State::rescue:
			{
				// Monitor rescue thread

				break;
			}
		}
	}

	return 0;
}
