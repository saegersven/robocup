#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "line.h"
#include "robot.h"
#include "rescue.h"
#include "utils.h"
#include "neural_networks.h"

#define SILVER_NN_ID 1
#define SILVER_RESULT_NEGATIVE 0
#define SILVER_RESULT_POSITIVE 1

enum class State {
	line,
	rescue
};

int main() {
	std::cout << "Init" << std::endl;

	const std::string GREEN_MODEL_PATH = "../ml/green/model.tflite";

	State state = State::line;
	std::shared_ptr<Robot> robot = std::make_shared<Robot>();

	while(robot->get_heading() == 0) {
		std::cout << "FRONT DISTANCE: " << robot->distance(DIST_1, 5, 2000) << std::endl;
		std::cout << "SIDE DISTANCE: " << robot->distance(DIST_2, 5, 2000) << std::endl;
		robot->m(-30, 30, 20);
		robot->m(30, -30, 20);
	}
	std::cout << "Heading not zero" << std::endl;

	robot->beep(200, BUZZER);

	const auto start_time = std::chrono::system_clock::now();

	const std::string SUB_MASK_PATH = "../runtime_data/front_sub_mask.png";

	robot->servo(SERVO_2, GRAB_CLOSED, 500);
	robot->servo(SERVO_1, ARM_UP, 500);

	// CAMERA SETUP
	const int FRONT_CAM = robot->init_camera("/dev/cams/front", false, 80, 48, 60, SUB_MASK_PATH);	// Front camera

	Line line(FRONT_CAM, robot);
	line.start();

	while(!robot->button(BTN_DEBUG));
	while(robot->button(BTN_DEBUG));
	robot->beep(200, BUZZER);

	// cv::Mat f1 = robot->capture(FRONT_CAM);

	// robot->stop_video(FRONT_CAM);

	// robot->m(100, 100, 48 * DISTANCE_FACTOR);

	// robot->start_video(FRONT_CAM);

	// cv::Mat f2 = robot->capture(FRONT_CAM);

	// cv::imshow("F1", f1);
	// cv::imshow("F2", f2);
	// cv::waitKey(10000);

	// exit(0);

	// robot->servo(SERVO_2, ARM_UP_ANGLE);
	// delay(500);

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
