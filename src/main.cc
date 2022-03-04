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

	const std::string GREEN_MODEL_PATH = "/home/pi/robocup/ml/green/model.tflite";

	State state = State::line;
	std::shared_ptr<Robot> robot = std::make_shared<Robot>();

	robot->stop();
	robot->set_gpio(LED_1, true);
	robot->set_gpio(LED_2, false);
	robot->set_gpio(BUZZER, false);

	int waiting_for_heading_cnt = 0;

	while(robot->get_heading() == 0) {
		std::cout << "FRONT DISTANCE: " << robot->single_distance(DIST_1, 2000) << std::endl;
		std::cout << "SIDE DISTANCE: " << robot->single_distance(DIST_2, 2000) << std::endl;
		robot->m(-30, 30, 20);
		robot->m(30, -30, 20);
		waiting_for_heading_cnt++;
		if (waiting_for_heading_cnt > 50) robot->beep(3000);
	}

	std::cout << "Heading not zer" << std::endl;
	
	std::cout << "\n/dev/cams/:" << std::endl;
   	system("ls /dev/cams/");

	robot->set_gpio(LED_1, false);
	robot->set_gpio(LED_2, true);
	const auto start_time = std::chrono::system_clock::now();

	const std::string SUB_MASK_PATH = "/home/pi/robocup/runtime_data/front_sub_mask.png";

	robot->servo(SERVO_2, GRAB_CLOSED, 500);
	robot->servo(SERVO_1, ARM_UP, 500);

	// CAMERA SETUP
	const int FRONT_CAM = robot->init_camera("/dev/cams/front", false, 80, 48, 60, SUB_MASK_PATH);	// Front camera

	Line line(FRONT_CAM, robot);
	line.start();

	Rescue rescue(robot);

	while(!robot->button(BTN_RESTART));
	while(robot->button(BTN_RESTART));
	robot->set_gpio(LED_2, false);
	robot->beep(100, BUZZER);
	delay(200);

	// MAIN LOOP
	while(1) {
		if(robot->button(BTN_RESTART)) {
			while(robot->button(BTN_RESTART));
			switch(state) {
				case State::line:
					line.stop();
					break;
				case State::rescue:
					rescue.stop();
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
					rescue.start();
				}
				break;
			}
			case State::rescue:
			{
				// Monitor rescue thread
				if(rescue.finished) {
					//rescue.stop();
					line.start();
					state = State::line;
				}
				break;
			}
		}
	}

	return 0;
}
 