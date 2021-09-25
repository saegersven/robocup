#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "line.h"
#include "robot.h"
#include "rescue.h"
#include "utils.h"
#include "neural_networks.h"

#define TEST
#define TEST_MOTORS
//#define TEST_SERVO_1
//#define TEST_SERVO_2
//#define TEST_BUTTON_RESTART
//#define TEST_BUTTON_DEBUG
//#define TEST_DISTANCE

#include "test.h"

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

	// DEBUG SETUP
	//cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);

	State state = State::line;
	std::shared_ptr<Robot> robot = new Robot();

#ifdef TEST
	test(robot);
#endif

	const auto start_time = std::chrono::system_clock::now();

	// CAMERA SETUP
	const int FRONT_CAM = robot->init_camera(0, false, 160, 96, 60);	// Front camera
	//const int BACK_LEFT_CAM = robot->init_camera(1, true);			// Back left camera
	//const int BACK_RIGHT_CAM = robot->init_camera(2, true);			// Back right camera

	//robot->start_video(FRONT_CAM);
	cv::Mat frame = robot->capture(FRONT_CAM);

	cv::imshow("F", frame);
	cv::waitKey(2000);

	NeuralNetworks neural_networks;
	neural_networks.load_model(GREEN_MODEL_PATH);
	//neural_networks.load_model(SILVER_MODEL_PATH);

	Line line(FRONT_CAM, robot, neural_networks);
	line.start();

	//Rescue rescue();

	// MAIN LOOP
	while(1) {
		if(robot->button(BTN_RESTART)) {
			robot->button_wait(BTN_RESTART, false);
			// Wait for button to be pressed and released to resume
			bool p = false;
			while(1) {
				// TODO: Show debug camera views while waiting
				
				if(robot->button(BTN_RESTART) && !p) p == true;
				else if(!robot->button(BTN_RESTART) && p) break;
			}
			// Reset
			state = State::line;
			//rescue.stop();

			line.stop();
			line.start();
		}

		switch(state) {
			case State::line:
			{
				cv::Mat frame = robot->capture(FRONT_CAM);
				// Line is synchronised
				line.line(frame);

				// Check for silver
				/*float confidence = 0.0f;
				switch(neural_networks.infere(SILVER_NN_ID, frame, confidence)) {
					case SILVER_RESULT_POSITIVE:
						// Switch to rescue
						line.stop();
						//rescue.start();
						state = State::rescue;
						break;
				}*/
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
