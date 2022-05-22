#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "line.h"
#include "robot.h"
#include "rescue.h"
#include "utils.h"
#include "victim_ml.h"

#define SILVER_NN_ID 1
#define SILVER_RESULT_NEGATIVE 0
#define SILVER_RESULT_POSITIVE 1

enum class State {
	line,
	rescue
};

int main() {
	std::cout << "Init" << std::endl;

	State state = State::line;
	std::shared_ptr<Robot> robot = std::make_shared<Robot>();

	robot->stop();
	robot->set_gpio(LED_1, true);
	robot->set_gpio(LED_2, false);
	robot->set_gpio(BUZZER, false);

	/*while(true) {
		float dist = robot->get_vl53l0x_distance(0);
		std::cout << std::to_string(dist) << std::endl;
		if (dist < 100.0f) robot->set_gpio(LED_2, true);
		else robot->set_gpio(LED_2, false);
	}*/
	/*
	//####################
	while(true) {
		for(int i = 0; i < 3; ++i) {
			float dist = robot->get_vl53l0x_distance(i);
			std::cout << dist << "\t";
		}
		std::cout << std::endl;
	}*/

	/*while(true) {
		int dist1 = robot->get_vl53l0x_distance(1);
		int dist2 = robot->get_vl53l0x_distance(2);
		std::cout << dist1 << "\t\t" << dist2 << "\t\t" << dist1 - dist2 << "\t\t" << "Angle: " << rad_to_deg(std::atan((dist2-dist1)/145.0f)) << "\n";
	}*/
	

	//####################
	VictimML v;

	cv::VideoCapture cap;
	cap.open("/dev/cams/back", cv::CAP_V4L2);
	if(!cap.isOpened()) {
		std::cout << "Back cam not opened" << std::endl;
	}

	while(true) {
		cv::Mat frame;
		cap.grab();
		cap.retrieve(frame);
		cv::Mat debug_frame = frame.clone();

		cv::Mat out = v.invoke(frame);
		
		std::vector<Victim> victims = v.extract_victims(out);

		std::string alive_text("Alive");
		std::string dead_text("Dead");

		for(int i = 0; i < victims.size(); ++i) {
			int x = victims[i].x * 4;
			int y = victims[i].y * 4;
			std::cout << x << ", " << y << std::endl;
			cv::putText(debug_frame, victims[i].dead ? dead_text : alive_text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, false);
			cv::circle(debug_frame, cv::Point(x, y), 2, cv::Scalar(255, 0, 0), 3);
		}

		cv::imshow("Frame", debug_frame);

		cv::Mat out_resized;
		cv::resize(out, out_resized, cv::Size(160, 120));
		cv::imshow("Out", out_resized);
		cv::waitKey(1);
	}

	exit(0);
	//####################

	int waiting_for_heading_cnt = 0;

	while(robot->get_heading() == 0) {
		std::cout << "FRONT DISTANCE: " << robot->single_distance(DIST_1, 2000) << std::endl;
		std::cout << "SIDE DISTANCE: " << robot->single_distance(DIST_2, 2000) << std::endl;
		robot->m(-30, 30, 20);
		robot->m(30, -30, 20);
		waiting_for_heading_cnt++;
		if (waiting_for_heading_cnt > 50) robot->beep(3000);
	}

	std::cout << "Heading not zero" << std::endl;
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
	delay(100);

	/*while(1) {
		std::cout << robot->get_pitch() << std::endl;
		delay(10);
	}*/

	/*cv::VideoCapture cap("/dev/cams/back", cv::CAP_V4L2);
	if(!cap.isOpened()) {
		std::cout << "Back cam not opened" << std::endl;
	}

	cv::Mat frame;
	while (1) {
		cap.grab();
		cap.retrieve(frame);
		//cv::flip(frame, frame, -1);
		cv::imshow("Back cam frame", frame);
		cv::waitKey(1);

		if(robot->button(BTN_DEBUG)) {
			save_img("/home/pi/Desktop/back_cam_calibration/", frame);
			delay(1000);
		}
	}
	cap.release();*/
	
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
			std::cout << "Stopped." << std::endl;
			delay(50);
			while(!robot->button(BTN_DEBUG)) {
				robot->stop();
			}
			delay(50);
			while(robot->button(BTN_DEBUG));
			delay(100);
			std::cout << "Restarted line." << std::endl;

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
					std::cout << "Starting line" << std::endl;
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
 