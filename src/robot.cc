#include "robot.h"

#include <iostream>
#include <vector>
#include <chrono>
#include <mutex>
#include <atomic>
#include <thread>
#include <cmath>

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <softPwm.h>
#include <opencv2/opencv.hpp>

#include "utils.h"
#include "errcodes.h"
#include "vision.h"

Robot::Robot() : asc_stop_time(std::chrono::high_resolution_clock::now()) {
	std::cout << "Robot setup" << std::endl;

	io_mutex.lock();

	// Initialize the robot
	// Setup GPIO
	wiringPiSetupGpio();

	// Setup I2C
	/*mcp_fd = wiringPiI2CSetup(0x20); // TODO: Check if this is the actual ID of MCP23017
	if(mcp_fd == -1) {
		std::cout << "Error setting up I2C for Encoder board" << std::endl;
		exit(ERRCODE_BOT_SETUP_I2C);
	}*/

	// Setup Drive motor pins
	pinMode(M1_1, OUTPUT);
	pinMode(M1_2, OUTPUT);
	pinMode(M2_1, OUTPUT);
	pinMode(M2_2, OUTPUT);

	pinMode(BUZZER, OUTPUT);
	pinMode(LED_1, OUTPUT);
	pinMode(LED_2, OUTPUT);

	pinMode(BTN_DEBUG, INPUT);
	pinMode(BTN_RESTART, INPUT);

	pinMode(DIST_1_ECHO, INPUT);
	pinMode(DIST_1_TRIG, OUTPUT);
	pinMode(DIST_2_ECHO, INPUT);
	pinMode(DIST_2_TRIG, OUTPUT);

	// Create PWM for the two drive motors
	if(softPwmCreate(M1_E, 0, 100)) {
		std::cout << "Error setting up PWM for M1" << std::endl;
		exit(ERRCODE_BOT_SETUP_PWM);
	}
	if(softPwmCreate(M2_E, 0, 100)) {
		std::cout << "Error setting up PWM for M2" << std::endl;
		exit(ERRCODE_BOT_SETUP_PWM);
	}

	// Setup Servo motor pins
	// 50Hz has a period of 20ms, so multiply value by 100 to get period in microseconds
	if(softPwmCreate(SERVO_1, 0, 200)) {
		std::cout << "Error setting up PWM for Servo 1" << std::endl;
		exit(ERRCODE_BOT_SETUP_PWM);
	}
	if(softPwmCreate(SERVO_2, 0, 200)) {
		std::cout << "Error setting up PWM for Servo 2" << std::endl;
		exit(ERRCODE_BOT_SETUP_PWM);
	}

	io_mutex.unlock();

	this->asc_speed_left = 0.0f;
	this->asc_speed_right = 0.0f;
	this->asc_has_duration = false;

	std::thread motor_update_thread(&Robot::asc, this); // Create async motor control thread
	motor_update_thread.detach();
}

int Robot::init_camera(int id, bool calibrated, int width, int height, int fps) {
	Camera cam(id, calibrated, width, height, fps);

	cams.push_back(cam);
	return cams.size() - 1;
}

cv::Mat Robot::capture(int cam_id, bool undistort) {
	if(cams[cam_id].cap.isOpened()) {
		return cams[cam_id].retrieve_video_frame(undistort);
	} else {
		return cams[cam_id].single_capture(undistort);
	}
}

void Robot::start_video(int cam_id) {
	cams[cam_id].open_video();
}

void Robot::stop_video(int cam_id) {
	cams[cam_id].stop_video();
}

void Robot::m(int8_t left, int8_t right, uint16_t duration, uint8_t brake_duty_cycle) {
	left = clip(left, -100, 100);
	right = clip(right, -100, 100);

	io_mutex.lock();

	// Reconfigure the H-Bridge's direction control pins
	//digitalWrite(M1_1, left < 0 ? HIGH : LOW);
	//digitalWrite(M1_2, left <= 0 ? LOW : HIGH);

	digitalWrite(M1_1, left < 0 ? HIGH : LOW);
	digitalWrite(M1_2, left <= 0 ? LOW : HIGH);

	digitalWrite(M2_1, right < 0 ? HIGH : LOW);
	digitalWrite(M2_2, right <= 0 ? LOW : HIGH);

	// Adjust motor speed over PWM pins
	softPwmWrite(M1_E, std::abs(left));
	softPwmWrite(M2_E, std::abs(right));

	io_mutex.unlock();

	// Wait for duration and stop
	if(duration != 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(duration));
		stop(brake_duty_cycle);
	}
}

void Robot::stop(uint8_t brake_duty_cycle) {
	io_mutex.lock();
	digitalWrite(M1_1, LOW);
	digitalWrite(M1_2, LOW);
	digitalWrite(M2_1, LOW);
	digitalWrite(M2_2, LOW);

	softPwmWrite(M1_E, brake_duty_cycle);
	softPwmWrite(M2_E, brake_duty_cycle);
	io_mutex.unlock();
}

uint8_t Robot::encoder_value_a() {
	// Read from GPIOA register of MCP23017
	io_mutex.lock();
	uint8_t value = wiringPiI2CReadReg8(mcp_fd, ENCODER_PORTA);
	io_mutex.unlock();
	return value;
}

uint8_t Robot::encoder_value_b() {
	// Read from GPIOB register of MCP23017
	io_mutex.lock();
	uint8_t value = wiringPiI2CReadReg8(mcp_fd, ENCODER_PORTB);
	io_mutex.unlock();
	return value;
}

void Robot::m_asc(int8_t left, int8_t right, uint16_t duration, bool wait) {
	asc_speed_left = left;
	asc_speed_right = right;

	if(duration != 0) {
		asc_has_duration = true;
		asc_stop_time = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(duration);
	}
}

void Robot::asc() {
	uint8_t last_encoder_value_left = encoder_value_a();
	uint8_t last_encoder_value_right = encoder_value_b();

	int8_t real_speed_left = 0;
	int8_t real_speed_right = 0;

	int8_t real_speed_offset_left = 0;
	int8_t real_speed_offset_right = 0;

	auto last_update = std::chrono::high_resolution_clock::now();
	while(1) {
		if(asc_speed_left == 0.0f && asc_speed_right == 0.0f) continue;

		auto now = std::chrono::high_resolution_clock::now();

		if(asc_has_duration && now > asc_stop_time.load()) {
			asc_speed_left == 0.0f;
			asc_speed_right == 0.0f;
			asc_has_duration = false;
			continue;
		}

		int8_t speed_sign_left = asc_speed_left < 0.0f ? -1 : 1;
		int8_t speed_sign_right = asc_speed_right < 0.0f ? -1 : 1;
		
		real_speed_left = asc_speed_left * MOTOR_SPEED_CONVERSION_FACTOR;
		real_speed_right = asc_speed_right * MOTOR_SPEED_CONVERSION_FACTOR;

		float delta_t = std::chrono::duration_cast<std::chrono::seconds>(now - last_update).count();
		last_update = now;

		uint8_t delta_left = encoder_value_a() - last_encoder_value_left;
		uint8_t delta_right = encoder_value_b() - last_encoder_value_right;

		// Calculate current motor speed
		float speed_left = delta_left * WHEEL_CIRCUMFERENCE /
			PULSES_PER_REVOLUTION / GEAR_RATIO / delta_t;
		float speed_right = delta_right * WHEEL_CIRCUMFERENCE /
			PULSES_PER_REVOLUTION / GEAR_RATIO / delta_t;

		float left_error = speed_left * speed_sign_left - asc_speed_left;
		float right_error = speed_right * speed_sign_right - asc_speed_right;

		real_speed_offset_left -= (int8_t)left_error * MOTOR_SPEED_CORRECTION_FACTOR;
		real_speed_offset_right -= (int8_t)right_error * MOTOR_SPEED_CORRECTION_FACTOR;

		m(real_speed_left + real_speed_offset_left, real_speed_right + real_speed_offset_right);
	}
}

void Robot::servo(uint8_t pin, int8_t angle, bool wait) {
	io_mutex.lock();
	softPwmWrite(pin, map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE,
		SERVO_MIN_PULSE, SERVO_MAX_PULSE) / 100.0f);
	io_mutex.unlock();
	if(wait) std::this_thread::sleep_for(std::chrono::milliseconds(10 * angle));
}

void Robot::beep(uint16_t ms, uint8_t pin) {
	digitalWrite(pin, HIGH);
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
	digitalWrite(pin, LOW);

}
/*// Go in a straight line
void Robot::drive_distance(float distance, int8_t speed) {
	int8_t sign = distance < 0 ? -1 : 1;

	float distance_travelled = 0.0f;

	int16_t total_error = 0;

	io_mutex.lock();
	uint8_t last_encoder_a = encoder_value_a();
	uint8_t last_encoder_b = encoder_value_b();

	while(distance_travelled < distance) {
		uint8_t encoder_a = encoder_value_a();
		uint8_t encoder_b = encoder_value_b();

		// Difference in pulses
		// No need to account for counter overflow, as deltas will overflow too
		uint8_t d_encoder_a = encoder_a - last_encoder_a;
		uint8_t d_encoder_b = encoder_b - last_encoder_b;
		
		// Average encoder deltas to calculate distance travelled
		distance_travelled += (d_encoder_a + d_encoder_b) / 2.0f *
			PULSES_PER_REVOLUTION * GEAR_RATIO * WHEEL_CIRCUMFERENCE

		total_error += d_encoder_b - d_encoder_a;

		m(sign * (speed + total_error * FORWARD_CORRECTION_FACTOR),
			sign * (speed - total_error * FORWARD_CORRECTION_FACTOR), 0);

		last_encoder_a = encoder_a;
		last_encoder_b = encoder_b;
	}
	io_mutex.unlock();

	stop();
}*/

bool Robot::button(uint8_t pin) {
	io_mutex.lock();
	bool button_down = digitalRead(pin) == HIGH;
	io_mutex.unlock();
	return button_down;
}

void Robot::button_wait(uint8_t pin, bool state, uint32_t timeout) {
	io_mutex.lock();
	auto start_time = std::chrono::system_clock::now();
	while(Robot::button(pin) != state) {
		auto now = std::chrono::system_clock::now();
		if(std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() >= timeout) {
			break;
		}
	}
	io_mutex.unlock();
}

float Robot::distance(uint8_t echo, uint8_t trig, uint16_t iterations, uint32_t timeout) {
	float timeElapsed = 0.0f;
	io_mutex.lock();
	for(uint16_t i = 0; i < iterations; i++) {
		digitalWrite(trig, HIGH);
		std::this_thread::sleep_for(std::chrono::microseconds(100));
		digitalWrite(trig, LOW);

		std::chrono::time_point<std::chrono::high_resolution_clock> start_time, signal_start, signal_stop;

		while (digitalRead(echo) == LOW) {
			signal_start = std::chrono::high_resolution_clock::now();

			if(std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::high_resolution_clock::now() - start_time).count() > timeout) {
				// Timeout
				continue;
			}
		}

		while (digitalRead(echo) == HIGH) {
			signal_stop = std::chrono::high_resolution_clock::now();

			if(std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::high_resolution_clock::now() - start_time).count() > timeout) {
				// Timeout
				continue;
			}
		}

		timeElapsed += std::chrono::duration_cast<std::chrono::microseconds>(signal_stop - signal_start).count();
		delay(5);
	}
	io_mutex.unlock();
	// Multiply with speed of sound (0,0343 cm/us) and divide by 2 to get one-way distance
	return timeElapsed / (float)iterations * 0.0343f * 0.5f;
}