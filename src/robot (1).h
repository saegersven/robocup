#pragma once

#include <iostream>
#include <vector>
#include <chrono>
#include <mutex>
#include <atomic>
#include <thread>
#include <cmath>

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <opencv2/opencv.hpp>

#include "utils.h"
#include "errcodes.h"
#include "vision.h"

// PINS
#define M1_1 23
#define M1_2 24
#define M1_E 25
#define M2_1 17
#define M2_2 18
#define M2_E 4

#define BTN_RESTART 16
#define BTN_DEBUG 19

#define DIST_1 12, 13
#define DIST_2 5, 6

#define SERVO_1 26
#define SERVO_2 21

// Servo parameters
#define SERVO_MIN_PULSE 1000
#define SERVO_MAX_PULSE 2000
#define SERVO_MIN_ANGLE -45
#define SERVO_MAX_ANGLE 45

// Encoder movement parameters
#define FORWARD_CORRECTION_FACTOR 0.1f
// Unit: mm
#define WHEEL_CIRCUMFERENCE 50.0f
#define GEAR_RATIO 100.0f
// Encoder pulses per revolution of encoder shaft
#define PULSES_PER_REVOLUTION 20.0f

// Register Ids of GPIO expander GPIOA and GPIOB
#define ENCODER_PORTA 0x12
#define ENCODER_PORTB 0x13

// Wheel tangential speed * MOTOR_SPEED_CONVERSION_FACTOR = 0-100
// Approximate value; Used for initial speed setting
#define MOTOR_SPEED_CONVERSION_FACTOR 0.01f
// Factor for proportional control of motor speed
#define MOTOR_SPEED_CORRECTION_FACTOR 10.0f

class Robot {
private:
	// Main IO lock
	std::mutex io_mutex;
	// List of all initialized cameras
	std::vector<Camera> cams;

	// Async speed control thread
	//std::thread motor_update_thread;
	// Speed variables for both motors
	std::atomic<float> asc_speed_left;
	std::atomic<float> asc_speed_right;
	std::atomic<bool> asc_has_duration;
	// Speed values will be set to zero at this point in time
	std::atomic<std::chrono::time_point<std::chrono::high_resolution_clock>> asc_stop_time;

	// WiringPI id of Encoder GPIO Expander
	int mcp_fd;

	// Get encoder value via I2C
	uint8_t encoder_value_a();
	uint8_t encoder_value_b();

	// Main Async speed control function, runs in its own thread
	void asc();

public:
	Robot();

	// CAMERA
	int init_camera(int id, bool calibrated = false,
		int width = 320, int height = 192, int fps = 60);
	cv::Mat capture(int cam_id, bool undistort = false);
	void start_video(int cam_id);
	void stop_video(int cam_id);

	// MOVEMENT
	void stop();
	void turn(int8_t degrees);

	void m_asc(int8_t left, int8_t right, uint16_t duration = 0, bool wait = false);

	// Directly set motor speed
	void m(int8_t left, int8_t right, uint16_t duration = 0);
	
	void servo(uint8_t pin, int8_t angle);
	
	// SENSORS
	bool button(uint8_t pin);
	void button_wait(uint8_t pin, bool state = true, uint32_t timeout = 0xffffffff);
	uint16_t distance(uint8_t echo, uint8_t trig, uint16_t iterations);
};