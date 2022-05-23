// robot max speed = 25,33 cm/s = 0.2533 m/s = 0.912 km/h

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
#include <VL53L0X.hpp>
#include <opencv2/opencv.hpp>

#include "utils.h"
#include "errcodes.h"
#include "vision.h"

// PINS
#define M1_1 18
#define M1_2 17
#define M1_E 4
#define M2_1 23
#define M2_2 24
#define M2_E 22

#define BUZZER 25

#define LED_1 20
#define LED_2 27

#define BTN_RESTART 19
#define BTN_DEBUG 16

#define DIST_1_ECHO 5
#define DIST_1_TRIG 6
#define DIST_1 5, 6
#define DIST_2_ECHO 12
#define DIST_2_TRIG 13
#define DIST_2 12, 13

#define SERVO_1 21
#define SERVO_2 26

// VL53L0X Sensor IDs and I2C addresses
#define DIST_FORWARD 0
#define VL53L0X_FORWARD_ADDR 0x31
#define VL53L0X_FORWARD_XSHUT 6
#define DIST_SIDE_FRONT 1
#define VL53L0X_SIDE_FRONT_ADDR 0x33
#define VL53L0X_SIDE_FRONT_XSHUT 5
#define DIST_SIDE_BACK 2
#define VL53L0X_SIDE_BACK_ADDR 0x35
#define VL53L0X_SIDE_BACK_XSHUT 13

// Servo parameters
#define SERVO_MIN_PULSE 1000
#define SERVO_MAX_PULSE 2000
#define SERVO_MIN_ANGLE -45
#define SERVO_MAX_ANGLE 45

// Encoder movement parameters
#define FORWARD_CORRECTION_FACTOR 0.1f
// Unit: mm
#define WHEEL_CIRCUMFERENCE 196.0f
#define GEAR_RATIO 100.0f
#define WHEEL_SPAN 155.0f
#define TURN_DIAMETER 486.0f

#define TURN_MAX_DURATION_FACTOR 460
#define TURN_MIN_DURATION_FACTOR 380
#define TURN_DURATION_FACTOR 6.5f

#define DISTANCE_FACTOR (4.2f + 3 * 0.42f)
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


// I2C API
int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t len);
int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t len);
void API_delay_msek(uint32_t msek);
#define API_I2C_ADDR1 0x28
#define _BNO055_I2C_ADDR 0x28

class Robot {
private:
	// Main IO lock
	std::mutex io_mutex;

	std::mutex dist_mutex;
	// List of all initialized cameras
	std::vector<Camera> cams;

	// VL53L0X objects
	std::vector<std::unique_ptr<VL53L0X>> vl53l0x_vec;

	// Async speed control thread
	//std::thread motor_update_thread;
	// Speed variables for both motors
	std::atomic<float> asc_speed_left;
	std::atomic<float> asc_speed_right;
	std::atomic<bool> asc_has_duration;
	// Speed values will be set to zero at this point in time
	std::atomic<std::chrono::time_point<std::chrono::high_resolution_clock>> asc_stop_time;

	std::atomic<bool> block_m;
	float last_heading = 0.0f;
	// WiringPI id of Encoder GPIO Expander
	int mcp_fd;

	int bno_fd;

	// Get encoder value via I2C
	uint8_t encoder_value_a();
	uint8_t encoder_value_b();

	// Main Async speed control function, runs in its own thread
	void asc();

	void init_bno055();

public:
	Robot();

	// Delay grabbing frames from the camera to keep it from freezing
	// void delay_c(uint32_t ms, int id);

	// CAMERA
	int init_camera(const std::string& id, bool calibrated = false,
		int width = 320, int height = 192, int fps = 60, const std::string& subtractive_mask_path = "");
	cv::Mat capture(int cam_id, bool undistort = false);
	void start_video(int cam_id);
	void stop_video(int cam_id);

	// MOVEMENT
	void stop(uint8_t brake_duty_cycle = 100);
	void turn(float rad);
	void turn_to(float heading);
	void turn_to_heading(float heading);
	void turn_to_heading_last(float heading, float speed, bool dir);
	void straight(int8_t speed, uint32_t duration = 0);

	// Directly set motor speed
	void m(int8_t left, int8_t right, int32_t duration = 0, uint8_t brake_duty_cycle = 100);
	// Set motor speed with async speed control
	void m_asc(int8_t left, int8_t right, uint16_t duration = 0, bool wait = false);
	void block(bool val = true);

	void servo(uint8_t pin, float angle, uint16_t d = 0);
	void attach_servo(uint8_t pin);
	void write_servo(uint8_t pin, float angle);
	void release_servo(uint8_t pin);

	void beep(uint16_t ms, uint8_t pin = BUZZER);
	void set(uint8_t pin, bool value);
	
	// SENSORS
	bool button(uint8_t pin);
	void button_wait(uint8_t pin, bool state = true, uint32_t timeout = 0xffffffff);
	//float single_distance(int8_t echo, uint8_t trig, int timeout = 20);
	//float distance_avg(uint8_t echo, uint8_t trig, uint8_t measurements = 1, float remove_percentage = 0.2f, uint32_t timeout_single_measurement = 200, uint32_t timeout = 2000);

	float get_heading();
	float get_pitch();
	void set_gpio(int pin, bool state);

	uint16_t distance(uint8_t sensor_id);
};