#include "robot.h"

#include <iostream>
#include <vector>
#include <chrono>
#include <mutex>
#include <atomic>
#include <thread>
#include <cmath>

#include <unistd.h>
#include <iomanip>

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <softPwm.h>
#include <opencv2/opencv.hpp>

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include "utils.h"
#include "errcodes.h"
#include "vision.h"
#include <typeinfo>

Robot::Robot() : asc_stop_time(std::chrono::high_resolution_clock::now()) {
	std::cout << "Robot setup" << std::endl;

	io_mutex.lock();

	// Initialize the robot
	// Setup GPIO
	wiringPiSetupGpio();

	init_bno055();

	// Setup I2C
	/*mcp_fd = wiringPiI2CSetup(0x20); // TODO: Check if this is the actual ID of MCP23017
	if(mcp_fd == -1) {
		std::cout << "Error setting up I2C for Encoder board" << std::endl;
		exit(ERRCODE_BOT_SETUP_I2C);
	}*/
	mcp_fd = 0;

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
	if(softPwmCreate(M1_E, 0, 101)) {
		std::cout << "Error setting up PWM for M1" << std::endl;
		exit(ERRCODE_BOT_SETUP_PWM);
	}
	if(softPwmCreate(M2_E, 0, 101)) {
		std::cout << "Error setting up PWM for M2" << std::endl;
		exit(ERRCODE_BOT_SETUP_PWM);
	}

	const uint8_t vl53l0x_xshuts[3] = {
		VL53L0X_FORWARD_XSHUT,
		VL53L0X_SIDE_FRONT_XSHUT,
		VL53L0X_SIDE_BACK_XSHUT
	};

	const uint8_t vl53l0x_addresses[3] = {
		VL53L0X_FORWARD_ADDR,
		VL53L0X_SIDE_FRONT_ADDR,
		VL53L0X_SIDE_BACK_ADDR
	};

	// Setup VL53L0X distance sensors
	for(int i = 0; i < 3; ++i) {
		vl53l0x_vec.push_back(std::make_unique<VL53L0X>(vl53l0x_xshuts[i]));
		vl53l0x_vec[i]->powerOff();
	}
	delay(200);

	for(int i = 0; i < 3; ++i) {
		delay(100);
		vl53l0x_vec[i]->initialize();
		vl53l0x_vec[i]->setTimeout(200);
		vl53l0x_vec[i]->setMeasurementTimingBudget(40000);
		vl53l0x_vec[i]->setAddress(vl53l0x_addresses[i]);
		delay(100);
	}

	std::cout << "VL53L0Xs initialized" << std::endl;

	io_mutex.unlock();

	this->asc_speed_left = 0.0f;
	this->asc_speed_right = 0.0f;
	this->asc_has_duration = false;

	this->block_m = false;

	std::thread motor_update_thread(&Robot::asc, this); // Create async motor control thread
	motor_update_thread.detach();
}

int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t* reg_data, uint8_t len) {
	//std::cout << "W\t" << std::to_string(reg_addr) << "\t" << std::to_string(reg_data[0]) << std::endl;

	uint8_t write_buf[1 + len];
	write_buf[0] = reg_addr;
	for(int i = 0; i < len; ++i) {
		write_buf[i + 1] = reg_data[i];
	}

	if(write(dev_addr, write_buf, 1) != 1) {
		return 1;
	}
	return 0;
}

int8_t i2c_write8(uint8_t dev_addr, uint8_t reg_addr, uint8_t reg_data) {
	wiringPiI2CWriteReg8(dev_addr, reg_addr, reg_data);
	return 0;
}

int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t len) {
	//std::cout << "R\t" << std::to_string(reg_addr) << std::endl;
	uint8_t write_buf[1] = {reg_addr};
	if(write(dev_addr, write_buf, 1) != 1) {
		return 1;
	}
	if(read(dev_addr, reg_data, len) != len) {
		return 1;
	}
	return 0;
}

void API_delay_msek(uint32_t msek) {
	delay(msek);
}

void Robot::init_bno055() {
	// Init I2C device
	bno_fd = wiringPiI2CSetup(_BNO055_I2C_ADDR);
	if(bno_fd == -1) {
		std::cout << "I2C Error setting up BNO055" << std::endl;
		exit(ERRCODE_BOT_SETUP_I2C);
	}

	uint8_t chip_id[1] = {0};
	i2c_read(bno_fd, 0, chip_id, 1);

	i2c_write8(bno_fd, 61, 0);
	delay(30);
	i2c_write8(bno_fd, 63, 32); // SYS_TRIGGER bit 5: Reset
	delay(700);
	i2c_write8(bno_fd, 62, 0); // PWR_MODE: Set power mode to 0 (Normal)
	i2c_write8(bno_fd, 7, 0); // Set page id to 0
	i2c_write8(bno_fd, 63, 0); // Reset system triggers to 0
	i2c_write8(bno_fd, 7, 1); // Set page id to 1
	i2c_write8(bno_fd, 8, 13); // Set accel range 4G and bandwidth 62_5HZ
	i2c_write8(bno_fd, 7, 0); // Set page id to 0
	i2c_write8(bno_fd, 7, 1); // Set page id to 1
	i2c_write8(bno_fd, 10, 56); // Gyro config
	i2c_write8(bno_fd, 7, 0); // Set page id to 0
	i2c_write8(bno_fd, 7, 1); // Set page id to 1
	i2c_write8(bno_fd, 9, 13); // Mag config
	i2c_write8(bno_fd, 7, 0); // Set page id to 0
	delay(10);
	i2c_write8(bno_fd, 61, 0); // Enter config mode
	delay(20);
	i2c_write8(bno_fd, 61, 12); // Enter NDOF operation mode
	delay(10);

	std::cout << "Init complete" << std::endl;
}

float Robot::get_heading() {
	float fl_euler_data_h;

	//bno_comres += bno055_convert_double_euler_h_rad(&f_euler_data_h);
	int16_t temp = 0;
	i2c_read(bno_fd, 26, (uint8_t*)&temp, 2);

	fl_euler_data_h = (float)temp / 16.0f * PI / 180.0f;

	if (fl_euler_data_h < 0) return last_heading;
	last_heading = fl_euler_data_h;

	return fl_euler_data_h;
}

float Robot::get_pitch() {
	float fl_euler_data_p;

	int16_t temp = 0;
	i2c_read(bno_fd, 28, (uint8_t*)&temp, 2);

	fl_euler_data_p = (float)temp / 16.0f * PI / 180.0f;

	return fl_euler_data_p;
}

void Robot::reset_vl53l0x(uint8_t sensor_id) {
	uint8_t addr = vl53l0x_vec[sensor_id]->getAddress();
	int16_t xshut_pin = vl53l0x_vec[sensor_id]->getXshutGPIOPin();
	vl53l0x_vec[sensor_id] = std::make_unique<VL53L0X>(xshut_pin);
	vl53l0x_vec[sensor_id]->powerOff();
	delay(200);
	vl53l0x_vec[sensor_id]->initialize();
	vl53l0x_vec[sensor_id]->setTimeout(200);
	vl53l0x_vec[sensor_id]->setMeasurementTimingBudget(40000);
	vl53l0x_vec[sensor_id]->setAddress(addr);
	delay(200);
}

uint16_t Robot::distance(uint8_t sensor_id, uint8_t num_reboots) {
	uint16_t distance = 9000;
	if(num_reboots == 10) {
		std::cout << "Uh oh: Still not working after 10 reboots" << std::endl;
		return 9000;
	}
	try {
		distance = vl53l0x_vec[sensor_id]->readRangeSingleMillimeters();
	} catch(std::exception& e) {
		std::cout << "Failed to read VL53L0X, resetting sensor" << std::endl;
		std::cout << e.what() << std::endl;
		this->reset_vl53l0x(sensor_id);
		return this->distance(sensor_id, num_reboots + 1);
	}
	if(vl53l0x_vec[sensor_id]->timeoutOccurred()) {
		std::cout << "VL53L0X (" << std::to_string(sensor_id) << ") timeout!" << std::endl;
		return 4000;
	}
	return distance;
}

int Robot::init_camera(const std::string& id, bool calibrated, int width, int height, int fps, const std::string& subtractive_mask_path) {
	Camera cam(id, calibrated, width, height, fps);

	if(strcmp(subtractive_mask_path.c_str(), "")) {
		cam.load_subtractive_mask(subtractive_mask_path);
	}

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

void Robot::m(int8_t left, int8_t right, int32_t duration, uint8_t brake_duty_cycle) {
	// WARNING: left and rigth motor swapped
	if(block_m) return;

	left = clip(left, -100.0f, 100.0f);
	right = clip(right, -100.0f, 100.0f);

	// If duration is less than zero, flip motor pins
	if(duration < 0) {
		duration *= -1;
		left *= -1;
		right *= -1;
	}

	io_mutex.lock();

	// Reconfigure the H-Bridge's direction control pins

	digitalWrite(M1_1, left < 0 ? HIGH : LOW);
	digitalWrite(M1_2, left <= 0 ? LOW : HIGH);

	digitalWrite(M2_1, right < 0 ? HIGH : LOW);
	digitalWrite(M2_2, right <= 0 ? LOW : HIGH);

	// Adjust motor speed over PWM pins
	softPwmWrite(M1_E, std::abs(left));
	softPwmWrite(M2_E, std::abs(right));

	// Wait for duration and stop
	if(duration != 0) {
		delay(duration);
		io_mutex.unlock();
		stop(brake_duty_cycle);
		return;
	}
	io_mutex.unlock();
}

void Robot::block(bool val) {
	block_m = val;
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

void Robot::turn(float rad) {
	if(rad == 0.0f) {
		stop();
		return;
	}
	if(block_m) return;

	//m(100, -100, deg * TURN_DURATION_FACTOR);
	float start_heading = get_heading();
	float to_turn = std::abs(rad);

	bool clockwise = rad > 0.0;

	if(to_turn < 0.15) {
		m(clockwise ? -30 : 30, clockwise ? 30 : -30, TURN_DURATION_FACTOR * std::abs(rad_to_deg(rad)));
		return;
	}

	auto start_time = std::chrono::high_resolution_clock::now();
	uint32_t min_time = TURN_MIN_DURATION_FACTOR * std::abs(rad);
	//uint32_t max_time = TURN_MAX_DURATION_FACTOR * std::abs(rad);

	//std::cout << to_turn << " " << min_time << " " << max_time << std::endl;
	m(clockwise ? -30 : 30, clockwise ? 30 : -30);
	while(1) {
		float new_heading;
		do {
			new_heading = get_heading();
		} while(new_heading > RAD_360 || new_heading < 0.0);
		// std::cout << "New heading " << rad_to_deg(new_heading) << std::endl;

		if(clockwise) {
			if(new_heading < start_heading - RAD_90) start_heading -= RAD_360;
		} else {
			if(new_heading > start_heading + RAD_90) start_heading += RAD_360;
		}
		float d_heading = new_heading - start_heading;

		uint32_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();

		if((ms > min_time && std::abs(d_heading) >= to_turn - deg_to_rad(11.0f))) break;
	}
	stop();
}

// dir: true = clockwise, false = counterclockwise
void Robot::turn_to_heading_last(float heading, float speed, bool dir) {
	float last_heading = get_heading();
	float curr_heading = last_heading;

	const float CORRECTION_DURATION = 70;

	while(1) {
		curr_heading = get_heading();
		if(std::abs(curr_heading - last_heading) > RAD_180) break;
		last_heading = curr_heading;
		if(dir) {
			if(curr_heading >= heading) break;
			m(-speed, speed);
		} else {
			if(curr_heading <= heading) break;
			m(speed, -speed);
		}
	}

	if(dir) {
		m(50, -50, CORRECTION_DURATION);
	} else {
		m(-50, 50, CORRECTION_DURATION);
	}
}

void Robot::turn_to_heading(float heading) {
	// TODO: recursive

	float curr_heading = get_heading();
	std::cout << "pHeading: " << heading << "   curr_heading: " << curr_heading << std::endl;

	if (heading == 0) heading = 0.01f;
	if (heading == curr_heading) turn(deg_to_rad(3));
	int SPEED = 30;

	if (curr_heading < heading && heading - PI < curr_heading) {
		std::cout << "Case 1" << std::endl;
		turn_to_heading_last(heading, SPEED, true);
	} 
	else if (curr_heading < heading && heading - PI > curr_heading) {
		std::cout << "Case 2" << std::endl;
		while (get_heading() > 0.15) m(SPEED, -SPEED); // turn to ~0°
		turn(-0.2); // turn over 0° threshold
		turn_to_heading_last(heading, SPEED, false);
	}
	else if (curr_heading > heading && heading + PI < curr_heading) {
		std::cout << "Case 3" << std::endl;
		while (get_heading() < 6.13) m(-SPEED, SPEED); // turn to ~0°
		turn(0.2); // turn over 0° threshold
		turn_to_heading_last(heading, SPEED, true);
	}
	else if (curr_heading > heading && heading + PI > curr_heading) {
		std::cout << "Case 4" << std::endl;
		turn_to_heading_last(heading, SPEED, false);
	} else {
		std::cout << "turn_to_heading: UNKNOWN CASE, pHeading: " << heading << "   curr_heading: " << curr_heading << std::endl;
	}

	stop();
}

void Robot::straight(int8_t speed, uint32_t duration) {
	auto start_time = std::chrono::high_resolution_clock::now();
	auto last_update = std::chrono::high_resolution_clock::now();

	float start_heading;
	do {
		start_heading = get_heading();
	} while(start_heading > RAD_360 || start_heading < 0.0f);

	std::cout << rad_to_deg(start_heading) << std::endl;

	float integral = 0.0f;

	const float I_FACTOR = 0.1f;
	const float I_DAMPEN_FACTOR = 0.75f;
	const float P_FACTOR = 200.0f;

	float last_error = 0.0f;

	while(1) {
		auto now = std::chrono::high_resolution_clock::now();
		uint32_t t = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

		float dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update).count() / 1000000.0f;

		if(duration != 0 && t >= duration) {
			break;
		}

		float error = get_heading() - start_heading;
		if(error > RAD_180) error -= RAD_360;
		if(error < -RAD_180) error += RAD_360;

		std::cout << "E\t" << rad_to_deg(error) << "\r";

		if((error < 0 && last_error > 0) || (error > 0 && last_error < 0)) {
			// Sign change, reset integral to prevent overshooting
			integral = 0.0f;
		}
		last_error = error;

		integral = integral * I_DAMPEN_FACTOR + error * dt;
		float correction = P_FACTOR * error + I_FACTOR * integral;

		m((int8_t)clip(speed - correction, -100.0f, 100.0f), (int8_t)clip(speed + correction, -100.0f, 100.0f));
	}
	std::cout << std::endl;
	stop();
	delay(20);
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

void Robot::servo(uint8_t pin, float angle, uint16_t d) {
	io_mutex.lock();
	attach_servo(pin);
	
	write_servo(pin, angle);

	if(d != 0) delay(d);
	release_servo(pin);
	io_mutex.unlock();
}

void Robot::attach_servo(uint8_t pin) {
	if(softPwmCreate(pin, 0, 200)) {
		std::cout << "Error setting up PWM for pin " << pin << std::endl;
		exit(ERRCODE_BOT_SETUP_PWM);
	}
}

void Robot::write_servo(uint8_t pin, float angle) {
	softPwmWrite(pin, map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE,
		SERVO_MIN_PULSE, SERVO_MAX_PULSE) / 100.0f);
}

void Robot::release_servo(uint8_t pin) {
	softPwmStop(pin);
}

void Robot::beep(uint16_t ms, uint8_t pin) {
	digitalWrite(pin, HIGH);
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
	digitalWrite(pin, LOW);
}

void Robot::set(uint8_t pin, bool value) {
	digitalWrite(pin, value ? HIGH : LOW);
}

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

int Robot::distance_avg(uint8_t sensor_id, uint8_t num_measurements, float remove_percentage) {
	float arr[num_measurements];

	// take measurements
	for(int i = 0; i < sizeof(arr) / sizeof(arr[0]); i++) {
		float dist = distance(sensor_id);
		//std::cout << i << ": "<< dist << std::endl;
		arr[i] = dist;
	}

	// calculate avg after removing n percent of exteme measurements
	int arr_len = sizeof(arr) / sizeof(arr[0]);

	std::sort(arr, arr + arr_len);
	int kthPercent = (arr_len * remove_percentage);
	float sum = 0;

	for(int i = 0; i < arr_len; i++)
		if (i >= kthPercent && i < (arr_len - kthPercent))
			sum += arr[i];

	float avg = sum / (arr_len - 2 * kthPercent);

	//std::cout << "Avg: "<< avg << std::endl;
	return (int)avg;
}

void Robot::set_gpio(int pin, bool state) {
	digitalWrite(pin, state);
}
