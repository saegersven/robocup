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
	// if(softPwmCreate(SERVO_1, 0, 200)) {
	//  std::cout << "Error setting up PWM for Servo 1" << std::endl;
	//  exit(ERRCODE_BOT_SETUP_PWM);
	// }
	// if(softPwmCreate(SERVO_2, 0, 200)) {
	//  std::cout << "Error setting up PWM for Servo 2" << std::endl;
	//  exit(ERRCODE_BOT_SETUP_PWM);
	// }

	io_mutex.unlock();

	this->asc_speed_left = 0.0f;
	this->asc_speed_right = 0.0f;
	this->asc_has_duration = false;

	this->block_m = false;

	std::thread motor_update_thread(&Robot::asc, this); // Create async motor control thread
	motor_update_thread.detach();
}

int8_t API_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t len) {
	//std::cout << "W\t" << std::to_string(reg_addr) << "\t" << std::to_string(reg_data[0]) << std::endl;

	uint8_t write_buf[1 + len];
	write_buf[0] = reg_addr;
	for(int i = 0; i < len; ++i) {
		write_buf[i + 1] = reg_data[i];
	}

	if(write(dev_addr, write_buf, 1) != 1) {
		return 1;
	}
	// if(write(dev_addr, reg_data, len) != len) {
	//  return 1;
	// }
	// if(len == 1) {
	//  i2c_smbus_write_byte_data(dev_addr, reg_addr, reg_data[0]);
	//  return 0;
	// }

	// i2c_smbus_write_block_data(dev_addr, reg_addr, len, reg_data);
	return 0;
}

int8_t API_I2C_bus_write8(uint8_t dev_addr, uint8_t reg_addr, uint8_t reg_data) {
	//uint8_t write_buf[1] = {reg_data};
	//return API_I2C_bus_write(dev_addr, reg_addr, write_buf, 1);
	wiringPiI2CWriteReg8(dev_addr, reg_addr, reg_data);
}

int8_t API_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t len) {
	//std::cout << "R\t" << std::to_string(reg_addr) << std::endl;
	uint8_t write_buf[1] = {reg_addr};
	if(write(dev_addr, write_buf, 1) != 1) {
		return 1;
	}
	if(read(dev_addr, reg_data, len) != len) {
		return 1;
	}
	// if(len == 1) {
	//  reg_data = new uint8_t();
	//  reg_data[0] = i2c_smbus_read_byte_data(dev_addr, reg_addr);
	//  return 0;
	// }

	// i2c_smbus_read_block_data(dev_addr, reg_addr, reg_data);
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

	// bno055.bus_write = &API_I2C_bus_write;
	// bno055.bus_read = &API_I2C_bus_read;
	// bno055.delay_msec = &API_delay_msek;
	// bno055.dev_addr = bno_fd;

	// bno_comres = bno055_init(&this->bno055);

	// bno_comres += bno055_set_sys_rst(0x01);
	// delay(700);

	// bno_comres += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);

	// bno_comres += bno055_write_page_id(0x00);

	// uint8_t write_buf[1] = {0x00};
	// bno_comres += bno055_write_register(BNO055_SYS_TRIGGER_ADDR, write_buf, 1);

	// // Set accelerometer range
	// bno_comres += bno055_set_accel_range(BNO055_ACCEL_RANGE_2G);
	// bno_comres += bno055_set_gyro_range(BNO055_GYRO_RANGE_2000DPS);
	// bno_comres += bno055_set_mag_data_output_rate(0x05);

	// delay(10);

	// // write_buf[0] = BNO055_OPERATION_MODE_NDOF;
	// // bno_comres += bno055_write_register(BNO055_OPERATION_MODE_REG, write_buf, 1);

	// bno_comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

	// delay(10);

	// R       0
	// W       61      0
	// W       63      32
	// W       62      0
	// W       7       0
	// W       63      0
	// W       7       1
	// R       8
	// W       8       13
	// W       7       0
	// R       61
	// W       7       1
	// R       10
	// W       10      56
	// W       7       0
	// R       61
	// W       7       1
	// R       9
	// W       9       13
	// W       7       0
	// W       61      0
	// W       61      12


	uint8_t chip_id[1] = {0};
	API_I2C_bus_read(bno_fd, 0, chip_id, 1);

	API_I2C_bus_write8(bno_fd, 61, 0);
	delay(30);
	API_I2C_bus_write8(bno_fd, 63, 32); // SYS_TRIGGER bit 5: Reset
	delay(700);
	API_I2C_bus_write8(bno_fd, 62, 0); // PWR_MODE: Set power mode to 0 (Normal)
	API_I2C_bus_write8(bno_fd, 7, 0); // Set page id to 0
	API_I2C_bus_write8(bno_fd, 63, 0); // Reset system triggers to 0
	API_I2C_bus_write8(bno_fd, 7, 1); // Set page id to 1
	API_I2C_bus_write8(bno_fd, 8, 13); // Set accel range 4G and bandwidth 62_5HZ
	API_I2C_bus_write8(bno_fd, 7, 0); // Set page id to 0
	API_I2C_bus_write8(bno_fd, 7, 1); // Set page id to 1
	API_I2C_bus_write8(bno_fd, 10, 56); // Gyro config
	API_I2C_bus_write8(bno_fd, 7, 0); // Set page id to 0
	API_I2C_bus_write8(bno_fd, 7, 1); // Set page id to 1
	API_I2C_bus_write8(bno_fd, 9, 13); // Mag config
	API_I2C_bus_write8(bno_fd, 7, 0); // Set page id to 0
	delay(10);
	API_I2C_bus_write8(bno_fd, 61, 0); // Enter config mode
	delay(20);
	API_I2C_bus_write8(bno_fd, 61, 12); // Enter NDOF operation mode
	delay(10);

	std::cout << "Init complete" << std::endl;
}

float Robot::get_heading() {
	int16_t euler_data_h, euler_data_r, euler_data_p;
	euler_data_h = euler_data_r = euler_data_p = 0;

	double f_euler_data_h;
	float fl_euler_data_h;

	// bno_comres += bno055_read_euler_h(&euler_data_h);
	// bno_comres += bno055_read_euler_r(&euler_data_r);
	// bno_comres += bno055_read_euler_p(&euler_data_p);

	//bno_comres = 0;
	//auto start_time = std::chrono::high_resolution_clock::now();

	//bno_comres += bno055_convert_double_euler_h_rad(&f_euler_data_h);
	int16_t temp = 0;
	API_I2C_bus_read(bno_fd, 26, (uint8_t*)&temp, 2);

	fl_euler_data_h = (float)temp / 16.0f * PI / 180.0f;

	//std::cout << fl_euler_data_h << std::endl;

	//auto end_time = std::chrono::high_resolution_clock::now();

	//std::cout << "Reading took " <<
	//  std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << "ms" << std::endl;

	// bno_comres += bno055_convert_float_euler_r_rad(&f_euler_data_r);
	// bno_comres += bno055_convert_float_euler_p_rad(&f_euler_data_p);

	//std::cout << "X: " << std::to_string(rad_to_deg(f_euler_data_h)) << std::endl;

	return fl_euler_data_h;
}

// void Robot::delay_c(uint32_t ms, int id) {
//  if(!cams[id].cap.isOpened()) {
//      delay(ms);
//      return;
//  }
//  std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();
//  while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start) < ms) {
//      cams[cam_id].cap.grab();
//  }
// }

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
	if(block_m) return;

	left = clip(left, -100, 100);
	right = clip(right, -100, 100);

	// If duration is less than zero, flip motor pins
	if(duration < 0) {
		duration *= -1;
		left *= -1;
		right *= -1;
	}

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

	// Wait for duration and stop
	if(duration != 0) {
		//std::this_thread::sleep_for(std::chrono::milliseconds(duration));
		//delay_c(duration, this->front_cam_id);
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
	if(block_m) return;

	//m(100, -100, deg * TURN_DURATION_FACTOR);
	float start_heading = get_heading();
	float to_turn = std::abs(rad);

	bool clockwise = rad > 0.0;

	if(to_turn < 0.3) {
		m(clockwise ? -30 : 30, clockwise ? 30 : -30, TURN_DURATION_FACTOR * std::abs(rad_to_deg(rad)));
		return;
	}

	auto start_time = std::chrono::high_resolution_clock::now();
	uint32_t min_time = TURN_MIN_DURATION_FACTOR * std::abs(rad);
	uint32_t max_time = TURN_MAX_DURATION_FACTOR * std::abs(rad);

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
		//std::cout << rad_to_deg(new_heading) << " " << rad_to_deg(d_heading) << std::endl;
		//std::cout << rad_to_deg(new_heading) << " - " << rad_to_deg(start_heading) << " = " << rad_to_deg(std::abs(d_heading)) << std::endl;

		uint32_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();

		if((ms > min_time && std::abs(d_heading) >= to_turn - deg_to_rad(10.0f))) break;
	}
	stop();

	//std::cout << "Accuracy: " << rad_to_deg(std::abs(get_heading() - start_heading)) << "°" << std::endl;
}

void Robot::turn_to_heading(float heading) {
	if(block_m) return;
	std::cout << heading << std::endl;

	float curr_heading = get_heading();
	const float tolerance = deg_to_rad(5.0f);

	bool clockwise = false;
	float offset = 0.0f;
	if(heading > curr_heading) {
		if(heading - curr_heading > RAD_180) {
			clockwise = false;
			offset = RAD_360;
		} else {
			clockwise = true;
			offset = 0.0f;
		}
	} else {
		if(curr_heading - heading > RAD_180) {
			clockwise = true;
			offset = -RAD_360;
		} else {
			clockwise = false;
			offset = 0.0f;
		}
	}

	const float SPEED = 30;

	while(1) {
		if(clockwise) m(SPEED, -SPEED);
		else m(-SPEED, SPEED);

		curr_heading = get_heading() + offset;

		if((clockwise && curr_heading >= heading)
		  || (!clockwise && curr_heading <= heading))
			break;
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

		//std::cout << "E\t" << error << std::endl;

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

float Robot::single_distance(int8_t echo, uint8_t trig, int timeout) {
	timeout = timeout * 1000; // convert msec to usec for micros() function
	digitalWrite(trig, HIGH);
	delayMicroseconds(10);
	digitalWrite(trig, LOW);

	long start_time = micros();

	while (digitalRead(echo) == LOW && micros() - start_time < timeout);
	volatile long startTimeUsec = micros();
	while (digitalRead(echo) == HIGH);
	volatile long endTimeUsec = micros();

	long travelTimeUsec = endTimeUsec - startTimeUsec;

	// convert distance to cm, multiply with speed of sound (0,0343 cm/us) and divide by 2 to get one-way distance
	return ((travelTimeUsec / 10000.0f) * 340.29f * 0.5f);
}

float Robot::distance_avg(uint8_t echo, uint8_t trig, uint8_t measurements, float remove_percentage, uint32_t timeout_single_measurement, uint32_t timeout) {
	float arr[measurements];

	// take measurements
	for(int i = 0; i < sizeof(arr) / sizeof(arr[0]); i++) {
		float dist = single_distance(echo, trig, timeout_single_measurement);
		//std::cout << i << ": "<< dist << std::endl;
		arr[i] = dist;
		delay(5);
	}

	// calculate avg after removing n percent of exteme measurements
	int arr_len = sizeof(arr) / sizeof(arr[0]);

	std::sort(arr, arr + arr_len);
	int kthPercent = (arr_len * remove_percentage);
	float sum = 0;

	for (int i = 0; i < arr_len; i++)
		if (i >= kthPercent && i < (arr_len - kthPercent))
			sum += arr[i];

	float avg = sum / (arr_len - 2 * kthPercent);

	//std::cout << "Avg: "<< avg << std::endl;

	return avg;
}

void Robot::set_gpio(int pin, bool state) {
	digitalWrite(pin, state);
}
