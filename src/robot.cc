#include "robot.hpp"

Robot::Robot() {
	io_mutex.lock();
	// Initialize the robot
	// Setup GPIO
	wiringPiSetupPhys();

	// Setup I2C
	mcp_fd = wiringPiI2CSetup(0x20); // TODO: Check if this is the actual ID of MCP23017
	if(mcp_fd == -1) {
		std::cout << "Error setting up I2C for Encoder board" << std::endl;
		exit(ERRCODE_BOT_SETUP_I2C);
	}

	// Setup Drive motor pins
	pinMode(M1_1, OUTPUT);
	pinMode(M1_2, OUTPUT);
	pinMode(M2_1, OUTPUT);
	pinMode(M2_2, OUTPUT);

	// Create PWM for the two drive motors
	if(softPwmCreate(M1_E, 0, 100)) {
		std::cout << "Error setting up PWM for M1" << std::endl;
		exit(ERROCDE_BOT_SETUP_PWM);
	}
	if(softPwmCreate(M2_E, 0, 100)) {
		std::cout << "Error setting up PWM for M2" << std::endl;
		exit(ERROCDE_BOT_SETUP_PWM);
	}
	io_mutex.unlock();

	motor_update_thread(asc); // Create async motor control thread
	motor_update_thread.detach();
}

int Robot::init_camera(int id, bool calibrated = false, int width = 80, int height = 48, int fps = 60) {
	Camera cam(id, calibrated, width, height, fps);

	cams.push_back(cam);
	return cams.size() - 1;
}

cv::Mat Robot::capture(int cam_id, bool undistort = false) {
	if(cams[cam_id].cap.isOpened()) {
		return cams[cam_id].retrieve_frame(undistort);
	} else {
		return cams[cam_id].single_capture(undistort);
	}
}

void Robot::start_video(int cam_id) {
	cams[cam_id].start_video();
}

void Robot::stop_video(int cam_id) {
	cams[cam_id].stop_video();
}

void Robot::m(int8_t left, int8_t right, int16_t duration = 0) {
	left = clip(left, -100, 100);
	right = clip(right, -100, 100);

	io_mutex.lock();

	// Reconfigure the H-Bridge's direction control pins
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
		stop();
	}
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

void m_asc(int8_t left, int8_t right, int16_t duration = 0) {
	asc_speed_left = left;
	asc_speed_right = right;
}

void asc() {
	uint8_t last_encoder_value_left = encoder_value_a();
	uint8_t last_encoder_value_right = encoder_value_b();

	int8_t real_speed_left = 0;
	int8_t real_speed_right = 0;

	int8_t real_speed_offset_left = 0;
	int8_t real_speed_offset_right = 0;

	std::chrono::time_point last_update = std::chrono::high_resolution_clock::now();
	while(1) {
		if(asc_speed_left == 0.0f && asc_speed_right == 0.0f) return;

		int8_t speed_sign_left = asc_speed_left < 0.0f ? -1 : 1;
		int8_t speed_sign_left = asc_speed_right < 0.0f ? -1 : 1;
		
		real_speed_left = asc_speed_left * MOTOR_SPEED_CONVERSION_FACTOR;
		real_speed_right = asc_speed_right * MOTOR_SPEED_CONVERSION_FACTOR;

		std::chrono::time_point now = std::chrono::high_resolution_clock::now();
		float delta_t = std::chrono::duration_cast<std::chrono::seconds>(
			now - last_update);
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

		real_speed_offset_left += (int8_t)left_error * 10.0f;
		real_speed_offset_right += (int8_t)right_error * 10.0f;

		m(real_speed_left + real_speed_offset_left, real_speed_right + real_speed_offset_right);
	}
}

/*// Speed in mm/s
void Robot::set_motor_speed(int8_t left, int8_t right, int16_t duration) {
	int8_t speed_l = clip(left / WHEEL_CIRCUMFERENCE / 10
		* PULSES_PER_REVOLUTION * GEAR_RATIO * MOTOR_SPEED_CONVERSION_FACTOR, 0, 100);
	int8_t speed_r = clip(right / WHEEL_CIRCUMFERENCE / 10
		* PULSES_PER_REVOLUTION * GEAR_RATIO * MOTOR_SPEED_CONVERSION_FACTOR, 0, 100);

	m(initial_sped_l, initial_speed_r);
	if(duration != 0) auto start_time = std::chrono::system_clock::now();

	while(duration == 0 || std::chrono::duration_cast<std::chrono::milliseconds>
		(std::chrono::system_clock::now() - start_time).count() < duration) {

	}
	stop();
}*/

// Go in a straight line
void Robot::drive_distance(float distance, int8_t speed = 100) {
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
}

bool Robot::button(uint8_t pin) {
	io_mutex.lock();
	bool button_down = digitalRead(pin) == HIGH;
	io_mutex.unlock();
	return button_down;
}

void Robot::button_wait(uint8_t pin, bool state = true, uint32_t timeout = 0xffffffff) {
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

void Robot::stop() {
	io_mutex.lock();
	digitalWrite(M1_1, LOW);
	digitalWrite(M1_2, LOW);
	digitalWrite(M2_1, LOW);
	digitalWrite(M2_2, LOW);

	softPwmWrite(M1_E, 0);
	softPwmWrite(M2_E, 0);
	io_mutex.unlock();
}

uint16_t Robot::distance(uint8_t echo, uint8_t trig, uint16_t iterations = 1) {
	float timeElapsed = 0.0f;
	io_mutex.lock();
	for(uint16_t i = 0; i < iterations; i++) {
		digitalWrite(trig, HIGH);
		std::this_thread::sleep_for(std::chrono::microseconds(100));
		digitalWrite(trig, LOW);

		while (digitalRead(echo) == LOW) {
			auto startTime = std::chrono::system_clock::now();
		}

		while (digitalRead(echo) == HIGH) {
			auto stopTime = std::chrono::system_clock::now();
		}

		timeElapsed += std::chrono::duration_cast<std::chrono::milliseconds>(stopTime - startTime).count();
		if(i != 0) timeElapsed /= 2.0f;
	}
	io_mutex.unlock();
	// Multiply with speed of sound (34,3 cm/ms) and divide by 2 to get one-way distance
	return (timeElapsed * 34.3f) / 2
}