#include "robot.hpp"

Robot::Robot() {
	// Initialize the robot
	wiringPiSetupPhys();
	pinMode(M1_1, OUTPUT);
	pinMode(M1_2, OUTPUT);
	pinMode(M2_1, OUTPUT);
	pinMode(M2_2, OUTPUT);

	// Create PWM for the two drive motors
	if(softPwmCreate(M1_E, 0, 100)) {
		std::cerr << "Error setting up PWM for M1" << std::endl;
	}
	if(softPwmCreate(M2_E, 0, 100)) {
		std::cerr << "Error setting up PWM for M2" << std::endl;
	}
}

void Robot::m(int8_t left, int8_t right, int16_t duration) {
	left = clip(left, -100, 100);
	right = clip(right, -100, 100);

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
		std::this_thread::sleep_for(std::chrono::milliseconds(duration));
		stop();
	}
}

uint16_t retrieve_encoder_values() {
	encoder_values = 0;
}

void set_encoder_anchor() {
	encoder_anchor = encoder_values;
}

uint8_t encoder_value_a() {
	// Return eight lest significant bits for a
	return (uint8_t)(encoder_values - encoder_anchor);
}

uint8_t encoder_value_b() {
	// Return eight most significant bits for b
	return (uint8_t)((encoder_values - encoder_anchor) >> 8);
}

// Go in a straight line
void Robot::forward(float distance, int8_t speed = 100) {
	int8_t sign = distance < 0 ? -1 : 1;

	float distance_travelled = 0.0f;

	while(distance_travelled < distance) {
		retrieve_encoder_values();
		// encoder_value_x() gives us 0 now

		// Difference in pulses
		uint8_t d_encoder_a = encoder_value_a();
		uint8_t d_encoder_b = encoder_value_b();
		
		// Average encoder deltas to calculate distance travelled
		distance_travelled += (d_encoder_a + d_encoder_b) / 2.0f *
			PULSES_PER_REVOLUTION * GEAR_RATIO * WHEEL_CIRCUMFERENCE

		int16_t error = d_encoder_b - d_encoder_a;

		m(sign * (speed + error * FORWARD_CORRECTION_FACTOR),
			sign * (speed - error * FORWARD_CORRECTION_FACTOR), 0);
		
		set_encoder_anchor(); // Reset encoder to zero
	}

	stop();
}

void Robot::button(uint8_t pin) {
	return digitalRead(pin) == HIGH;
}

void Robot::button_wait(uint8_t pin, uint32_t timeout = 0) {
	if(timeout != 0) {
		auto start_time = std::chrono::system_clock::now();
		while(!Robot::button(pin)) {
			auto now = std::chrono::system_clock::now();
			if(std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() >= timeout) {
				return;
			}
		}
	}
	while(!Robot::button(pin)) {}
}

void Robot::stop() {
	digitalWrite(M1_1, LOW);
	digitalWrite(M1_2, LOW);
	digitalWrite(M2_1, LOW);
	digitalWrite(M2_2, LOW);

	softPwmWrite(M1_E, 0);
	softPwmWrite(M2_E, 0);
}

uint16_t Robot::distance(uint8_t echo, uint8_t trig) {
	digitalWrite(trig, HIGH);
	std::this_thread::sleep_for(std::chrono::milliseconds(0.00001));
	digitalWrite(trig, LOW);

	while (digitalRead(echo) == LOW) {
		auto startTime = std::chrono::system_clock::now();
	}

	while (digitalRead(echo) == HIGH) {
		auto stopTime = std::chrono::system_clock::now();
	}

	uint16_t timeElapsed = stopTime - startTime

	// multiply with acoustic velocity (34300 cm/s) and divide by 2
	return = (timeElapsed * 34300) / 2
}