#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <cmath>

#define M1_1 16
#define M1_2 18
#define M1_E 22
#define M2_1 11
#define M2_2 12
#define M2_E 7

#define BTN_RESTART 36
#define BTN_DEBUG 35

#define DIST_1 32, 33
#define DIST_2 29, 31

#define FORWARD_CORRECTION_FACTOR 0.1f
// cm
#define WHEEL_CIRCUMFERENCE 5.0f
#define GEAR_RATIO 100.0f
#define PULSES_PER_REVOLUTION 20.0f

#define ENCODER_PORTA 0x12
#define ENCODER_PORTB 0x13

class Robot {
private:
	int mcp_fd;

	uint8_t encoder_value_a();
	uint8_t encoder_value_b();

public:
	Robot();

	void m(int8_t left, int8_t right, int16_t duration);
	void drive_distance(float distance, int8_t speed = 100);
	void stop();
	void turn(int8_t degrees);

	void servo();
	
	void button(uint8_t pin);
	uint16_t distance(uint8_t echo, uint8_t trig);
}

float clip(float n, float lower, float upper) {
	return std::max(lower, std::min(n, upper));
}