#include <iostream>
#include <wiringPi.h>
#include <cmath>

#define M1_1 16
#define M1_2 18
#define M1_E 22
#define M2_1 11
#define M2_2 12
#define M2_E 7

#define BTN_RESTART 36
#define BTN_DEBUG 35

#define DIST_1 33, 32
#define DIST_2 31, 29

class Robot {
	Robot();

	void m(int8_t left, int8_t right, int16_t duration);
	void stop();
	void turn(int8_t degrees);
	void servo();
	void button(uint8_t pin);
	uint16_t distance();
}

float clip(float n, float lower, float upper) {
	return std::max(lower, std::min(n, upper));
}