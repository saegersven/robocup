#include <iostream>
#include <wiringPi.h>
#include <cmath>

#define M1_1 0
#define M1_2 0
#define M1_E 0
#define M2_1 0
#define M2_2 0
#define M2_E 0

class Robot {
	Robot();

	void m(int8_t left, int8_t right, int16_t duration);
	void stop();
	void turn(int8_t degrees);
	void servo();
}

float clip(float n, float lower, float upper) {
	return std::max(lower, std::min(n, upper));
}