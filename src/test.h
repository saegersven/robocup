#pragma once

#include <chrono>

#include "robot.h"

void test(std::shared_ptr<Robot> bot) {
#ifdef TEST_MOTORS
	test_motors(bot);
#endif
#ifdef TEST_SERVO_1
	test_servo(bot, SERVO_1);
#endif
#ifdef TEST_SERVO_2
	test_servo(bot, SERVO_2);
#endif
#ifdef TEST_BUTTON_RESTART
	test_button(bot, BTN_RESTART);
#endif
#ifdef TEST_BUTTON_DEBUG
	test_button(bot, BTN_DEBUG);
#endif
#ifdef TEST_DISTANCE
	test_distance(bot, DIST_1);
#endif
}

void test_motors(std::shared_ptr<Robot> bot);

void test_servo(std::shared_ptr<Robot> bot, uint8_t pin);

void test_camera(std::shared_ptr<Robot> bot, int hardware_id);

void test_button(std::shared_ptr<Robot> bot, uint8_t pin);

void test_distance(std::shared_ptr<Robot> bot, uint8_t echo, uint8_t trig);