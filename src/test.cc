#include "test.h"

void test_motors(std::shared_ptr<Robot> bot) {
	bot->m(100, -100, 200);
	bot->m(-100, 100, 200);
	bot->m(50, 50, 500);
	bot->m(-50, -50, 500);
}

void test_servo(std::shared_ptr<Robot> bot, uint8_t pin) {
	bot->servo(pin, 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	bot->servo(pin, -60);
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	bot->servo(pin, 60);
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	bot->servo(pin, 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
}

void test_camera(std::shared_ptr<Robot> bot, int hardware_id) {

}

void test_button(std::shared_ptr<Robot> bot, uint8_t pin) {
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	while(!bot->button(pin)) {
		std::cout << ".";
	}
	std::cout << " Press" << std::endl;
}

void test_distance(std::shared_ptr<Robot> bot, uint8_t echo, uint8_t trig) {
	for(int i = 0; i < 5; i++) {
		std::cout << std::to_string(bot->distance(echo, trig, 1)) << std::endl;
		std::cout << std::to_string(bot->distance(echo, trig, 10)) << std::endl;
		std::cout << std::to_string(bot->distance(echo, trig, 100)) << std::endl;
	}
}