#pragma once

#define ARM_UP 80
#define ARM_DOWN -95
#define ARM_DROP -50

#define GRAB_OPEN -20
#define GRAB_CLOSED -100

#define REFLECTANCE_THRESHOLD_VICTIM 1000

// Corner positions
#define CORNER_POS_SHORT_LEFT 0
#define CORNER_POS_SHORT_RIGHT 1
#define CORNER_POS_LONG_LEFT 2
#define CORNER_POS_LONG_RIGHT 3

#define WALL_LONG(var) (var & 0x02)
#define WALL_RIGHT(var) (var & 0x01)

enum VictimType {
	ALIVE,
	DEAD
}

struct Victim {
	float screen_size;
	float x_pos, y_pos;
	VictimType type;
}

class Rescue {
private:
	std::mutex rescue_mutex;

	Robot* robot;
	std::thread::native_handle_type native_handle;

	void rescue();

public:
	Rescue();
	void start();
	void stop();
};