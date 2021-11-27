#pragma once

#define ARM_UP 80
#define ARM_DOWN -95
#define ARM_DROP -50

#define GRAB_OPEN -20
#define GRAB_CLOSED -100

#define REFLETCANCE_THRESHOLD_VICTIM 1000

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