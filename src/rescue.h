#pragma once

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
}