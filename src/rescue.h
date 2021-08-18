#pragma once

class Rescue {
private:
	Robot* robot;
	std::thread::native_handle_type native_handle;

	rescue();

public:
	Rescue();
	start();
	stop();
}