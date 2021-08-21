#pragma once
#include "utils.h"

class Line {
private:
	int front_cam_id;
	GreenModel green_model;
	Robot* robot;

	bool running = false;

public:
	Line(int front_cam_id, Robot* robot);
	void start();
	void stop();
	void line();
}