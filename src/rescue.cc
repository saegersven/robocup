#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "robot.h"
#include "rescue.h"
#include "utils.h"

void Rescue::rescue() {
	// Gets called once inside the evacuation zone

	// Determine entrance position
	float distance_right = robot->distance(DIST_1, 10);
	float distance_front = robot->distance(DIST_2, 10);

	uint8_t corner_pos = 0x00;
	corner_pos |= distance_front > 90.0f ? 0x02 : 0x00;
	corner_pos |= distance_right < 40.0f ? 0x01 : 0x00;

	// Turn around and search for first victim
	robot->turn(RIGHT_WALL(corner_pos) ? RAD_180 : -RAD_180);

	float start_heading = robot->get_heading();

	bool ignore_dead = true;
	while(1) {
		const uint32_t num_steps = 4;
		const float total_angle = RAD_90;

		// Flip sign to avoid hitting wall
		float turn_sign = RIGHT_WALL(corner_pos) ? -1.0f : 1.0f;

		if(!ignore_dead) {
			// When no living victim was found, we are turned 90°
			// Flip sign to turn back while searching for dead victims
			turn_sign = -turn_sign;
		}

		bool found_victim = false;
		for(int i = 0; i < num_steps; ++i) {
			// Search for living victim
			// find_victim includes search and collection of the victim
			// When it returns true, start next turn and search
			if(find_victim(ignore_dead)) {
				found_victim = true;
				break;
			}
			robot->turn(turn_sign * total_angle / num_steps);
		}
		
		if(!found_victim) {
			if(ignore_dead) {
				// There are no living victims left
				ignore_dead = false;
			} else {
				// There are no victims left, we are done
				break;
			}
		}
	}

	// Search for exit
}

// drives to victim and picks it up
void rescue_victim() {

}

/**
 * returns reflectance of victim to distinguish between "living" and "dead"
 *
 * @param current camera frame
 * @return reflectance of victim in image (low reflectance -> dead victim)
 *
*/
int victim_reflectance(cv::Mat& frame) {
	return 42;
}

/**
 * checks if there are round contours (-> victim) in the given frame
 *
 * @param current camera frame
 * @return boolean value
 *
*/
bool victim_in_frame(cv::Mat& frame) {
	cv::GaussianBlur(frame, image, Size(5, 5), 1.0); // adjust params
	
	cv::cvtColor(image, cv::Mat gray, cv::COLOR_BGR2GRAY);

	std::vector<Vec3f> circles;
    HoughCircles(gray, circles, HOUGH_GRADIENT, 2, gray.rows/4, 200, 100); // old Python parameters: minDist = 60, param1 = 34, param2 = 24, minRadius = 2, maxRadius = 300

	return circles.size() != 0;
}

// searches the corner and drops victim
void find_corner() {

}