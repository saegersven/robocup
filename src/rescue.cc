#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "robot.h"
#include "rescue.h"
#include "utils.h"

// drives to victim and picks it up
void rescueVictim() {

}

/**
 * returns reflectance of victim to distinguish between "living" and "dead"
 *
 * @param current camera frame
 * @return reflectance of victim in image (low reflectance -> dead victim)
 *
*/
int victimReflectance(cv::Mat& frame) {
	return 42;
}

/**
 * checks if there are round contours (-> victim) in the given frame
 *
 * @param current camera frame
 * @return boolean value
 *
*/
bool victimInFrame(cv::Mat& frame) {
	cv::GaussianBlur(frame, image, Size(5, 5), 1.0); // adjust params
	
	cv::cvtColor(image, cv::Mat gray, cv2::COLOR_BGR2GRAY);

	vector<Vec3f> circles;
    HoughCircles(gray, circles, HOUGH_GRADIENT, 2, gray.rows/4, 200, 100); // old Python parameters: minDist = 60, param1 = 34, param2 = 24, minRadius = 2, maxRadius = 300

	if(circles.size() != 0) {
		return true;
	} else {
		return false;
	}
}

// searches the corner and drops victim
void searchBlackCorner() {

}