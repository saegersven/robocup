#include "line.h"

Line::Line(int front_cam_id, Robot* robot, NeuralNetworks neural_networks) {
	this->front_cam_id = front_cam_id;
	this->robot = robot;
	this->neural_networks = neural_networks

	reset();
}

void Line::start() {
	// Start video feed from front camera
	robot->start_video(front_cam_id);

	running = true;
}

void Line::stop() {
	// Stop video if running
	if(!running) return;

	robot->stop_video(front_cam_id);
	running = false;
}

void Line::line(cv::Mat& frame) {
	//cv::Mat frame = robot->capture(front_cam_id); // Retrieve video frame
	cv::Mat green_cut = frame(cv::Range(10, 70), cv::Range(8, 40));

	// TODO: Green color values and threshold
	if(pixel_count_over_threshold_hue(green_cut, 110, 140, 40, 1000)) {
		// Consult the mighty AI
		float confidence = 0.0f;
		switch(neural_networks.infere(GREEN_NN_ID, frame, confidence)) {
			case GREEN_RESULT_LEFT:
				// TODO: Go left
				std::cout << "Left (" << std::to_string(confidence) << ")" << std::endl;
				break;
			case GREEN_RESULT_RIGHT:
				// TODO: Go right
				std::cout << "Right (" << std::to_string(confidence) << ")" << std::endl;
				break;
			case GREEN_RESULT_DEAD_END:
				// TODO: Turn around
				std::cout << "Dead End (" << std::to_string(confidence) << ")" << std::endl;
				break;
		}
	}
}

cv::Mat Line::inRange_green(cv::Mat& in) {
	int rows = in.rows;
	int cols = in.cols;

	uint8_t* p;
	cv::Mat out(rows, cols, cv::CV_8UC1);

	int i, j;
	for(i = 0; i < rows; ++i) {
		p = in.ptr<uint8_t>(i);
		p_out = out.ptr<uint8_t>(i);
		for(j = 0; j < cols; ++j) {
			float sum = (float)p[j][0] + (float)p[j][2];
			if(sum == 0.0f) continue;
			ratio = (float)p[j][1] / sum;

			p_out[j] = ratio > 0.85f && p[j][1] > 40 ? 0xFF : 0x00;
		}
	}
	return out;
}

// TODO: Find better values
uint8_t Line::green(cv::Mat& frame) {
	cv::Mat green = inRange_green(frame);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(green, contours, cv::RETR_TREE, cv::CV_CHAIN_APPROX_SIMPLE);

	for(int i = 0; i < contours.size(); ++i) {
		cv::RotatedRect r = cv::minAreaRect(contours[i]);
		// Only consider contour if it is big enough
		if(r.size.width * r.size.height < 40.0f) continue;

		float initial_angle = deg_to_rad(r.angle);
		if(initial_angle > deg_to_rad(45.0f)) initial_angle -= deg_to_rad(360.0f);
		float angle = initial_angle;

		std::vector<cv::Point2f> black_points;
		black_points.reserve(30);

		for(int x = 0; x < 30; ++x) {
			angle = deg_to_rad(360.0f / 30.0f * x);

			float sin = std::sin(angle);
			float cos = std::cos(angle);

			float cx = r.center.x + sin * (r.size.width * 0.75f);
			float cy = r.center.y - cos * (r.size.height * 0.75f);

			for(int y = 0; y < 4; ++y) {
				cx += sin * y * 0.5f;
				cy -= cos * y * 0.5f;

				cv::Vec3b pix = frame.at<cv::Vec3b>((int)cy, (int)cx);
				float lightness = ((float)pix[0] + (float)pix[1] + (float)pix[2]) / 3.0f;
				float saturation = std::max(pix[0], pix[1], pix[2]) - std::min(pix[0], pix[1], pix[2]);
				if(lightness <= 20.0f && saturation <= 20.0f) {
					black_points.push_back(cv::Point2f(cx, cy));
					break;
				}
			}
		}

		if(black_points.size() > 5) {
			float mean_x, mean_y;
			for(cv::Point2f p : black_points) {
				mean_x += p.x;
				mean_y += p.y;
			}

			mean_x /= black_points.size();
			mean_y /= black_points.size();

			float relative_black_angle = std::atan2(r.center.y - mean_y, r.center.x - mean_x) - initial_angle;

			if(relative_black_angle > 90.0f && relative_black_angle < 90.0f) {
				// If angle is positive, green is left
				green_points |= relative_black_angle > 0.0f ? 0x01 : 0x02;
			}
		}

		// // For each contour, compute the position relative to the nearest line
		// // by checking the colors at the edges of the min area rects
		// uint8_t black = 0;
		// float angle = deg_to_rad(r.angle);
		// float cx, cy;
		// for(int x = 0; x < 4; ++x) {
		// 	angle += deg_to_rad(90.0f);
		// 	cx = r.center.x + std::cos(angle) * r.size.width * check_distance_factor;
		// 	cy = r.center.y + std::sin(angle) * r.size.height * check_distance_factor;
		// 	black |= (bgr_to_lightness(frame.at<cv::Vec3b>((int)cx, (int)cy)) < 30 ? 0x01 : 0x00) << x;
		// }

		// switch(black) {
		// 	case 0b00001001:
		// 		// Bottom left, bit for line right (1st) and line top (4th) are set
		// 		green_points |= 0x01; // Set first bit
		// 		break;
		// 	case 0b00001100:
		// 		// Bottom right, bit for line left (3rd) and line top (4th) are set
		// 		green_points |= 0x02; // Set second bit
		// 		break;
		// 	default:
		// 		// Ignore green points at the top
		// }
	}
	// If both left and right green is there, return value is 3
	// If only left or right, return value is 1 or 2
	// If no contours were at the bottom, return value is 0
	return green_points;
}