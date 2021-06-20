#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <cmath>

#define DEBUG

const cv::Range LINE_ROI_X(20, 300);
const cv::Range LINE_ROI_Y(100, 192);

int main() {
	// Initialize Camera
	cv::VideoCapture cap(0);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 192);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
	cap.set(cv::CAP_PROP_FPS, 100);

	if(!cap.isOpened()) {
		std::cerr << "Could not open camera" << std::endl;
		return 1;
	}

	cv::Mat img, line_roi, line_thr;

	int16_t last_line_pos = 0;

#ifdef DEBUG
	cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Line ROI", cv::WINDOW_AUTOSIZE);

	uint32_t num_frames = 0;
#endif

	auto time = std::chrono::system_clock::now();
	// Capture loop
	while(1) {

		cap.grab();
		cap.retrieve(img);

		if(img.empty()) {
			std::cerr << "Error grabbing frame from camera" << std::endl;
			return 2;
		}

		cv::GaussianBlur(img, img, cv::Size(3, 3), 2);
		cv::cvtColor(img, img, cv::COLOR_BGR2HSV);

		line_roi = img(LINE_ROI_Y, LINE_ROI_X);
		cv::inRange(line_roi, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 75), line_thr);

		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(line_thr, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

		int16_t line_pos = 0;

		if(contours.size() > 0) {
			int16_t nearest = 1000;
			int16_t line_width = 0;

			for(size_t i = 0; i < contours.size(); i++) {
				cv::Rect b = cv::boundingRect(contours[i]);

				int16_t pos = b.x + b.width / 2 - (LINE_ROI_X.end - LINE_ROI_X.start) / 2;
				int16_t dist = std::abs(pos - last_line_pos);
				if(dist < nearest) {
					line_pos = pos;
					line_width = b.width;
					nearest = dist;
				}
			}

			last_line_pos = line_pos;
		}
#ifdef DEBUG
		int16_t debug_line_pos = line_pos + line_roi.cols / 2;
		cv::line(line_roi, cv::Point(debug_line_pos, 0),
			cv::Point(debug_line_pos, line_roi.rows), cv::Scalar(0, 0, 255), 2);

		for(size_t i = 0; i < contours.size(); i++) {
			cv::drawContours(line_roi, contours, (int)i,
				cv::Scalar(0, 255, 255), 2, cv::LINE_8, hierarchy, 0);
		}

		cv::imshow("Line ROI", line_roi);

		cv::imshow("Video", img);
		++num_frames;

		if(cv::waitKey(5) >= 0)
			break;
#else
		auto t = std::chrono::system_clock::now();
		auto dt = t - time;
		time = t;

		//std::cout << 1.0f / dt.count() * 1000000000.0f << "fps | " << line_pos << std::endl;
		printf("%3.1ffps | %4d\n", 1.0f / dt.count() * 1000000000.0f, line_pos);
#endif
	}

	return 0;
}