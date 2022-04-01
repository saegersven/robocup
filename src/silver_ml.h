#pragma once

#include <memory>

#include <fdeep/fdeep.hpp>
#include <opencv2/opencv.hpp>

class SilverML {
private:
	std::unique_ptr<fdeep::model> model;

public:
	SilverML();
	bool predict_silver(cv::Mat image);
};