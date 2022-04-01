#include "silver_ml.h"
 
SilverML::SilverML() { 
    model = std::make_unique<fdeep::model>(fdeep::load_model("../runtime_data/silver.json"));
}

bool SilverML::predict_silver(cv::Mat p_image) {
    cv::Mat byte_image = p_image.clone();
    cv::cvtColor(byte_image, byte_image, cv::COLOR_BGR2RGB);

    cv::Mat image;
    byte_image.convertTo(image, CV_32FC3, 1.0f/255.0f);

    assert(image.isContinuous());

    cv::imshow("NN food", image);

    // Use the correct scaling, i.e., low and high.
    const auto input = fdeep::tensor_from_bytes(image.ptr(),
        static_cast<std::size_t>(image.rows),
        static_cast<std::size_t>(image.cols),
        static_cast<std::size_t>(image.channels()),
        0.0f, 1.0f);
    const auto result = model->predict_class_with_confidence({input});
    std::cout << result.first << "\t" << result.second << std::endl;
}
