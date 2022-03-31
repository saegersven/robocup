#include <fdeep/fdeep.hpp>
#include <opencv2/opencv.hpp>

bool predict_silver(cv::Mat roi)
{
    cv::cvtColor(roi, roi, cv::COLOR_BGR2RGB);
    assert(roi.isContinuous());
    const auto model = fdeep::load_model("model.json");
    // Use the correct scaling, i.e., low and high.
    const auto input = fdeep::tensor_from_bytes(image.ptr(),
        static_cast<std::size_t>(image.rows),
        static_cast<std::size_t>(image.cols),
        static_cast<std::size_t>(image.channels()),
        0.0f, 1.0f);
    const auto result = model.predict_class({input});
    std::cout << result << std::endl;
}
