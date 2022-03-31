#include <fdeep/fdeep.hpp>
#include <opencv2/opencv.hpp>

bool predict_silver(cv::Mat p_image)
{
    cv::cvtColor(p_image, p_image, cv::COLOR_BGR2RGB);

    cv::Mat image;
    p_image.convertTo(image, CV_32FC3, 1.0f/255.0f);

    assert(image.isContinuous());
    const auto model = fdeep::load_model("../runtime_data/silver.json");
    // Use the correct scaling, i.e., low and high.
    /*const auto input = fdeep::tensor_from_bytes(image.ptr(),
        static_cast<std::size_t>(image.rows),
        static_cast<std::size_t>(image.cols),
        static_cast<std::size_t>(image.channels()),
        0.0f, 1.0f);
    const auto result = model.predict_class({input});
    std::cout << result << std::endl;*/
}
