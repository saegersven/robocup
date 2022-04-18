#include "silver_ml.h"
 
SilverML::SilverML() { 
    model = tflite::FlatBufferModel::BuildFromFile("/home/pi/robocup/runtime_data/silver.tflite");
    tflite::ops::builtin::BuiltinOpResolver resolver;
    tflite::InterpreterBuilder builder(*model, resolver);
    builder(&interpreter);
    interpreter->AllocateTensors();
}

bool SilverML::predict_silver(cv::Mat p_image) {
    cv::Mat byte_image = p_image.clone();
    cv::cvtColor(byte_image, byte_image, cv::COLOR_BGR2RGB);

    cv::Mat image;
    byte_image.convertTo(image, CV_32FC3, 1.0f/255.0f);

    assert(image.isContinuous());

    float* input_layer = interpreter->typed_input_tensor<float>(0);
    float* output_layer = interpreter->typed_output_tensor<float>(0);

    float* img_ptr = image.ptr<float>(0);
    memcpy(input_layer, img_ptr.ptr<float>(0),
        in.cols * in.rows * in.channels() * sizeof(float));

    interpreter->Invoke();

    std::cout << "NN says: " << output_layer[0] << "\t" << output_layer[1] << std::endl;
    return output_layer[0] < output_layer[1];
}
