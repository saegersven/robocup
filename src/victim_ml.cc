#include "victim_ml.h"
 
VictimML::VictimML() {
    std::cout << "Loading victim NN" << std::endl;
    model = tflite::FlatBufferModel::BuildFromFile("/home/pi/robocup/runtime_data/victim.tflite");
    tflite::ops::builtin::BuiltinOpResolver resolver;
    tflite::InterpreterBuilder builder(*model, resolver);
    builder(&interpreter);
    interpreter->AllocateTensors();

    input_layer = interpreter->typed_input_tensor<float>(0);
    output_layer = interpreter->typed_output_tensor<float>(0);
    std::cout << "Done" << std::endl;
}

cv::Mat VictimML::invoke(cv::Mat image) {
    // Conversion from byte & BGR to normalized float & RGB is done later in the copying for loop
    //cv::cvtColor(byte_image, byte_image, cv::COLOR_BGR2RGB);

    //cv::Mat image;
    //byte_image.convertTo(image, CV_32FC3, 1.0f/255.0f);

    // INPUT IMAGE IS 640x480 BGR image
    // Convert to 160x120 grayscale and flatten onto input layer

    uint32_t channels = image.channels();

    cv::Vec3b* p;
    int i, j;
    for(i = 0; i < image.rows; i += 4) {
        p = image.ptr<cv::Vec3b>(i);
        for(j = 0; j < image.cols; j += 4) {
            input_layer[i * image.cols + j] = ((float)p[j][0] + p[j][1] + p[j][2]) / 3.0f;
        }
    }

    interpreter->Invoke();

    const uint32_t OUT_WIDTH = 40;
    const uint32_t OUT_HEIGHT = 12;

    // Map output layer to cv::Mat
    cv::Mat out(OUT_HEIGHT, OUT_WIDTH, CV_32FC1);

    float* p;
    int i, j;
    for(i = 0; i < OUT_HEIGHT; ++i) {
        p = out.ptr<float>(i);
        for(j = 0; j < OUT_WIDTH; ++j) {
            float val = output_layer[i * OUT_WIDTH + j];
            if(val > 1.0f) val == 1.0f;
            else if(val < 0.0f) val == 0.0f;
            p[j] = val;
        }
    }
    return out;
}