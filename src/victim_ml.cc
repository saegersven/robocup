#include "victim_ml.h"
 
VictimML::VictimML() {
    std::cout << "Loading victim NN" << std::endl;
    model = tflite::FlatBufferModel::BuildFromFile("/home/pi/robocup/runtime_data/victim.tflite");
    tflite::ops::builtin::BuiltinOpResolver resolver;
    tflite::InterpreterBuilder builder(*model, resolver);
    if(builder(&interpreter) != kTfLiteOk) {
        std::cout << "Failed building interpreter" << std::endl;
    }

    if(interpreter->AllocateTensors() != kTfLiteOk) {
        std::cout << "Failed allocating tensors" << std::endl;
    }

    input_layer = interpreter->typed_input_tensor<float>(0);

    std::cout << input_layer << std::endl;
    std::cout << output_layer << std::endl;

    std::cout << "Done" << std::endl;
}

cv::Mat VictimML::invoke(cv::Mat image) {
    // Conversion from byte & BGR to normalized float & RGB is done later in the copying for loop
    //cv::cvtColor(byte_image, byte_image, cv::COLOR_BGR2RGB);

    //cv::Mat image;
    //byte_image.convertTo(image, CV_32FC3, 1.0f/255.0f);

    // INPUT IMAGE IS 640x480 BGR image
    // Convert to 160x120 grayscale and flatten onto input layer

    //interpreter->AllocateTensors();
    //input_layer = interpreter->typed_input_tensor<float>(0);
    //output_layer = interpreter->typed_output_tensor<float>(0);

    cv::resize(image, image, cv::Size(160, 120));

    for(int i = 0; i < image.rows; ++i) {
        cv::Vec3b* p = image.ptr<cv::Vec3b>(i);
        for(int j = 0; j < image.cols; ++j) {
            float d = ((float)p[j][0] + p[j][1] + p[j][2]) / 3.0f;
            input_layer[i * image.cols + j] = d;
        }
    }

    std::cout << "Invoking" << std::endl;
    interpreter->Invoke();
    output_layer = interpreter->typed_output_tensor<float>(0);

    const uint32_t OUT_WIDTH = 40;
    const uint32_t OUT_HEIGHT = 12;

    // Map output layer to cv::Mat
    cv::Mat out(OUT_HEIGHT, OUT_WIDTH, CV_32FC1);

    for(int i = 0; i < OUT_HEIGHT; ++i) {
        float* p = out.ptr<float>(i);
        for(int j = 0; j < OUT_WIDTH; ++j) {
            float val = output_layer[i * OUT_WIDTH + j];
            if(val > 1.0f) val == 1.0f;
            else if(val < 0.0f) val == 0.0f;
            p[j] = val;
        }
    }
    return out;
}