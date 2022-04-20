#include "silver_ml.h"
 
SilverML::SilverML() {}

void SilverML::start() {
    running = true;
    has_frame = false;
    model = tflite::FlatBufferModel::BuildFromFile("/home/pi/robocup/runtime_data/silver.tflite");
    tflite::ops::builtin::BuiltinOpResolver resolver;
    tflite::InterpreterBuilder builder(*model, resolver);
    builder(&interpreter);
    interpreter->AllocateTensors();

    std::thread t([this]() { this->internal_loop(); });
    t.detach();
}

void SilverML::stop() {
    running = false;
}

void SilverML::internal_loop() {
    while(running) {
        if(!has_frame) continue;
        frame_swap_lock.lock();
        cv::Mat byte_image = current_frame.clone();
        frame_swap_lock.unlock();
        //cv::cvtColor(byte_image, byte_image, cv::COLOR_BGR2RGB);

        cv::Mat image;
        byte_image.convertTo(image, CV_32FC3, 1.0f/255.0f);

        //assert(image.isContinuous());

        float* input_layer = interpreter->typed_input_tensor<float>(0);
        float* output_layer = interpreter->typed_output_tensor<float>(0);

        float* img_ptr = image.ptr<float>(0);

        cv::Vec3f* p;
        int i, j;
        for(i = 0; i < image.rows; ++i) {
            p = image.ptr<cv::Vec3f>(i);
            for(j = 0; j < image.cols; ++j) {
                for(int k = 0; k < image.channels(); ++k) {
                    input_layer[i * image.cols * image.channels() + image.channels() * j + k] = p[j][k];
                
                }
            }
        }       

        interpreter->Invoke();

        std::cout << "NN says: " << output_layer[0] << "\t" << output_layer[1] << std::endl;
        status = output_layer[1] > 0.9f;
    }   
}

bool SilverML::predict_silver(cv::Mat frame) {
    frame_swap_lock.lock();
    current_frame = frame;
    frame_swap_lock.unlock();
    has_frame = true;
    return status;
}