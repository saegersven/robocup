#include "green_nn.h"
 
GreenML::GreenML() {}

void GreenML::start() {
    running = true;
    has_new_frame = false;
    status = false;
    model = tflite::FlatBufferModel::BuildFromFile("/home/pi/robocup/runtime_data/green.tflite");
    tflite::ops::builtin::BuiltinOpResolver resolver;
    tflite::InterpreterBuilder builder(*model, resolver);
    builder(&interpreter);
    interpreter->AllocateTensors();

    input_layer = interpreter->typed_input_tensor<float>(0);
    output_layer = interpreter->typed_output_tensor<float>(0);
}

void GreenML::stop() {
    running = false;
}

void GreenML::internal_loop() {
    while(running) {
        if(!has_new_frame) continue;
        frame_swap_lock.lock();
        cv::Mat image = current_frame.clone();
        frame_swap_lock.unlock();
        
        // Conversion from byte & BGR to normalized float & RGB is done later in the copying for loop
        //cv::cvtColor(byte_image, byte_image, cv::COLOR_BGR2RGB);

        //cv::Mat image;
        //byte_image.convertTo(image, CV_32FC3, 1.0f/255.0f);

        uint32_t channels = image.channels();
        uint32_t width = image.cols * image.channels();

        cv::Vec3b* p;
        int i, j, k;
        for(i = 0; i < image.rows; ++i) {
            p = image.ptr<cv::Vec3b>(i);
            for(j = 0; j < image.cols; ++j) {
                for(k = 0; k < channels; ++k) {
                    input_layer[i * width + j * channels + k] = (float)p[j][channels - k - 1] / 255.0f;
                }
            }
        }       

        interpreter->Invoke();

        //std::cout << "NN says: " << output_layer[0] << "\t" << output_layer[1] << std::endl;
        status = output_layer[1] > 0.9f;
        if(status) {
            std::cout << "NN detected green [" << output_layer[1] << "]" << std::endl;
        }
        has_new_frame = false;
    }
}

bool GreenML::predict_green(cv::Mat image) {
    uint32_t channels = image.channels();
    uint32_t width = image.cols * image.channels();

    cv::Vec3b* p;
    int i, j, k;
    for(i = 0; i < image.rows; ++i) {
        p = image.ptr<cv::Vec3b>(i);
        for(j = 0; j < image.cols; ++j) {
            for(k = 0; k < channels; ++k) {
                input_layer[i * width + j * channels + k] = (float)p[j][channels - k - 1] / 255.0f;
            }
        }
    }       

    interpreter->Invoke();

    std::cout << "Green NN says: " << output_layer[0] << "\t" << output_layer[1] << std::endl;
    status = output_layer[0] > output_layer[1];
    if(status) {
        std::cout << "NN detected green [" << output_layer[1] << "]" << std::endl;
    }
    return status;
}