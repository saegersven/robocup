#include "victim_ml.h"
 
#define OUT_WIDTH 40
#define OUT_HEIGHT 30
#define OUT_CHANNELS 2

VictimML::VictimML() { }

void VictimML::init() {
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

    // Map output layer to cv::Mat
    cv::Mat out(OUT_HEIGHT, OUT_WIDTH, CV_32FC2);

    for(int i = 0; i < OUT_HEIGHT; ++i) {
        float* p = out.ptr<float>(i);
        for(int j = 0; j < OUT_WIDTH; ++j) {
            for(int k = 0; k < OUT_CHANNELS; ++k) {
                float val = output_layer[i * OUT_WIDTH * OUT_CHANNELS + j * OUT_CHANNELS + k];
                if(val > 1.0f) val == 1.0f;
                else if(val < 0.0f) val == 0.0f;
                p[j * OUT_CHANNELS + k] = val;
            }
        }
    }
    return out;
}

std::vector<Victim> VictimML::extract_victims(cv::Mat probability_map) {
    const float THRESHOLD = 0.2f;

    cv::Mat blurred;
    cv::GaussianBlur(probability_map, blurred, cv::Size(1, 3), 0);

    cv::Mat thresh1;
    cv::Mat thresh2;
    cv::inRange(blurred, cv::Scalar(THRESHOLD, 0.0f), cv::Scalar(1.0f, 1.0f), thresh1);
    cv::inRange(blurred, cv::Scalar(0.0f, THRESHOLD), cv::Scalar(1.0f, 1.0f), thresh2);
    cv::Mat thresh;
    cv::bitwise_or(thresh1, thresh2, thresh);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<Victim> victims;

    for(int i = 0; i < contours.size(); ++i) {
        cv::Mat mask(blurred.rows, blurred.cols, CV_8UC1, cv::Scalar(0));
        cv::drawContours(mask, contours, i, cv::Scalar(255), -1);
        cv::Scalar mean = cv::mean(blurred, mask);

        bool is_dead = mean[0] > mean[1];

        /*float total_alive = 0.0f;
        float total_dead = 0.0f;
        for(int y = 0; y < blurred.rows; ++y) {
            cv::Vec3f* p = blurred.ptr<cv::Vec3f>(y);
            uint8_t* p_mask = thresh.ptr<uint8_t>(y);
            for(int x = 0; x < blurred.cols; ++x) {
                if(p_mask[x]) {
                    total_alive += p[x][2];
                    total_dead += p[x][1];
                }
            }
        }
        bool dead = total_dead > total_alive;*/

        //std::cout << is_dead << "\t";

        const float CHUNK_WIDTH = (float)160 / OUT_WIDTH;
        const float CHUNK_HEIGHT = (float)120 / OUT_HEIGHT;

        cv::Rect rect = cv::boundingRect(contours[i]);
        float min_area = (180.0f / 60.0f) * (rect.y + rect.height / 2.0f) + 20.0f;

        if((float)rect.height * CHUNK_HEIGHT * (float)rect.width * CHUNK_WIDTH < min_area) {
            continue;
        }

        for(int j = 0; j < victims.size(); ++j) {
            
        }

        victims.push_back({
            is_dead,
            ((float)rect.x + rect.width / 2.0f) * CHUNK_WIDTH,
            ((float)rect.y + rect.height / 2.0f) * CHUNK_HEIGHT
        });
    }
    std::cout << std::endl;

    return victims;
}