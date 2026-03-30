#include "svm.h"
#include <opencv2/core.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

// Global variable accessible by Paparazzi
uint8_t obstacle_detected = 0;

// Store the previous frame for optical flow
static cv::Mat previous_gray;

// Paste the parameters exported from your Python script here
const float scaler_mean[3] = {0.849761, 0.004354, 4.496045};
const float scaler_scale[3] = {1.168939, 0.011164, 13.586546};
const float svm_weights[3] = {2.977097, -0.235010, -2.915282};
const float svm_intercept = -0.059826;

extern "C" {

void svm_init(void) {
    obstacle_detected = 0;
}

struct image_t *svm_run(struct image_t *img, uint8_t camera_id) {
    // 1. Convert Paparazzi image_t to OpenCV Mat
    // Assuming the incoming image is grayscale (YUV Y-channel)
    cv::Mat current_gray_full(img->h, img->w, CV_8UC1, img->buf);
    cv::Mat current_gray;

    // 2. CRITICAL: Downsample the image to save Bebop CPU!
    // Example: shrink to 160x120
    cv::resize(current_gray_full, current_gray, cv::Size(160, 120));

    if (!previous_gray.empty()) {
        cv::Mat flow(current_gray.size(), CV_32FC2);
        
        // 3. Calculate Dense Optical Flow
        cv::calcOpticalFlowFarneback(previous_gray, current_gray, flow, 0.5, 2, 11, 2, 5, 1.2, 0);

        // Split flow into u (horizontal) and v (vertical) components
        cv::Mat flow_parts[2];
        cv::split(flow, flow_parts);
        cv::Mat u = flow_parts[0];
        cv::Mat v = flow_parts[1];

        // --- FEATURE 1: Flow Magnitude ---
        cv::Mat magnitude, angle;
        cv::cartToPolar(u, v, magnitude, angle);
        float flow_magnitude = cv::mean(magnitude)[0];

        // --- FEATURE 2: Center Divergence ---
        // Simple C++ approximation of your Python np.gradient
        cv::Mat du_dx, dv_dy;
        cv::Sobel(u, du_dx, CV_32F, 1, 0, 1);
        cv::Sobel(v, dv_dy, CV_32F, 0, 1, 1);
        cv::Mat divergence_field = du_dx + dv_dy;
        
        // Extract center region (h/4 to 3h/4, w/4 to 3w/4)
        int h = divergence_field.rows;
        int w = divergence_field.cols;
        cv::Rect center_roi(w/4, h/4, w/2, h/2);
        float mean_divergence = cv::mean(divergence_field(center_roi))[0];

        // --- FEATURE 3: Vertical Variance ---
        cv::Mat mean_v, stddev_v;
        cv::meanStdDev(v, mean_v, stddev_v);
        float flow_y_var = stddev_v.at<double>(0,0) * stddev_v.at<double>(0,0);

        // --- SVM INFERENCE ---
        float features[3] = {flow_magnitude, mean_divergence, flow_y_var};
        float decision_value = 0.0f;

        for (int i = 0; i < 3; i++) {
            float scaled_feature = (features[i] - scaler_mean[i]) / scaler_scale[i];
            decision_value += scaled_feature * svm_weights[i];
        }
        decision_value += svm_intercept;

        obstacle_detected = (decision_value > 0.0f) ? 1 : 0;
    }

    // Save current frame for the next iteration
    current_gray.copyTo(previous_gray);

    return img;
}

} // end extern "C"