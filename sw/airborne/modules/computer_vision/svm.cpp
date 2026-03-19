#include "svm.h"
#include <opencv2/core.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

// Global variable accessible by Paparazzi
uint8_t obstacle_detected = 0;

// Store the previous frame for optical flow (middle third only to save memory)
static cv::Mat previous_gray_middle;

// Trained SVM parameters from scikit-learn
const float scaler_mean[3] = {0.849761, 0.004354, 4.496088};
const float scaler_scale[3] = {1.168942, 0.011164, 13.587011};
const float svm_weights[3] = {2.977478, -0.235030, -2.915399};
const float svm_intercept = -0.060160;

extern "C" {

void svm_init(void) {
    obstacle_detected = 0;
    previous_gray_middle.release();
}

struct image_t *svm_run(struct image_t *img, uint8_t camera_id) {
    // 1. Convert Paparazzi image_t to OpenCV Mat
    // Assuming the incoming image is grayscale (YUV Y-channel)
    cv::Mat current_gray_full(img->h, img->w, CV_8UC1, img->buf);
    cv::Mat current_gray;

    // 2. CRITICAL: Downsample the image to save Bebop CPU!
    // Bebop has limited hardware - use 120x80 (very aggressive downsampling)
    cv::resize(current_gray_full, current_gray, cv::Size(120, 80));

    // 3. Extract MIDDLE THIRD of the downsampled image
    // Image sliced into 3 vertical sections: [0, w/3), [w/3, 2w/3), [2w/3, w)
    // We analyze only the middle section: [w/3, 2w/3)
    int third_width = current_gray.cols / 3;  // Width of each third
    int middle_start_x = third_width;          // Start of middle third (at w/3)
    int middle_width = third_width;            // Width of middle section
    
    // Extract middle third ROI
    cv::Rect middle_roi(middle_start_x, 0, middle_width, current_gray.rows);
    cv::Mat current_gray_middle = current_gray(middle_roi).clone();

    if (!previous_gray_middle.empty()) {
        cv::Mat flow(current_gray_middle.size(), CV_32FC2);
        
        // 4. Calculate Dense Optical Flow (optimized window size for Bebop)
        cv::calcOpticalFlowFarneback(previous_gray_middle, current_gray_middle, flow, 
                                      0.5, 2, 9, 2, 5, 1.1, 0);

        // Split flow into u (horizontal) and v (vertical) components
        cv::Mat flow_parts[2];
        cv::split(flow, flow_parts);
        cv::Mat u = flow_parts[0];
        cv::Mat v = flow_parts[1];

        // --- FEATURE 1: Flow Magnitude ---
        cv::Mat magnitude, angle;
        cv::cartToPolar(u, v, magnitude, angle);
        float flow_magnitude = cv::mean(magnitude)[0];

        // --- FEATURE 2: Center Divergence (expansion detection) ---
        cv::Mat du_dx, dv_dy;
        cv::Sobel(u, du_dx, CV_32F, 1, 0, 1);
        cv::Sobel(v, dv_dy, CV_32F, 0, 1, 1);
        cv::Mat divergence_field = du_dx + dv_dy;
        
        // Use center region of the middle third to reduce edge artifacts
        int h = divergence_field.rows;
        int w = divergence_field.cols;
        int margin_x = w / 6;
        int margin_y = h / 4;
        cv::Rect center_roi(margin_x, margin_y, w - 2*margin_x, h - 2*margin_y);
        
        float mean_divergence = 0.0f;
        if (center_roi.width > 0 && center_roi.height > 0) {
            mean_divergence = cv::mean(divergence_field(center_roi))[0];
        }

        // --- FEATURE 3: Vertical Flow Variance ---
        cv::Scalar mean_v, stddev_v;
        cv::meanStdDev(v, mean_v, stddev_v);
        float flow_y_var = stddev_v[0] * stddev_v[0];

        // --- SVM INFERENCE ---
        float features[3] = {flow_magnitude, mean_divergence, flow_y_var};
        float decision_value = 0.0f;

        // Normalize features using StandardScaler parameters from training
        for (int i = 0; i < 3; i++) {
            float scaled_feature = (features[i] - scaler_mean[i]) / scaler_scale[i];
            decision_value += scaled_feature * svm_weights[i];
        }
        decision_value += svm_intercept;

        // Classification: positive value = obstacle detected
        obstacle_detected = (decision_value > 0.0f) ? 1 : 0;
    }

    // Save current frame for the next iteration
    current_gray_middle.copyTo(previous_gray_middle);

    return img;
}

} // end extern "C"