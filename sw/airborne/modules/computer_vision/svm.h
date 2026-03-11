#ifndef SVM
#define SVM

#include <stdint.h>
#include "modules/computer_vision/lib/vision/image.h"

#ifdef __cplusplus
extern "C" {
#endif

// Global flag for the flight plan
extern uint8_t obstacle_detected;

// Initialization function
void svm_init(void);

// The main processing function hooked to the camera thread
struct image_t *svm_run(struct image_t *img, uint8_t camera_id);

#ifdef __cplusplus
}
#endif

#endif // SVM