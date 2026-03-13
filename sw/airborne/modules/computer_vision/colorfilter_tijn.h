#ifndef COLORFILTER_TIJN_H
#define COLORFILTER_TIJN_H

#include "std.h"
#include "modules/computer_vision/lib/vision/image.h"

// Global thresholds for YUV filtering
extern uint8_t orange_y_min, orange_y_max;
extern uint8_t orange_u_min, orange_u_max;
extern uint8_t orange_v_min, orange_v_max;

// Output variable for navigation
extern uint16_t safe_heading_x;

// Initialization and per-frame function
extern void colorfilter_tijn_init(void);
extern struct image_t *colorfilter_tijn_func(struct image_t *img, uint8_t camera_id);

#endif