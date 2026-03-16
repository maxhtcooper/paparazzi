/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

// #ifndef ORANGE_AVOIDER_H
// #define ORANGE_AVOIDER_H
#ifndef MOTION_MODULE_GROUP13_H
#define MOTION_MODULE_GROUP13_H
#include <stdint.h>

// settings
extern float oa_color_count_frac;
extern uint8_t oa_flow_lpf_enable;
extern float oa_flow_lpf_alpha;

// global variables
extern int16_t svm_bbox_x;
extern int16_t svm_bbox_y;
extern int16_t svm_bbox_width;
extern int16_t svm_bbox_height;
// functions
extern void motion_module_group13_init(void);
extern void motion_module_group13_periodic(void);

#endif

