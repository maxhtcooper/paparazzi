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
extern float oa_hist_flow_threshold;
extern float oa_min_forward_distance;
extern uint8_t oa_flow_lpf_enable;
extern float oa_flow_lpf_alpha;
extern uint8_t reset_confidence_level;
extern uint8_t max_trajectory_confidence;
extern float turn_base_deg;
extern float turn_gain_deg;
extern float max_distance;
extern uint8_t turn_freq;

// functions
extern void motion_module_group13_init(void);
extern void motion_module_group13_periodic(void);

#endif

