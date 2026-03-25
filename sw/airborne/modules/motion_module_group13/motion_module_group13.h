/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/motion_module_group13/motion_module_group13.h"
 * @author Roland Meertens
 * Combined module: Optic flow avoidance with an Orange Color Detection Failsafe.
 */

#ifndef MOTION_MODULE_GROUP13_H
#define MOTION_MODULE_GROUP13_H

#include <stdint.h>

// --- Settings (Exposed to XML / GCS) ---
extern float oa_color_count_frac;
extern uint8_t oa_flow_lpf_enable;
extern float oa_flow_lpf_alpha;
extern int32_t oa_avg_flow_threshold;  // ADDED: Matches the new XML slider!

// --- Global variables ---
extern int32_t color_count;            // Exposed in case you want to log it via telemetry later

// --- Functions ---
extern void motion_module_group13_init(void);
extern void motion_module_group13_periodic(void);

#endif