/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/cv_group13.h
 * Optic flow test
 */

#ifndef CV_GROUP13_H
#define CV_GROUP13_H

#include <stdint.h>
#include <stdbool.h>

// Include opticflow calculator
#include "opticflow/opticflow_calculator.h"

// Struct from the above header used to store optic flow settings
extern struct opticflow_t opticflow[];
// opticflow_result_t also comes from the above header file
// used to store results

// Module settings
extern uint8_t test_setting;

extern bool test_flag;

extern uint8_t edge_threshold;

// Module functions
extern void optic_flow_detector_init(void);
extern void optic_flow_detector_periodic(void);

#endif /* CV_GROUP13_H */
