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
 * @file modules/computer_vision/group13.c
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "cv_group13.h"

#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"
#include "modules/pose_history/pose_history.h"
#include "state.h"

#include "lib/v4l/v4l2.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef OPTIC_FLOW_DETECTOR_FPS1
#define OPTIC_FLOW_DETECTOR_FPS1 10 ///< Default FPS (zero means run at camera fps)
#endif

// Filter Settings
uint8_t test_setting = 0;
bool test_flag = false;
uint8_t edge_threshold = 0;

/* The main opticflow variables */
// we always have 1 active camera
struct opticflow_t opticflow[1];  ///< Opticflow calculations settings
static struct opticflow_result_t opticflow_result[1]; ///< The opticflow result (scoped in this file)
static bool opticflow_got_result[1];  ///< Whether we got a new result since the last time we sent it out (scoped in this file)

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
};
struct color_object_t global_filters[1];


/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  // derotate optic flow based on the current pose of the drone
  struct pose_t pose = get_rotation_at_timestamp(img->pprz_ts);
  // store in image metadata for the optic flow algorithms
  img->eulers = pose.eulers;

  // static so that the number of corners is kept between frames
  static struct opticflow_result_t temp_result[1];
  // Do the optical flow calculation using the settings struct
  if (opticflow_calc_frame(&opticflow[filter-1], img, &temp_result[filter-1])) {
    // Copy the result if finished
    pthread_mutex_lock(&mutex);
    // static struct to store results to be later sent via Abi
    opticflow_result[filter-1] = temp_result[filter-1];
    opticflow_got_result[filter-1] = true;
    pthread_mutex_unlock(&mutex);
  }

  return img;
}

// video callback, runs in the video thread
// it calls a function that updates the global_filters with the new color filter outputs,
// which are then sent to the main thread in the periodic function
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

void optic_flow_detector_init(void)
{
  memset(global_filters, 0, 1*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);
  opticflow_calc_init(opticflow);

#ifndef OPTIC_FLOW_DETECTOR_CAMERA1
#error "OPTIC_FLOW_DETECTOR_CAMERA1 has to be defined"
#endif

#ifdef TEST_SETTING // platform specific (nps vs ap)
  test_setting = TEST_SETTING;
#endif

#ifdef TEST_FLAG
  test_flag = TEST_FLAG;
#endif

  cv_add_to_device(&OPTIC_FLOW_DETECTOR_CAMERA1, object_detector1, OPTIC_FLOW_DETECTOR_FPS1, 0);
}

void optic_flow_detector_periodic(void)
{
  // TODO: when is it worth it to lock the mutex for the Abi send too
  // and skip memcpy? Perhaps if the struct is too large and copying takes
  // a lot of time?

  // video thread saves results into the global_filters struct, so we need to
  // copy it to a local variable before sending it to the rest of the system
  // to not block the video thread for too long
  static struct opticflow_result_t local_results[1];
  bool have_new_result = false;
  pthread_mutex_lock(&mutex);
  if (opticflow_got_result[0]) {
    memcpy(local_results, opticflow_result, 1*sizeof(struct opticflow_result_t));
    opticflow_got_result[0] = false;
    have_new_result = true;
  }
  pthread_mutex_unlock(&mutex);

  // updated flag comes from the video thread, which sets it to true
  // when new filter outputs are available, and the main thread sets it back to false
  // after sending the new outputs to the rest of the system

  // OPTIC_FLOW_VISUAL_DETECTION_ID is defined in the XML to be
  // COLOR_OBJECT_DETECTION1_ID whic is defined on the Abi level to be 1
  if (have_new_result) {
    AbiSendMsgVISUAL_DETECTION(OPTIC_FLOW_VISUAL_DETECTION_ID, local_results[0].flow_x, local_results[0].flow_y,
        local_results[0].color_frac, local_results[0].sparse_edge_bins, local_results[0].avg_flow, 0);
  }
}
