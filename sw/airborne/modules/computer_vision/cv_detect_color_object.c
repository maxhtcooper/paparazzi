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
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"
#include "modules/computer_vision/lib/vision/image.h"

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

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif

// Filter Settings
uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;

uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  int32_t width;
  int32_t height;
  bool updated;
};
struct color_object_t global_filters[2];

// Function
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, 
                              int32_t* p_width, int32_t* p_height, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);

/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;

  switch (filter){
    case 1:
      lum_min = cod_lum_min1;
      lum_max = cod_lum_max1;
      cb_min = cod_cb_min1;
      cb_max = cod_cb_max1;
      cr_min = cod_cr_min1;
      cr_max = cod_cr_max1;
      draw = cod_draw1;
      break;
    case 2:
      lum_min = cod_lum_min2;
      lum_max = cod_lum_max2;
      cb_min = cod_cb_min2;
      cb_max = cod_cb_max2;
      cr_min = cod_cr_min2;
      cr_max = cod_cr_max2;
      draw = cod_draw2;
      break;
    default:
      return img;
  };

  int32_t x_c, y_c, width, height;

  // Filter and find centroid
  uint32_t count = find_object_centroid(img, &x_c, &y_c, &width, &height, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);
  VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
  VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
        hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));

  pthread_mutex_lock(&mutex);
  global_filters[filter-1].color_count = count;
  global_filters[filter-1].x_c = x_c;
  global_filters[filter-1].y_c = y_c;
  global_filters[filter-1].width = width;
  global_filters[filter-1].height = height;
  global_filters[filter-1].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}

struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

struct image_t *object_detector2(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector2(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 2);
}

void color_object_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1
  cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
  cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
  cod_cb_min1 = COLOR_OBJECT_DETECTOR_CB_MIN1;
  cod_cb_max1 = COLOR_OBJECT_DETECTOR_CB_MAX1;
  cod_cr_min1 = COLOR_OBJECT_DETECTOR_CR_MIN1;
  cod_cr_max1 = COLOR_OBJECT_DETECTOR_CR_MAX1;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
  cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1, 0);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2
  cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2;
  cod_lum_max2 = COLOR_OBJECT_DETECTOR_LUM_MAX2;
  cod_cb_min2 = COLOR_OBJECT_DETECTOR_CB_MIN2;
  cod_cb_max2 = COLOR_OBJECT_DETECTOR_CB_MAX2;
  cod_cr_min2 = COLOR_OBJECT_DETECTOR_CR_MIN2;
  cod_cr_max2 = COLOR_OBJECT_DETECTOR_CR_MAX2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW2
  cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2, 1);
#endif
}

/*
 * find_object_centroid
 *
 * Finds the centroid of pixels in an image within filter bounds.
 * Also returns the amount of pixels that satisfy these filter bounds.
 *
 * @param img - input image to process formatted as YUV422.
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param lum_min - minimum y value for the filter in YCbCr colorspace
 * @param lum_max - maximum y value for the filter in YCbCr colorspace
 * @param cb_min - minimum cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return number of pixels of image within the filter bounds.
 */
// uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, 
//                               int32_t* p_width, int32_t* p_height, bool draw,
//                               uint8_t lum_min, uint8_t lum_max,
//                               uint8_t cb_min, uint8_t cb_max,
//                               uint8_t cr_min, uint8_t cr_max)
// {
//   uint32_t cnt = 0;
//   uint32_t tot_x = 0;
//   uint32_t tot_y = 0;
//   uint8_t *buffer = img->buf;

//   // initialize bounding box extremes
//   int32_t x_min = img->w;
//   int32_t x_max = 0;
//   int32_t y_min = img->h;
//   int32_t y_max = 0;

//   // Go through all the pixels
//   for (uint16_t y = 0; y < img->h; y++) {
//     for (uint16_t x = 0; x < img->w; x ++) {
//       // Check if the color is inside the specified values
//       uint8_t *yp, *up, *vp;
//       if (x % 2 == 0) {
//         // Even x
//         up = &buffer[y * 2 * img->w + 2 * x];      // U
//         yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
//         vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
//         //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
//       } else {
//         // Uneven x
//         up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
//         //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
//         vp = &buffer[y * 2 * img->w + 2 * x];      // V
//         yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
//       }
//       if ( (*yp >= lum_min) && (*yp <= lum_max) &&
//            (*up >= cb_min ) && (*up <= cb_max ) &&
//            (*vp >= cr_min ) && (*vp <= cr_max )) {
//         cnt ++;
//         tot_x += x;
//         tot_y += y;

//         // track the edges of the orange object for bounding box drawing
//         if (x < x_min) x_min = x;
//         if (x > x_max) x_max = x;
//         if (y < y_min) y_min = y;
//         if (y > y_max) y_max = y;
//         if (draw){
//           *yp = 255;  // make pixel brighter in image
//         }
//       }
//     }
//   }
//   if (cnt > 0) {
//     *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
//     *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
//     *p_width = x_max - x_min;
//     *p_height = y_max - y_min;
//   } else {
//     *p_xc = 0;
//     *p_yc = 0;
//     *p_width = 0;
//     *p_height = 0;
//   }
//   return cnt;
// }
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, 
                              int32_t* p_width, int32_t* p_height, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  uint8_t *buffer = img->buf;

  // Blob clustering setup
  #define MAX_BLOBS 10
  int b_xmin[MAX_BLOBS], b_xmax[MAX_BLOBS], b_ymin[MAX_BLOBS], b_ymax[MAX_BLOBS], b_cnt[MAX_BLOBS];
  int num_blobs = 0;

  // Go through all the pixels
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x ++) {
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        up = &buffer[y * 2 * img->w + 2 * x];      
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  
      } else {
        up = &buffer[y * 2 * img->w + 2 * x - 2];  
        vp = &buffer[y * 2 * img->w + 2 * x];      
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  
      }

      // If the pixel matches the orange color threshold
      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) {
        
        if (draw) *yp = 255;  // Highlight the pixel white
        
        // 1-Pass Clustering: Find the closest existing blob
        int matched_blob = -1;
        for (int i = 0; i < num_blobs; i++) {
            // Calculate Manhattan distance to the blob's bounding box
            int dx = 0, dy = 0;
            if (x < b_xmin[i]) dx = b_xmin[i] - x;
            else if (x > b_xmax[i]) dx = x - b_xmax[i];
            
            if (y < b_ymin[i]) dy = b_ymin[i] - y;
            else if (y > b_ymax[i]) dy = y - b_ymax[i];
            
            // If the pixel is within 30 pixels of the box, merge it!
            if (dx + dy < 15) { 
                matched_blob = i;
                break;
            }
        }
        
        if (matched_blob != -1) {
            // Expand the existing blob's boundaries
            if (x < b_xmin[matched_blob]) b_xmin[matched_blob] = x;
            if (x > b_xmax[matched_blob]) b_xmax[matched_blob] = x;
            if (y < b_ymin[matched_blob]) b_ymin[matched_blob] = y;
            if (y > b_ymax[matched_blob]) b_ymax[matched_blob] = y;
            b_cnt[matched_blob]++;
        } else if (num_blobs < MAX_BLOBS) {
            // Create a completely new distinct object
            b_xmin[num_blobs] = x;
            b_xmax[num_blobs] = x;
            b_ymin[num_blobs] = y;
            b_ymax[num_blobs] = y;
            b_cnt[num_blobs] = 1;
            num_blobs++;
        }
      }
    }
  }

  // Find the largest valid obstacle and draw ALL valid blobs
  int best_blob = -1;
  int max_count = 0;
  uint8_t red_color[3] = {81, 90, 240}; // red color for bounding box in YUV

  for (int i = 0; i < num_blobs; i++) {
      // NOISE FILTER: Ignore any tiny boxes with fewer than 60 pixels
      if (b_cnt[i] > 60) {
          if (draw) {
              // Draw the bounding box directly on the camera feed!
              for (int t = 0 ; t < 3; t++) { // make the box thicker by drawing it 3 times
                  image_draw_rectangle(img, b_xmin[i]-t, b_xmax[i]+t, b_ymin[i]-t, b_ymax[i]+t, red_color);
              }
              
          }
          
          // Track the absolute largest object for the flight controller
          if (b_cnt[i] > max_count) {
              max_count = b_cnt[i];
              best_blob = i;
          }
      }
  }

  // Send ONLY the largest threat to the drone's autopilot module
  if (best_blob != -1) {
    *p_xc = b_xmin[best_blob] + (b_xmax[best_blob] - b_xmin[best_blob]) / 2;
    *p_yc = b_ymin[best_blob] + (b_ymax[best_blob] - b_ymin[best_blob]) / 2;
    *p_width = b_xmax[best_blob] - b_xmin[best_blob];
    *p_height = b_ymax[best_blob] - b_ymin[best_blob];
    return max_count;
  } else {
    *p_xc = 0; *p_yc = 0; *p_width = 0; *p_height = 0;
    return 0;
  }
}

void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        local_filters[0].width, local_filters[0].height, local_filters[0].color_count, 0);
    local_filters[0].updated = false;
  }
  if(local_filters[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
        local_filters[1].width, local_filters[1].height, local_filters[1].color_count, 1);
    local_filters[1].updated = false;
  }
}
