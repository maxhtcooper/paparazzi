#include "modules/computer_vision/colorfilter_tijn.h"
#include "modules/computer_vision/cv.h"
#include <string.h>
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

// Initialize with YUV values for orange poles
uint8_t orange_y_min = 105, orange_y_max = 205;
uint8_t orange_u_min = 52,  orange_u_max = 140;
uint8_t orange_v_min = 180, orange_v_max = 255;

// Global variable for telemetry or ABI coupling
uint16_t safe_heading_x = 0;

// Static memory allocation for the 1D array to prevent malloc issues
#define MAX_IMG_WIDTH 1280
static uint16_t column_orange_count[MAX_IMG_WIDTH];

struct image_t *colorfilter_tijn_func(struct image_t *img, uint8_t camera_id) {
  // Security: check if the frame is in the expected YUV422 format

  //unusedparameter warning suppression
  (void)camera_id;

  if (img->type != IMAGE_YUV422) {
    return img;
  }

  // Efficiently reset the array with memset
  memset(column_orange_count, 0, sizeof(column_orange_count));

  // Compute optimization: downsampling
  const uint8_t step = 4; // Analyze 1 out of every 4 pixels

  for (uint16_t y = 0; y < img->h; y += step) {
    for (uint16_t x = 0; x < img->w; x += step) {
      
      // Calculate the index in the flat memory buffer (YUV422 structure)
      uint32_t idx = (y * img->w * 2) + (x * 2);

      // U and V are shared in YUV422; extract correctly based on 4-byte blocks
      uint8_t u_val = ((uint8_t *)img->buf)[idx - (idx % 4)];     
      uint8_t y_val = ((uint8_t *)img->buf)[idx - (idx % 4) + 1]; 
      uint8_t v_val = ((uint8_t *)img->buf)[idx - (idx % 4) + 2]; 

      // YUV threshold check for the color orange
      if (y_val > orange_y_min && y_val < orange_y_max &&
          u_val > orange_u_min && u_val < orange_u_max &&
          v_val > orange_v_min && v_val < orange_v_max) {
        
        column_orange_count[x]++;
      }
    }
  }

  // --- START GAP-FINDING LOGIC ---
  uint16_t current_gap_start = 0;
  uint16_t current_gap_width = 0;
  uint16_t best_gap_start = 0;
  uint16_t best_gap_width = 0;
  
  // Threshold to ignore noise (a few false positive pixels)
  const uint16_t noise_threshold = 3; 

  for (uint16_t x = 0; x < img->w; x += step) {
    if (column_orange_count[x] <= noise_threshold) {
      // This is 'free' space
      if (current_gap_width == 0) {
        current_gap_start = x; // Start a new gap
      }
      current_gap_width += step;
    } else {
      // Obstacle detected, close the current gap
      if (current_gap_width > best_gap_width) {
        best_gap_width = current_gap_width;
        best_gap_start = current_gap_start;
      }
      current_gap_width = 0; // Reset for the next iteration
    }
  }
  
  // Check if the last measured gap (at the right edge) is the largest
  if (current_gap_width > best_gap_width) {
    best_gap_width = current_gap_width;
    best_gap_start = current_gap_start;
  }

  // Calculate the x-coordinate of the center of the largest gap
  safe_heading_x = best_gap_start + (best_gap_width / 2);
  // --- END GAP-FINDING LOGIC ---

 // Transmit the variable to the GCS
  DOWNLINK_SEND_COLOR_FILTER_OUTPUT(DefaultChannel, DefaultDevice, &safe_heading_x);
  
  return img;
}

void colorfilter_tijn_init(void) {
  cv_add_to_device(&front_camera, colorfilter_tijn_func, 0, 0); 
}