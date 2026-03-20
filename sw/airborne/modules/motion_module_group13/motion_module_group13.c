/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used with cv_group13 optical flow output and the navigation mode of the autopilot.
 * The avoidance strategy uses only the optical-flow histogram signal (avg_flow).
 * When |avg_flow| exceeds a threshold, we assume there is an obstacle and turn away.
 */

#include "modules/motion_module_group13/motion_module_group13.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

// some includes to visualize bounding boxes in video stream
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"

#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif


// Bounding box memory variables
int16_t svm_bbox_x = 0;
int16_t svm_bbox_y = 0;
int16_t svm_bbox_width = 0;
int16_t svm_bbox_height = 0;

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

#ifndef MOTION_GROUP13_FLOW_LPF_ENABLE
#define MOTION_GROUP13_FLOW_LPF_ENABLE 1
#endif

#ifndef MOTION_GROUP13_FLOW_LPF_ALPHA
#define MOTION_GROUP13_FLOW_LPF_ALPHA 0.20f
#endif

#ifndef MOTION_GROUP13_HIST_FLOW_THRESHOLD
#define MOTION_GROUP13_HIST_FLOW_THRESHOLD 50.f
#endif

#ifndef MOTION_GROUP13_HIST_FLOW_CLEAR_THRESHOLD
#define MOTION_GROUP13_HIST_FLOW_CLEAR_THRESHOLD 25.f
#endif

#ifndef MOTION_GROUP13_MIN_FORWARD_DISTANCE
#define MOTION_GROUP13_MIN_FORWARD_DISTANCE 0.35f
#endif

#ifndef MOTION_GROUP13_GAP_BALANCE_THRESHOLD
#define MOTION_GROUP13_GAP_BALANCE_THRESHOLD 20.f
#endif

#ifndef MOTION_GROUP13_EMERGENCY_FLOW_THRESHOLD
#define MOTION_GROUP13_EMERGENCY_FLOW_THRESHOLD 160.f
#endif

// define settings  
float oa_hist_flow_threshold = MOTION_GROUP13_HIST_FLOW_THRESHOLD;
float oa_hist_flow_clear_threshold = MOTION_GROUP13_HIST_FLOW_CLEAR_THRESHOLD;
float oa_min_forward_distance = MOTION_GROUP13_MIN_FORWARD_DISTANCE;
float oa_gap_balance_threshold = MOTION_GROUP13_GAP_BALANCE_THRESHOLD;
float oa_emergency_flow_threshold = MOTION_GROUP13_EMERGENCY_FLOW_THRESHOLD;
uint8_t oa_flow_lpf_enable = MOTION_GROUP13_FLOW_LPF_ENABLE;
float oa_flow_lpf_alpha = MOTION_GROUP13_FLOW_LPF_ALPHA;

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;

int16_t flow_der_x = 0;
int16_t flow_der_y = 0;
int32_t avg_flow = 0;
static float avg_flow_lpf_state = 0.f;
static uint8_t avg_flow_lpf_initialized = 0;
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float heading_increment = 50.f;          // heading angle increment [deg]
float maxDistance = 2.25;               // max waypoint displacement [m]

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
#ifndef OPTIC_FLOW_VISUAL_DETECTION_ID
#define OPTIC_FLOW_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

static abi_event color_detection_ev;
static void optic_flow_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) flow_x, int16_t __attribute__((unused)) flow_y,
                               int16_t flow_der_x_received, int16_t flow_der_y_received,
                               int32_t avg_received, int16_t __attribute__((unused)) extra)
{
  flow_der_x = flow_der_x_received;
  flow_der_y = flow_der_y_received;

  if (oa_flow_lpf_enable) {
    float alpha = oa_flow_lpf_alpha;
    float avg_received_f = (float)avg_received;

    if (alpha < 0.f) {
      alpha = 0.f;
    } else if (alpha > 1.f) {
      alpha = 1.f;
    }

    if (!avg_flow_lpf_initialized) {
      avg_flow_lpf_state = avg_received_f;
      avg_flow_lpf_initialized = 1;
    } else {
      avg_flow_lpf_state += alpha * (avg_received_f - avg_flow_lpf_state);
    }

    avg_flow = (int32_t)avg_flow_lpf_state;
  } else {
    avg_flow = avg_received;
    avg_flow_lpf_state = (float)avg_received;
    avg_flow_lpf_initialized = 1;
  }
}


// --- 2. SVM LISTENER (For Visualization) ---
#ifndef SVM_VISUAL_DETECTION_ID
#define SVM_VISUAL_DETECTION_ID ABI_BROADCAST // We will update this ID later to match the SVM!
#endif

static abi_event svm_detection_ev;
static void svm_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t pixel_x, int16_t  pixel_y,
                               int16_t pixel_width, int16_t  pixel_height,
                               int32_t __attribute__((unused)) quality, int16_t __attribute__((unused)) extra)
{
  // Save the AI's coordinates so our drawing function can see them
  svm_bbox_x = pixel_x;
  svm_bbox_y = pixel_y;
  svm_bbox_width = pixel_width;
  svm_bbox_height = pixel_height;
}

static struct video_listener *my_video_listener;

// --- 3. SVM DRAWING FUNCTION ---
static struct image_t * draw_svm_bounding_box(struct image_t *img, uint8_t camera_id){
  (void)camera_id; 
  if (svm_bbox_width > 0 && svm_bbox_height > 0){
    int x_min = svm_bbox_x - (svm_bbox_width / 2);
    int y_min = svm_bbox_y - (svm_bbox_height / 2);
    int x_max = svm_bbox_x + (svm_bbox_width / 2);
    int y_max = svm_bbox_y + (svm_bbox_height / 2);

    // YUV color for bright Green so it contrasts with your red color filter boxes!
    uint8_t green_yuv[3] = {150, 43, 21}; 
    
    // Draw thick green bounding box
    for (int t = 0; t < 3; t++) {
        image_draw_rectangle(img, x_min-t, x_max+t, y_min-t, y_max+t, green_yuv);
    }
  }
  return img;
}
/*
 * Initialisation function, setting random seed and heading increment
 */
void motion_module_group13_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind callback to receive optical-flow outputs
  AbiBindMsgVISUAL_DETECTION(OPTIC_FLOW_VISUAL_DETECTION_ID, &color_detection_ev, optic_flow_cb);
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void motion_module_group13_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  int32_t abs_avg_flow = (avg_flow >= 0) ? avg_flow : -avg_flow;
  int32_t abs_flow_der_x = (flow_der_x >= 0) ? flow_der_x : -flow_der_x;
  int32_t abs_flow_der_y = (flow_der_y >= 0) ? flow_der_y : -flow_der_y;
  float abs_avg_flow_f = (float)abs_avg_flow;
  float flow_mag_f = (float)(abs_flow_der_x + abs_flow_der_y);

  // keep a valid hysteresis band: clear threshold must be below obstacle threshold
  if (oa_hist_flow_clear_threshold >= oa_hist_flow_threshold) {
    oa_hist_flow_clear_threshold = oa_hist_flow_threshold * 0.6f;
  }
  if (oa_gap_balance_threshold >= oa_hist_flow_threshold) {
    oa_gap_balance_threshold = oa_hist_flow_threshold * 0.5f;
  }
  if (oa_emergency_flow_threshold <= oa_hist_flow_threshold) {
    oa_emergency_flow_threshold = oa_hist_flow_threshold * 1.8f;
  }

  uint8_t obstacle_detected = (flow_mag_f >= oa_hist_flow_threshold);
  uint8_t path_clear = (flow_mag_f <= oa_hist_flow_clear_threshold);
  // corridor_open means "close flow exists, but left-right imbalance is small"
  uint8_t corridor_open = (obstacle_detected && abs_avg_flow_f <= oa_gap_balance_threshold);
  uint8_t emergency_close = (flow_mag_f >= oa_emergency_flow_threshold);

  VERBOSE_PRINT("avg_flow: %ld flow_mag: %.1f close_th: %.1f clear_th: %.1f gap_th: %.1f state: %d\n",
                (long)avg_flow, flow_mag_f, oa_hist_flow_threshold,
                oa_hist_flow_clear_threshold, oa_gap_balance_threshold, navigation_state);

  // update confidence with hysteresis around flow peaks
  if (path_clear || corridor_open) {
    obstacle_free_confidence++;
  } else if (obstacle_detected) {
    obstacle_free_confidence -= emergency_close ? 3 : 2;  // be more cautious with strong positive obstacle detections

    // avg_flow > 0 indicates stronger edge flow on the left side than right side.
    // Turn away from the denser side.
    float turn_base_deg = corridor_open ? 2.0f : 5.f;
    float turn_gain_deg = corridor_open ? 8.f : 15.f;
    if (emergency_close) {
      turn_base_deg += 3.f;
      turn_gain_deg += 8.f;
    }
    float peak_ratio = (flow_mag_f - oa_hist_flow_threshold) / (oa_hist_flow_threshold + 1.f);
    Bound(peak_ratio, 0.f, 1.5f);
    float turn_increment = turn_base_deg + turn_gain_deg * peak_ratio;

    if (avg_flow > 0) {
      heading_increment = -turn_increment;
    } else if (avg_flow < 0) {
      heading_increment = turn_increment;
    } else {
      chooseRandomIncrementAvoidance();
    }
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float moveDistance;
  if (path_clear) {
    moveDistance = maxDistance;
  } else if (corridor_open) {
    // Close flow but balanced side-to-side: advance through the gap.
    float symmetry = 1.f - (abs_avg_flow_f / (oa_gap_balance_threshold + 1.f));
    Bound(symmetry, 0.f, 1.f);
    moveDistance = oa_min_forward_distance +
                   symmetry * (0.8f * maxDistance - oa_min_forward_distance);
  } else if (obstacle_detected) {
    moveDistance = emergency_close ? (oa_min_forward_distance * 0.7f) : oa_min_forward_distance;
  } else {
    // Between clear and close thresholds: interpolate forward step.
    float clear_to_close = (oa_hist_flow_threshold - flow_mag_f) /
                           (oa_hist_flow_threshold - oa_hist_flow_clear_threshold);
    Bound(clear_to_close, 0.f, 1.f);
    moveDistance = oa_min_forward_distance +
                   clear_to_close * (maxDistance - oa_min_forward_distance);
  }

  // confidence can still gently reduce stride if recent history is uncertain
  float confidence_scale = 0.5f + 0.1f * obstacle_free_confidence;
  Bound(confidence_scale, 0.5f, 1.f);
  moveDistance *= confidence_scale;

  switch (navigation_state){
    case SAFE:
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_detected && !corridor_open) {
        navigation_state = OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
      }

      break;
    case OBSTACLE_FOUND:
      // Keep moving while turning away, instead of stopping in place.
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.2f * moveDistance);
      moveWaypointForward(WP_GOAL, 0.6f * moveDistance);

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      increase_nav_heading(heading_increment);

      // keep progressing while searching: through open corridors and non-critical zones.
      if (!emergency_close || corridor_open) {
        moveWaypointForward(WP_TRAJECTORY, moveDistance);
        moveWaypointForward(WP_GOAL, moveDistance * 0.7f);
      } else {
        moveWaypointForward(WP_TRAJECTORY, oa_min_forward_distance * 0.6f);
      }

      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence >= 2 && (!obstacle_detected || corridor_open)) {
        navigation_state = SAFE;
      }
      break;
    case OUT_OF_BOUNDS:
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        increase_nav_heading(heading_increment);

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      break;
    default:
      break;
  }
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  nav.heading = new_heading;

  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  VERBOSE_PRINT("RECEIVED parameters: x_dot: %i y_dot: %i  avg_flow: %i\n", flow_der_x, flow_der_y, avg_flow);
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving check_module workings. waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = 5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = -5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}

