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
#include <math.h>

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

static struct EnuCoor_i moveWaypointForward(uint8_t waypoint, float distanceMeters, float headingIncrementDegrees);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters, float headingIncrementDegrees);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t align_nav_heading_to_straight_section(void);
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
#define MOTION_GROUP13_FLOW_LPF_ALPHA 0.12f
#endif

#ifndef MOTION_GROUP13_HIST_FLOW_THRESHOLD
#define MOTION_GROUP13_HIST_FLOW_THRESHOLD 35.f
#endif

#ifndef MOTION_GROUP13_MIN_FORWARD_DISTANCE
#define MOTION_GROUP13_MIN_FORWARD_DISTANCE 0.35f
#endif

#ifndef MAX_CONFIDENCE_LEVEL
#define MAX_CONFIDENCE_LEVEL 10
#endif

#ifndef RESET_CONFIDENCE_LEVEL
#define RESET_CONFIDENCE_LEVEL 10
#endif

#ifndef TURN_BASE_DEG
#define TURN_BASE_DEG 12
#endif

#ifndef TURN_GAIN_DEG
#define TURN_GAIN_DEG 0.05f
#endif

#ifndef MAX_DIST
#define MAX_DIST 0.8f
#endif

// define settings  
float oa_hist_flow_threshold = MOTION_GROUP13_HIST_FLOW_THRESHOLD;
float oa_min_forward_distance = MOTION_GROUP13_MIN_FORWARD_DISTANCE;
uint8_t oa_flow_lpf_enable = MOTION_GROUP13_FLOW_LPF_ENABLE;
float oa_flow_lpf_alpha = MOTION_GROUP13_FLOW_LPF_ALPHA;
float turn_base_deg = TURN_BASE_DEG;
float turn_gain_deg = TURN_GAIN_DEG;
float max_distance = MAX_DIST;               // max waypoint displacement [m]

// define and initialise global variables
enum navigation_state_t navigation_state = SAFE;

int32_t avg_flow = 0;
uint8_t sparse_bin;
static float avg_flow_lpf_state = 0.f;
static uint8_t avg_flow_lpf_initialized = 0;
static float straight_section_heading = 0.f;
static uint8_t straight_section_heading_initialized = 0;
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float heading_increment = 50.f;          // heading angle increment [deg]
float oob_hdg_incr_deg = 30.f;

uint8_t reset_confidence_level = RESET_CONFIDENCE_LEVEL;
uint8_t max_trajectory_confidence = MAX_CONFIDENCE_LEVEL; // number of consecutive negative object detections to be sure we are obstacle free

// edge histogram bin will give general headings
// edge flow gives the depth (closer object has larger edge flow so it will weigh more when chosing the heading)

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
                               int16_t  __attribute__((unused)), uint8_t sparse_bin_received,
                               int32_t avg_received, int16_t __attribute__((unused)) extra)
{
  sparse_bin = sparse_bin_received;
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

  straight_section_heading = stateGetNedToBodyEulers_f()->psi;
  straight_section_heading_initialized = 1;
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
  float abs_avg_flow_f = (float)abs_avg_flow;

  uint8_t obstacle_detected = (abs_avg_flow_f >= oa_hist_flow_threshold);

  float turn_increment = 0.f;

  if (obstacle_detected) {
    obstacle_free_confidence -= 1; // -1 for consistency and to filter jitter when yawing
    Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

    // only deal with turning if we detect obstacles many times in a row
    // since we require the drone to fly straight for a while for the optic flow
    if (obstacle_free_confidence == 0) {
      // avg_flow > 0 indicates stronger edge flow on the left side than right side.
      // so + avg flow means objects moving to the left more so objects close on the left so turn right
      // - avg flow means the opposite so turn left      

      turn_increment = turn_base_deg + turn_gain_deg * abs_avg_flow_f;

      if (avg_flow > 0) {
        heading_increment = turn_increment;
      } else {
        heading_increment = -turn_increment;
      }

      navigation_state = OBSTACLE_FOUND;
    }

  } else {
    // Find out which bit closest to the center of the sparse map is 0
    // and calculate a heading increment that will center that would center that
    obstacle_free_confidence += 2; // 2 for consistency and to filter jitter when yawing
  }

  VERBOSE_PRINT("avg_flow: %d sparse_bin: 0b%d%d%d%d%d%d%d%d state: %d detection: %d turning: %.1f\n",
                avg_flow,
                (sparse_bin >> 7) & 1, (sparse_bin >> 6) & 1, (sparse_bin >> 5) & 1, (sparse_bin >> 4) & 1,
                (sparse_bin >> 3) & 1, (sparse_bin >> 2) & 1, (sparse_bin >> 1) & 1, sparse_bin & 1,
                navigation_state, obstacle_detected, heading_increment);

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float move_distance = obstacle_detected ? oa_min_forward_distance : max_distance;

  // confidence can still gently reduce stride if recent history is uncertain
  float confidence_scale = 0.5f + 0.1f * obstacle_free_confidence;
  Bound(confidence_scale, 0.5f, 1.f);
  move_distance *= confidence_scale;

  switch (navigation_state){
    case SAFE:
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 1.f * move_distance, 0.f);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else {
        struct EnuCoor_i new_goal = moveWaypointForward(WP_GOAL, move_distance, 0.f);
        // Update heading reference based on vector from current position to new goal
        struct EnuCoor_i current_pos = *stateGetPositionEnu_i();
        float dx = POS_FLOAT_OF_BFP(new_goal.x - current_pos.x);
        float dy = POS_FLOAT_OF_BFP(new_goal.y - current_pos.y);
        straight_section_heading = atan2f(dx, dy);
      }

      break;
    case OBSTACLE_FOUND:
      // move waypoint forward and at an angle compared to the previous heading
      moveWaypointForward(WP_TRAJECTORY, 1.f * move_distance, heading_increment);

      navigation_state = SAFE;
      // reset this so we fly straight after turning
      obstacle_free_confidence = reset_confidence_level;

      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else {
        struct EnuCoor_i new_goal = moveWaypointForward(WP_GOAL, move_distance, heading_increment);
        // Update heading reference based on vector from current position to new goal
        struct EnuCoor_i current_pos = *stateGetPositionEnu_i();
        float dx = POS_FLOAT_OF_BFP(new_goal.x - current_pos.x);
        float dy = POS_FLOAT_OF_BFP(new_goal.y - current_pos.y);
        straight_section_heading = atan2f(dx, dy);
        // Follow the new straight section heading with a coordinated turn.
        align_nav_heading_to_straight_section();
      }

      break;
    case OUT_OF_BOUNDS:
      moveWaypointForward(WP_TRAJECTORY, 1.f, oob_hdg_incr_deg);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        struct EnuCoor_i new_goal = moveWaypointForward(WP_GOAL, oa_min_forward_distance, oob_hdg_incr_deg);
        // Update heading reference based on vector from current position to new goal
        struct EnuCoor_i current_pos = *stateGetPositionEnu_i();
        float dx = POS_FLOAT_OF_BFP(new_goal.x - current_pos.x);
        float dy = POS_FLOAT_OF_BFP(new_goal.y - current_pos.y);
        straight_section_heading = atan2f(dx, dy);

        // Turn toward the newly established straight section heading.
        align_nav_heading_to_straight_section();

        // reset safe counter
        obstacle_free_confidence = reset_confidence_level;

        // ensure direction is safe before continuing
        navigation_state = SAFE;
      } else {
        oob_hdg_incr_deg += 30.f;
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

  return false;
}

/*
 * Turns NAV heading by the delta between current psi and straight_section_heading.
 */
uint8_t align_nav_heading_to_straight_section(void)
{
  float current_heading = stateGetNedToBodyEulers_f()->psi;
  float delta_heading = straight_section_heading - current_heading;
  FLOAT_ANGLE_NORMALIZE(delta_heading);
  return increase_nav_heading(DegOfRad(delta_heading));
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates.
 * Returns the new waypoint coordinates.
 */
struct EnuCoor_i moveWaypointForward(uint8_t waypoint, float distanceMeters, float headingIncrementDegrees)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters, headingIncrementDegrees);
  moveWaypoint(waypoint, &new_coor);
  return new_coor;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters, float headingIncrementDegrees)
{
  float heading = straight_section_heading + RadOfDeg(headingIncrementDegrees);
  FLOAT_ANGLE_NORMALIZE(heading);

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
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
  } else {
    heading_increment = -5.f;
  }
  return false;
}

