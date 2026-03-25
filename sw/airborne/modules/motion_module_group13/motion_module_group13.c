/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/motion_module_group13/motion_module_group13.c"
 * @author Roland Meertens
 * Combined module: Optic flow avoidance with a 30% Orange Color Detection Failsafe.
 */

#include "modules/motion_module_group13/motion_module_group13.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

// Includes to visualize bounding boxes and access camera settings
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/video_thread.h" // Needed for front_camera size

#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[motion_module_group13->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Forward declarations
static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseDirectionalAvoidance(int16_t current_flow_der_x);

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

#ifndef MOTION_GROUP13_FLOW_DER_THRESHOLD
#define MOTION_GROUP13_FLOW_DER_THRESHOLD 200
#endif

#ifndef MOTION_GROUP13_AVG_FLOW_THRESHOLD
#define MOTION_GROUP13_AVG_FLOW_THRESHOLD 350
#endif

#ifndef MOTION_GROUP13_DIVERGENCE_THRESHOLD
#define MOTION_GROUP13_DIVERGENCE_THRESHOLD 50
#endif

// External camera reference for threshold calculation
extern struct video_config_t front_camera;

// --- Settings exposed to XML / GCS ---
float oa_color_count_frac = 0.30f; // HARD DEFAULT TO 30%
uint8_t oa_flow_lpf_enable = MOTION_GROUP13_FLOW_LPF_ENABLE;
float oa_flow_lpf_alpha = MOTION_GROUP13_FLOW_LPF_ALPHA;
int32_t oa_avg_flow_threshold = MOTION_GROUP13_AVG_FLOW_THRESHOLD;

// Internal thresholds
int16_t oa_flow_der_threshold = MOTION_GROUP13_FLOW_DER_THRESHOLD;
int16_t oa_divergence_threshold = MOTION_GROUP13_DIVERGENCE_THRESHOLD;

// Global state variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;

// Optic Flow Variables
int16_t flow_der_x = 0;
int16_t flow_der_y = 0;
int32_t avg_flow = 0;
int16_t divergence = 0;
static float avg_flow_lpf_state = 0.f;
static uint8_t avg_flow_lpf_initialized = 0;

// Failsafe Color Count Variable
int32_t color_count = 0;                

// Navigation Variables
int16_t obstacle_free_confidence = 0;   
float heading_increment = 5.f;          
float maxDistance = 2.25;               
int turn_counter = 0;                   

const int16_t max_trajectory_confidence = 5; 
const int min_turn_cycles = 10;              

/*
 * ABI bindings for Optic Flow
 */
#ifndef OPTIC_FLOW_VISUAL_DETECTION_ID
#define OPTIC_FLOW_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

static abi_event optic_flow_ev;
static void optic_flow_cb(uint8_t __attribute__((unused)) sender_id,
                          int16_t __attribute__((unused)) flow_x, int16_t __attribute__((unused)) flow_y,
                          int16_t flow_der_x_received, int16_t flow_der_y_received,
                          int32_t avg_received, int16_t __attribute__((unused)) extra)
{
  flow_der_x = flow_der_x_received;
  flow_der_y = flow_der_y_received;
  divergence = extra;

  if (oa_flow_lpf_enable) {
    float alpha = oa_flow_lpf_alpha;
    float avg_received_f = (float)avg_received;

    if (alpha < 0.f) alpha = 0.f;
    else if (alpha > 1.f) alpha = 1.f;

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

/*
 * ABI bindings for Color Detection (Failsafe)
 */
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
}

/*
 * Initialisation function
 */
void motion_module_group13_init(void)
{
  srand(time(NULL));
  heading_increment = 5.f;
  
  AbiBindMsgVISUAL_DETECTION(OPTIC_FLOW_VISUAL_DETECTION_ID, &optic_flow_ev, optic_flow_cb);
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

/*
 * Periodic Function
 */
void motion_module_group13_periodic(void)
{
  if(!autopilot_in_flight()){
    return;
  }

  // --- 1. EVALUATE OPTIC FLOW PIPELINE ---
  bool optic_flow_safe = (abs(avg_flow) < oa_avg_flow_threshold);

  // We temporarily ignore optic flow while spinning to let the confidence counter recover
  if (navigation_state == SEARCH_FOR_SAFE_HEADING) {
      optic_flow_safe = true; 
  }

  // --- 2. EVALUATE EMERGENCY FAILSAFE (>30% ORANGE) ---
  int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  static int color_danger_counter = 0; 

  if (color_count >= color_count_threshold) {
      color_danger_counter++; // Increment if 30+% orange is seen
  } else {
      color_danger_counter = 0; // Reset immediately if screen clears up
  }

  // Failsafe triggers ONLY if screen is >= 30% orange for 2 consecutive frames
  bool color_safe = (color_danger_counter < 2);

  // Update confidence based on both pipelines
  if (optic_flow_safe && color_safe) {
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence--; 
  }
  
  VERBOSE_PRINT("State: %d avg_flow: %d color_count: %d (Thr: %d) conf: %d\n", 
                navigation_state, avg_flow, color_count, color_count_threshold, obstacle_free_confidence);

  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);
  float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);

  switch (navigation_state){
    case SAFE:
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
      
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } 
      // TRIGGER AVOIDANCE IF FAILSAFE TRIPPED *OR* OPTIC FLOW IS BLOCKED
      else if (!color_safe || obstacle_free_confidence == 0 || !optic_flow_safe){
        
        chooseDirectionalAvoidance(flow_der_x);
        navigation_state = OBSTACLE_FOUND;
        
        if (!color_safe) {
            VERBOSE_PRINT("F A I L S A F E : 30%%+ Orange Detected! Emergency Stop.\n");
        } else {
            VERBOSE_PRINT("O B S T A C L E : Optic flow pipeline blocked.\n");
        }
        
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
      }
      break;

    case OBSTACLE_FOUND:
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);
      chooseDirectionalAvoidance(flow_der_x);
      turn_counter = 0;
      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;

    case SEARCH_FOR_SAFE_HEADING:
      increase_nav_heading(heading_increment);
      turn_counter++;

      if (obstacle_free_confidence >= 3 && turn_counter >= min_turn_cycles){
        navigation_state = SAFE;
        turn_counter = 0; 
      }
      break;

    case OUT_OF_BOUNDS:
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        increase_nav_heading(heading_increment);
        obstacle_free_confidence = 0;
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      break;
      
    default:
      break;
  }
  return;
}

uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav.heading = new_heading;
  return false;
}

uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  return false;
}

uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

static uint8_t chooseDirectionalAvoidance(int16_t current_flow_der_x)
{
  if (rand() % 2 == 0) {
    heading_increment = 5.f;
  } else {
    heading_increment = -5.f;
  }
  return false;
}