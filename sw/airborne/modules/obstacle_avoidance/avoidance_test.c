#include "of_avoidance.h"
#include "subsystems/abi.h" // Required for ABI messaging [cite: 786, 789]
#include "firmwares/rotorcraft/guidance/guidance_h.h" // Required for GUIDED mode [cite: 947]
#include <stdio.h> // Required for printf debugging [cite: 1052, 1053]

// --- Define Default Tuning Parameters [cite: 1030] ---
#ifndef OF_AVOIDANCE_DIV_THRESHOLD
#define OF_AVOIDANCE_DIV_THRESHOLD 0.3f 
#endif

#ifndef OF_AVOIDANCE_SPEED
#define OF_AVOIDANCE_SPEED -0.5f 
#endif

// Global variables connected to the GCS settings [cite: 1034, 1035]
float avoidance_divergence_threshold = OF_AVOIDANCE_DIV_THRESHOLD;
float avoidance_speed = OF_AVOIDANCE_SPEED;

// ABI event structure [cite: 797]
static abi_event opticflow_ev;

// --- ABI Callback Function [cite: 798, 799] ---
static void opticflow_cb(uint8_t sender_id, uint32_t stamp, 
                         int16_t flow_x, int16_t flow_y, 
                         int16_t flow_der_x, int16_t flow_der_y, 
                         float quality, float div_size) // div_size is the divergence [cite: 824]
{
    // Check if the divergence value is higher than our threshold [cite: 825]
    if (div_size > avoidance_divergence_threshold) {
        
        printf("OBSTACLE FOUND! Divergence: %f\n", div_size); // Print to terminal [cite: 1053]

        // Only send commands if the drone is actually in GUIDED mode [cite: 949, 950]
        if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED) {
            // Command a negative X velocity to back away from the obstacle 
            guidance_h_set_guided_body_vel(avoidance_speed, 0.0f);
        }
    }
}

// --- Initialization Function [cite: 598] ---
void of_avoidance_init(void) {
    // Subscribe to the OPTICAL_FLOW message class [cite: 790]
    // ABI_BROADCAST accepts messages from all senders [cite: 796]
    AbiBindMsgOPTICAL_FLOW(ABI_BROADCAST, &opticflow_ev, opticflow_cb); 
}