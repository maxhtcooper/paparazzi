#ifndef OF_AVOIDANCE_H
#define OF_AVOIDANCE_H

// Variables that we will expose to the GCS for live tuning
extern float avoidance_divergence_threshold;
extern float avoidance_speed;

// Initialization function called at startup
extern void of_avoidance_init(void);

#endif