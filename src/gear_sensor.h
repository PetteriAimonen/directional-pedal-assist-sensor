/* Signal processing for the gear sensor */

#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    // Settings
    uint16_t threshold;  // Minimum pulse height for detection
    uint16_t decay_exp;  // Exponent of decay time, e.g. 8 = exponential decay in 256 samples
    
    // Track min/max values of both channels.
    bool initialized;
    uint16_t min_A, min_B;
    uint16_t max_A, max_B;

    // Exponential decay of min/max towards current value
    uint16_t decay_min_A, decay_min_B;
    uint16_t decay_max_A, decay_max_B;

    // Previous binarized state of quadrature signals
    uint8_t binarized;
} gear_sensor_state_t;

// Process analog signals from gear sensor and provide step count of -1, 0 or +1.
// A and B are the signal values read from ADC
// Threshold is the minimum pulse height to enable detection.
int8_t gear_sensor_process(gear_sensor_state_t *state, uint16_t a, uint16_t b);
