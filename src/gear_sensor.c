#include "gear_sensor.h"

// Binarize the value of a single channel with hysteresis.
static bool binarize_channel(bool oldstate, uint16_t value, uint16_t min, uint16_t max, uint16_t threshold)
{
    uint16_t height = max - min;
    if (height >= threshold)
    {
        // Good pulse height
        uint16_t threshold_low  = min + height / 4;
        uint16_t threshold_high = max - height / 4;
        if (value < threshold_low)
        {
            return false;
        }
        else if (value > threshold_high)
        {
            return true;
        }
        else
        {
            return oldstate;
        }
    }
    else
    {
        // Too low pulse height, keep old state
        return oldstate;
    }
}

int8_t gear_sensor_process(gear_sensor_state_t *state, uint16_t a, uint16_t b)
{
    // Initialize on first call
    if (!state->initialized)
    {
        state->initialized = true;
        state->max_A = state->min_A = a;
        state->max_B = state->min_B = b;
        state->decay_min_A = state->decay_min_B = 0;
        state->decay_max_A = state->decay_max_B = 0;
        state->binarized = 0;
    }

    // Update min/max values
    if (a < state->min_A) state->min_A = a; else state->decay_min_A += (a - state->min_A);
    if (a > state->max_A) state->max_A = a; else state->decay_max_A += (state->max_A - a);
    if (b < state->min_B) state->min_B = b; else state->decay_min_B += (b - state->min_B);
    if (b > state->max_B) state->max_B = b; else state->decay_max_B += (state->max_B - b);

    // Exponential decay towards current value
    // Implemented using separate accumulator to avoid rounding error.
    uint8_t bits = state->decay_exp;
    uint16_t mask = (1 << bits) - 1;
    state->min_A += (state->decay_min_A >> bits); state->decay_min_A &= mask;
    state->max_A -= (state->decay_max_A >> bits); state->decay_max_A &= mask;
    state->min_B += (state->decay_min_B >> bits); state->decay_min_B &= mask;
    state->max_B -= (state->decay_max_B >> bits); state->decay_max_B &= mask;

    // Update binarized state for channels
    bool oldA = state->binarized & 1;
    bool oldB = state->binarized & 2;
    bool newA = binarize_channel(oldA, a, state->min_A, state->max_A, state->threshold);
    bool newB = binarize_channel(oldB, b, state->min_B, state->max_B, state->threshold);
    state->binarized = (newA ? 1 : 0) | (newB ? 2 : 0);
    
    // Decode quadrature changes
    bool changeA = (newA != oldA);
    bool changeB = (newB != oldB);
    
    if (changeA && !changeB)
    {
        return (newA == newB) ? 1 : -1;
    }
    else if (changeB && !changeA)
    {
        return (newA == newB) ? -1 : 1;
    }
    else
    {
        // No movement or indeterminate movement
        return 0;
    }
}
