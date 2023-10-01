/* Take a binary data dump from dump_data() function through stdin and
 * run it through processing on PC.
 */

#ifndef PIO_UNIT_TESTING
#include <gear_sensor.h>
#include <stdio.h>

int main()
{
    gear_sensor_state_t state = {};
    state.decay_exp = 10;
    state.threshold = 512;

    printf("# ChannelA, ChannelB, Result, MinA, MaxA, MinB, MaxB, BinaryA, BinaryB\n");

    uint8_t data[5];
    while (fread(data, 1, sizeof(data), stdin) == sizeof(data))
    {
        if (data[0] != 0xAA)
        {
            // Resynchronize to data stream
            while (fread(data, 1, 1, stdin) == 1 && data[0] != 0xAA);
            fread(data + 1, 1, sizeof(data) - 1, stdin);
        }

        int16_t a = data[1] | ((uint16_t)data[2] << 8);
        int16_t b = data[3] | ((uint16_t)data[4] << 8);

        int8_t result = gear_sensor_process(&state, a, b);

        printf("%8d, %8d, %8d, %6d, %6d, %6d, %6d, %2d, %2d\n",
            a, b, result,
            state.min_A, state.max_A,
            state.min_B, state.max_B,
            state.binarized & 1, (state.binarized >> 1) & 1);
    }

    return 0;
}
#endif