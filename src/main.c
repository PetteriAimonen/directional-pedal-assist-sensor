// This file implements a pedal assist sensor for e-bike use.
// It detects the chainring teeth using linear hall sensors (DRV5053)
// and a magnet.
//
// Pinout:
// PA4 / AIN4: First hall sensor analog signal
// PA6 / AIN6: Second hall sensor analog signal
// PB3:        LED
// PA1:        USART1 TX
// PA2:        Output signal
#define AIN_HALL0   4
#define AIN_HALL1   6
#define LED_PIN     3
#define USART_TX    1
#define OUT_PIN     2

// Detection parameters
// THRESHOLD:  Smaller value gives longer detection distance but is more sensitive to noise
// DECAY:      Smaller value makes min/max follower react faster to DC bias changes
// REVERSE:    Invert fwd/rev directions
// TICKS_FWD:  Minimum number of teeth in forward direction to enable the output
// TICKS_REV:  Minimum number of teeth in backward direction to disable the output
// TIMEOUT:    Time without detection to disable the output (milliseconds)
#define THRESHOLD   512
#define DECAY       10
#define MAXTIME     500
#define MINTIME     10
#define REVERSE     1
#define TICKS_FWD   4
#define TICKS_REV   2
#define TIMEOUT     1000

// IO parameters
// ENABLE_UART: Enable UART output on TX/PA1 pin (if disabled, outputs same as PA2)
// DATA_DUMP:   Instead of gear detection, just dump raw ADC data to UART TX pin
#define ENABLE_UART     0
#define DATA_DUMP       0

#include <avr/io.h>
#include <util/delay.h>
#include "gear_sensor.h"
#include <stdio.h>
#include <stdbool.h>

void setup()
{
    // Set clock to 10 MHz (maximum for 3.3V supply)
    CPU_CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = 0x01;

    // Enable slew rate limitation (reduces noise)
    PORTA.PORTCTRL = PORT_SRL_bm;
    PORTB.PORTCTRL = PORT_SRL_bm;

    // Enable TX and OUT pins
    PORTA.DIR = (1 << USART_TX) | (1 << OUT_PIN);

    // Enable LED output
    PORTB.DIR = (1 << LED_PIN);

#if ENABLE_UART || DATA_DUMP
    // Enable USART1 at 115200 bps
    USART1.BAUD = (64L * F_CPU) / (16 * 115200L);
    USART1.CTRLC = USART_CHSIZE_8BIT_gc;
    USART1.CTRLB = USART_TXEN_bm;
#endif

    // Enable DACREF for subtracting the static magnetic field
    VREF.CTRLA = VREF_AC0REFSEL_2V5_gc;
    VREF.CTRLB = VREF_ADC0REFEN_bm | VREF_AC0REFEN_bm;
    AC0.DACREF = 100;

    // Enable ADC
    // Vref: internal 1.024 V
    // ADCCLK: 2.5 MHz
    // Averaging: 16 samples
    // PGA: enabled, 4x gain
    // PGA negative: DACREF
    ADC0.CTRLA = ADC_ENABLE_bm;
    ADC0.CTRLB = ADC_PRESC_DIV4_gc;
    ADC0.CTRLC = ADC_REFSEL_1024MV_gc | ADC_TIMEBASE2_bm;
    ADC0.CTRLE = 10;
    ADC0.CTRLF = ADC_LEFTADJ_bm | ADC_SAMPNUM_ACC16_gc;
    ADC0.PGACTRL = ADC_GAIN_4X_gc | ADC_PGABIASSEL_1_2X_gc | ADC_ADCPGASAMPDUR_20CLK_gc | ADC_PGAEN_bm;
    ADC0.MUXNEG = ADC_VIA_PGA_gc | ADC_MUXNEG_DACREF0_gc;
    ADC0.COMMAND = ADC_DIFF_bm | ADC_MODE_BURST_SCALING_gc | ADC_START_MUXPOS_WRITE_gc;
}

// Read single ADC channel, with hardware oversampling
int16_t read_adc(int hall_idx)
{
    if (hall_idx == 0)
    {
        ADC0.MUXPOS = ADC_VIA_PGA_gc | AIN_HALL0;
    }
    else
    {
        ADC0.MUXPOS = ADC_VIA_PGA_gc | AIN_HALL1;
    }

    loop_until_bit_is_set(ADC0.INTFLAGS, ADC_RESRDY_bp);
    return (int16_t)ADC0.RESULT;
}

// Adjust PGA negative reference level to keep the signals in measurable range
void adjust_dc_bias(int16_t hall1, int16_t hall2)
{
    static uint8_t dacref = 100;
    if (hall1 > 16384 && hall2 > 16384 && dacref < 255) dacref++;
    if (hall1 < -16384 && hall2 < -16384 && dacref > 0) dacref--;
    AC0.DACREF = dacref;
}

// Continuously send data to serial port
// Format is 5 bytes: SYNC, SENSOR1_LSB, SENSOR1_MSB, SENSOR2_LSB, SENSOR2_MSB.
// Sync byte is always 0xAA. Values encoded as signed 16-bit little-endian.
void dump_data()
{
    uint8_t dacref = 128;

    while(true)
    {
        loop_until_bit_is_set(USART1.STATUS, USART_DREIF_bp);
        USART1.TXDATAL = 0xAA;

        uint16_t hall1 = (uint16_t)read_adc(0);
        loop_until_bit_is_set(USART1.STATUS, USART_DREIF_bp);
        USART1.TXDATAL = (hall1 & 0xFF);
        loop_until_bit_is_set(USART1.STATUS, USART_DREIF_bp);
        USART1.TXDATAL = (hall1 >> 8);

        uint16_t hall2 = (uint16_t)read_adc(1);
        loop_until_bit_is_set(USART1.STATUS, USART_DREIF_bp);
        USART1.TXDATAL = (hall2 & 0xFF);
        loop_until_bit_is_set(USART1.STATUS, USART_DREIF_bp);
        USART1.TXDATAL = (hall2 >> 8);

        adjust_dc_bias(hall1, hall2);
    }
}

int main()
{
    setup();

#if DATA_DUMP
    // Dump raw ADC data over serial port.
    // Use tests/process_dump.c to analyze.
    dump_data();
#endif

    gear_sensor_state_t state = {};
    uint16_t timeout = 0;
    int8_t steps = 0;
    bool output_active = false;

    state.threshold = THRESHOLD;
    state.decay_exp = DECAY;

    while (1)
    {
        // Read ADC values
        int16_t hall1 = read_adc(0);
        int16_t hall2 = read_adc(1);
        adjust_dc_bias(hall1, hall2);

        // Detect gear movement
        int8_t delta = gear_sensor_process(&state, hall1, hall2);

        // Option to reverse direction
        if (REVERSE) delta = -delta;

        // Threshold step count and control output value
        if (delta > 0)
        {
            if (timeout > TIMEOUT)
            {
                // If restarting after timeout, give some
                // time for steps to accumulate.
                timeout = 0;
            }

            // One step forward
            if (steps < TICKS_FWD)
            {
                steps += delta;
            }
            else
            {
                timeout = 0;
                output_active = true;
            }
        }
        else if (delta < 0)
        {
            // One step backward
            if (steps > TICKS_REV)
            {
                steps += delta;
            }
            else
            {
                output_active = false;
            }
        }
        else
        {
            // No steps, keep track of timeout
            if (timeout > TIMEOUT)
            {
                steps = 0;
                output_active = false;
            }
            else
            {
                timeout++;
            }
        }

#if ENABLE_UART
        // Send direction to UART
        if (delta > 0)
        {
            USART1.TXDATAL = 'F';
        }
        else if (delta < 0)
        {
            USART1.TXDATAL = 'R';
        }
#endif

        // Output state on OUT pin and on UART pin (unless UART takes it over)
        if (output_active)
        {
            PORTA.OUTSET = (1 << USART_TX) | (1 << OUT_PIN);
        }
        else
        {
            PORTA.OUTCLR = (1 << USART_TX) | (1 << OUT_PIN);
        }

        // Status indication on LED
        // Blinks for each detected edge, shows the output state
        bool led_state = output_active;
        if (delta != 0)
        {
            led_state = !led_state;
        }

        if (led_state)
            PORTB.OUTSET = (1 << LED_PIN);
        else
            PORTB.OUTCLR = (1 << LED_PIN);
    }
}
