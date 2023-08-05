// This file implements a pedal assist sensor for e-bike use.
// It detects the chainring teeth using linear hall sensors (DRV5053)
// and a magnet.
//
// Pinout:
// PA4 / AIN4: First hall sensor analog output
// PA5:        Power to first hall sensor
// PA6 / AIN6: Second hall sensor analog output
// PA7:        Power to second hall sensor
// PB3:        LED
// PA1:        USART1 TX

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
#define REVERSE     0
#define TICKS_FWD   4
#define TICKS_REV   2
#define TIMEOUT     1000

#include <avr/io.h>
#include <util/delay.h>
#include "gear_sensor.h"
#include <stdio.h>
#include <stdbool.h>

#define LED_PIN (1 << 3)

int usart1_putchar(char c, FILE *stream)
{
  loop_until_bit_is_set(USART1.STATUS, USART_DREIF_bp);
  USART1.TXDATAL = c;
  return 0;
}

FILE usart1_stream = FDEV_SETUP_STREAM(usart1_putchar, NULL, _FDEV_SETUP_WRITE);

void setup()
{
    stdout = &usart1_stream;

    // Set clock to 10 MHz (maximum for 3.3V supply)
    CPU_CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = 0x01;

    // Enable power to hall sensors and enable TX pin
    PORTA.DIR = 0xA2;
    PORTA.OUT = 0xA2;

    // Enable LED output
    PORTB.DIR = LED_PIN;

    // Enable USART1 at 115200 bps
    USART1.BAUD = (64L * F_CPU) / (16 * 115200L);
    USART1.CTRLC = USART_CHSIZE_8BIT_gc;
    USART1.CTRLB = USART_TXEN_bm;

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

int16_t read_adc(int hall_idx)
{
    if (hall_idx == 0)
    {
        ADC0.MUXPOS = ADC_VIA_PGA_gc | ADC_MUXPOS_AIN4_gc;
    }
    else
    {
        ADC0.MUXPOS = ADC_VIA_PGA_gc | ADC_MUXPOS_AIN6_gc;
    }

    loop_until_bit_is_set(ADC0.INTFLAGS, ADC_RESRDY_bp);
    return (int16_t)ADC0.RESULT;
}

void dump_data()
{
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
    }
}

int main()
{
    setup();

    // Uncomment to dump raw data over serial port
    // dump_data();

    gear_sensor_state_t state = {};
    uint8_t dacref = 128;
    uint16_t timeout = 0;
    int8_t steps = 0;
    bool output_active = false;
    
    state.threshold = THRESHOLD;
    state.decay_exp = DECAY;

    while (1)
    {
        // Read ADC values
        int hall1 = read_adc(0);
        int hall2 = read_adc(1);

        // Adjust DC level to keep the signals in measurable range
        if (hall1 > 16384 && hall2 > 16384 && dacref < 255) dacref++;
        if (hall1 < -16384 && hall2 < -16384 && dacref > 0) dacref--;
        AC0.DACREF = dacref;

        // Detect gear movement
        int8_t delta = gear_sensor_process(&state, hall1, hall2);
        
        // Option to reverse direction
        if (REVERSE) delta = -delta;

        // Threshold step count and control output value
        if (delta > 0)
        {
            // One step forward
            USART1.TXDATAL = 'F';
            timeout = 0;
            if (steps < TICKS_FWD)
            {
                steps += delta;
            }
            else
            {
                output_active = true;
            }
        }
        else if (delta < 0)
        {
            // One step backward
            USART1.TXDATAL = 'R';
            timeout = 0;
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

        // Status indication on LED
        // Blinks for each detected edge, shows the output state
        bool led_state = output_active;
        if (delta != 0)
        {
            led_state = !led_state;
        }

        if (led_state)
            PORTB.OUTSET = LED_PIN;
        else
            PORTB.OUTCLR = LED_PIN;        
    }
}
