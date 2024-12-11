#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

#define blinky 0

// Array to track initialized slices (8 slices on the RP2040)
bool pwm_slice_initialized[8] = {false};

// Function to set a GPIO as PWM and configure its output value
void setup_pwm_gpio(uint gpio, uint16_t duty_cycle) {
    // Set the GPIO to PWM function
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    // Get the PWM slice and channel for the GPIO
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    uint channel = pwm_gpio_to_channel(gpio);

    // Check if the slice has already been initialized
    if (!pwm_slice_initialized[slice_num]) {
        // Configure the PWM slice only once
        pwm_config config = pwm_get_default_config();
        pwm_config_set_clkdiv(&config, 4.0f); // Adjust clock divider as needed
        pwm_init(slice_num, &config, true);   // Initialize and enable the PWM slice
        pwm_slice_initialized[slice_num] = true; // Mark the slice as initialized
    }

    // Set the duty cycle for the specified GPIO (channel)
    pwm_set_chan_level(slice_num, channel, duty_cycle);
}



int main()
{
    stdio_init_all();
    adc_init();

    adc_gpio_init(29);
    adc_select_input(3);

    gpio_init(blinky);
    gpio_set_dir(blinky, GPIO_OUT);

    for(int j = 2; j <= 25; j++)
    {
        if(j == 10)
        {
            j = 14;
        }
        if(j == 15)
        {
            j = 18;
        }
        setup_pwm_gpio(j, 5000);
        sleep_ms(300);
    }

    while (true) {
        // gpio_put(blinky, true);
        // sleep_ms(250);
        // gpio_put(blinky, false);
        sleep_ms(250);
        // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
        const float conversion_factor = 3.3f / (1 << 12);
        uint16_t result = adc_read();
        printf("Raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);
    }
}
