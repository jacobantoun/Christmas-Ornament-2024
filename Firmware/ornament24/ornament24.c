#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"

#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "lis3dh.h"
#include "hardware/irq.h"
#include "hardware/timer.h"

// Upper Middle LED
#define blinky 0

#define TOTAL_LEDS 16 // Total edge LEDs
#define FLAT_THRESHOLD 5.0f // Threshold in degrees for flat detection
#define TEMP_LED_COUNT 16

// LED for spirit level
#define LED_N       2
#define LED_NNW     3  
#define LED_NW      4
#define LED_WNW     5
#define LED_W       6
#define LED_WSW     7
#define LED_SW      8
#define LED_SSW     9
#define LED_S       18
#define LED_SSE     19
#define LED_SE      20
#define LED_ESE     21
#define LED_E       22
#define LED_ENE     23
#define LED_NE      24
#define LED_NNE     25    
#define LED_MID     14

// Button IO
#define button_wake 16
#define button_mode 17
#define WAKE_GPIO button_wake
#define WAKE_PIN WAKE_GPIO

// Temperature LEDs
#define LED_54      LED_SSW
#define LED_56      LED_SW
#define LED_58      LED_WSW
#define LED_60      LED_W
#define LED_62      LED_WNW
#define LED_64      LED_NW
#define LED_66      LED_NNW
#define LED_68      LED_N
#define LED_70      LED_NNE
#define LED_72      LED_NE
#define LED_74      LED_ENE
#define LED_76      LED_E
#define LED_78      LED_ESE
#define LED_80      LED_SE
#define LED_82      LED_SSE

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

// Array of temperature LEDs for easier access
const uint LED_TEMPERATURES[] = {
    LED_54, LED_56, LED_58, LED_60, LED_62, LED_64, LED_66,
    LED_68, LED_70, LED_72, LED_74, LED_76, LED_78, LED_80, LED_82
};

const float TEMPERATURES[] = {
    54.0f, 56.0f, 58.0f, 60.0f, 62.0f, 64.0f, 66.0f, 68.0f,
    70.0f, 72.0f, 73.0f, 74.0f, 76.0f, 78.0f, 820.0f
};

#define LED_COUNT (sizeof(LED_TEMPERATURES) / sizeof(LED_TEMPERATURES[0]))



// Function to map tilt angle to LED index
int angle_to_led(float angle) 
{
    // Normalize the angle to 0–360 degrees
    if (angle < 0) 
    {
        angle += 360.0f;
    }
    return (int)round(angle / 360.0f * TOTAL_LEDS) % TOTAL_LEDS;
}

void update_leds(float roll, float pitch, int numberOfLEDs) 
{
    // Define LED GPIOs in an array for easy access
    int led_pins[TOTAL_LEDS] = {
        LED_N, LED_NNW, LED_NW, LED_WNW, LED_W, LED_WSW, LED_SW, LED_SSW,
        LED_S, LED_SSE, LED_SE, LED_ESE, LED_E, LED_ENE, LED_NE, LED_NNE
    };

    // Turn off all LEDs
    for (int i = 0; i < TOTAL_LEDS; i++) 
    {
        //gpio_put(led_pins[i], 0);
        setup_pwm_gpio(led_pins[i], 0);
    }
    gpio_put(LED_MID, 0);

    // Calculate the total tilt magnitude
    float tilt_magnitude = sqrtf(pitch * pitch + roll * roll);

    // Check if the board is flat
    if (tilt_magnitude < FLAT_THRESHOLD) 
    {
        // Light up only the middle LED
        gpio_put(LED_MID, 1);
        return;
    }

    // Calculate tilt angle based on pitch and roll
    float angle = atan2f(pitch, roll) * 180.0f / M_PI - 90;

    // Get the primary LED index
    int main_led = angle_to_led(angle);

    // Light up the specified number of LEDs
    for (int i = -numberOfLEDs / 2; i <= numberOfLEDs / 2; i++) 
    {
        int led_index = (main_led + i + TOTAL_LEDS) % TOTAL_LEDS; // Ensure index wraps around
        //gpio_put(led_pins[led_index], 1);
        setup_pwm_gpio(led_pins[led_index], 5000);
    }
}

// Different modes that the ornament is in
#define BOOT_MODE   0
#define TILT_MODE   1
#define TEMP_MODE   2
#define NAP_MODE    3

int current_mode = BOOT_MODE;

volatile int counter = 0;

// Timer interrupt callback
bool timer_callback(repeating_timer_t *rt) 
{
    counter++;
    return true; // Return true to keep the timer running
}


// Store configurations for each slice
static struct {
    uint wrap;
    bool in_use;
} slice_configs[8] = {0};

// Initialize a GPIO pin for PWM output with independent configuration
bool pwm_gpio_init(uint gpio) {
    // Validate GPIO pin
    if (gpio >= 30) return false;  // RP2040 has GPIO 0-29
    
    // Get slice and channel for this GPIO
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    uint channel = pwm_gpio_to_channel(gpio);
    
    // If slice is already in use, only proceed if wrap value matches
    if (slice_configs[slice_num].in_use) {
        // Set GPIO function to PWM but don't reconfigure the slice
        gpio_set_function(gpio, GPIO_FUNC_PWM);
        return true;
    }
    
    // Configure new slice
    slice_configs[slice_num].wrap = 255;
    slice_configs[slice_num].in_use = true;
    
    // Set GPIO function to PWM
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    
    // Configure and enable the slice
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 255);
    pwm_init(slice_num, &config, true);
    
    // Set initial duty cycle to 0
    pwm_set_chan_level(slice_num, channel, 0);
    
    return true;
}

// Set PWM output level (0-255) for a specific GPIO
bool pwm_gpio_put(uint gpio, uint8_t level) {
    if (gpio >= 30) return false;
    
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    uint channel = pwm_gpio_to_channel(gpio);
    
    // Check if slice is initialized
    if (!slice_configs[slice_num].in_use) {
        return false;
    }
    
    pwm_set_chan_level(slice_num, channel, level);
    return true;
}

// Get the slice number for a GPIO (useful for checking potential conflicts)
uint pwm_gpio_get_slice(uint gpio) {
    if (gpio >= 30) return 8;
    return pwm_gpio_to_slice_num(gpio);
}

// Get the channel (A or B) for a GPIO
uint pwm_gpio_get_channel(uint gpio) {
    if (gpio >= 30) return 2;
    return pwm_gpio_to_channel(gpio);
}

void light_led_for_temperature(float temp_f) 
{
    // Turn off all LEDs first
    for (int i = 0; i < LED_COUNT; i++) 
    {
        gpio_put(LED_TEMPERATURES[i], 0);
        //pwm_gpio_put(LED_TEMPERATURES[i], 0);
        //setup_pwm_gpio(LED_TEMPERATURES[i], 0);
    }

    // Find the closest temperature index
    int led_index = -1; // Initialize to invalid index
    for (int i = 0; i < LED_COUNT; i++) 
    {
        if (temp_f >= TEMPERATURES[i] && temp_f < TEMPERATURES[i] + 2.0f) 
        {
            led_index = i;
            break;
        }
    }

    // If a valid index is found, light the corresponding LED
    if (led_index != -1) 
    {
        gpio_put(LED_TEMPERATURES[led_index], 1);
        sleep_ms(1);
        gpio_put(LED_TEMPERATURES[led_index], 0);
        //pwm_gpio_put(LED_TEMPERATURES[led_index], 10);
        //setup_pwm_gpio(led_index, 5000);
    }
}

int main()
{
    stdio_init_all();
    adc_init();

    // Create a repeating timer
    repeating_timer_t timer;

    // Set up a timer to call the callback once every second (1e6 microseconds)
    add_repeating_timer_ms(-1000, timer_callback, NULL, &timer);

    adc_gpio_init(29);
    adc_select_input(3);

    gpio_init(blinky);
    gpio_set_dir(blinky, GPIO_OUT);

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, I2C_FREQ);
    
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    gpio_put(blinky, false);

    // Define LED GPIOs in an array for easy access
    int led_pins_array[17] = {
        LED_N, LED_NNW, LED_NW, LED_WNW, LED_W, LED_WSW, LED_SW, LED_SSW,
        LED_S, LED_SSE, LED_SE, LED_ESE, LED_E, LED_ENE, LED_NE, LED_NNE, LED_MID
    };

    gpio_init(LED_MID);
    gpio_set_dir(LED_MID, GPIO_OUT);
    gpio_put(LED_MID, true);

    // Initialize the accelerometer
    if (!lis3dh_init()) {
        printf("Failed to initialize LIS3DH!\n");
        return -1;
    }
    
    printf("LIS3DH initialized successfully!\n");

    /* From section 7.3 of TMP235 datasheet */
    const float TMP235_Voffs = 0.500f;
    const float TMP235_Tinfl = 0.0f;
    const float TMP235_Tc    = 0.0100f;
    const float adc_conversion_factor = 3.3f / (1 << 12); // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    uint16_t TMP235_raw_val = 0;
    float TMP235_voltage = 0;
    float temp_C = 0;
    float temp_F = 0;

    gpio_init(button_mode);
    gpio_set_dir(button_mode, GPIO_IN);
    gpio_pull_up(button_mode);

    bool init_leds = true;

    int button_mode_pressed_count = 0;
    bool button_mode_pressed = false;

    while (true) {
        if(!gpio_get(button_mode))
        {
            button_mode_pressed_count++;
            if(button_mode_pressed_count > 50)
            {
                button_mode_pressed = true;
                counter = 0;
            }
        }
        else
        {
            button_mode_pressed_count--;
            if(button_mode_pressed_count <= 0)
            {
                button_mode_pressed_count = 0;
            }
            //button_mode_pressed = false;
        }
        if(button_mode_pressed == true)
        {
            for(int j = 0; j <= 16; j++)
            { 
                gpio_init(led_pins_array[j]);
                gpio_set_dir(led_pins_array[j], GPIO_OUT);
                gpio_put(led_pins_array[j], true);
            }
            current_mode++;
            init_leds = true;
            if(current_mode > 2)
            {
                current_mode = 0;
            }
            button_mode_pressed = false;
            button_mode_pressed_count = 0;
        }

        // Put to sleep if inactive for certain amount of time
        if(counter > 5)
        {    
            counter = 0;
        }


        /* Get acceleration values */
        acceleration_t accel = lis3dh_read_acceleration();
        angle_t angles = calculate_angles(accel);
        
        // Convert raw values to g (±2g range)
        float x_g = accel.x * 2.0f / 32768.0f;
        float y_g = accel.y * 2.0f / 32768.0f;
        float z_g = accel.z * 2.0f / 32768.0f;

        //printf("Acceleration: X=%.2fg Y=%.2fg Z=%.2fg\n", x_g, y_g, z_g);
        //printf("Angles: Pitch=%.1f° Roll=%.1f°\n\n", angles.pitch, angles.roll);

        TMP235_raw_val = adc_read();
        TMP235_voltage = TMP235_raw_val * adc_conversion_factor;
        temp_C = (TMP235_voltage - TMP235_Voffs) / TMP235_Tc; // + TMP235_Tinfl);
        temp_F = (9.0f/5.0f) * temp_C + 32.0f;
        //light_led_for_temperature(temp_F);
        //printf("Raw value: 0x%03x, voltage: %f V, Temp: %f F \r\n", TMP235_raw_val, TMP235_voltage, temp_F);
        sleep_ms(20);


        switch(current_mode)
        {
            case BOOT_MODE:
                if(init_leds == true)
                {
                    // Set LEDs as output
                    for(int j = 0; j <= 16; j++)
                    { 
                        gpio_init(led_pins_array[j]);
                        gpio_set_dir(led_pins_array[j], GPIO_OUT);
                        gpio_put(led_pins_array[j], true);
                        gpio_put(led_pins_array[j - 1], false);
                        //pwm_gpio_init(led_pins_array[j]);
                        //pwm_gpio_put(led_pins_array[j], 10);
                        //setup_pwm_gpio(led_pins_array[j - 1], 10);
                        sleep_ms(100);
                    }
                    sleep_ms(250);
                    for(int x = 16; x >= 0; x--)
                    {
                        // gpio_put(led_pins_array[x], true);
                        // gpio_put(led_pins_array[x + 1], false);
                        //pwm_gpio_init(led_pins_array[x]);
                        //pwm_gpio_put(led_pins_array[x], 0);
                        sleep_ms(75);
                    }
                    init_leds = false;
                    current_mode = TILT_MODE;
                }
                break;
            
            case TILT_MODE:
                if(init_leds == true)
                {
                    // Set LEDs as output
                    for(int j = 0; j <= 16; j++)
                    {
                        setup_pwm_gpio(led_pins_array[j], 0);
                    }
                    init_leds = false;
                }
                update_leds(angles.pitch, angles.roll, 3);
                break;

            case TEMP_MODE:
                if(init_leds == true)
                {
                    for(int j = 0; j <= 16; j++)
                    { 
                        gpio_init(led_pins_array[j]);
                        gpio_set_dir(led_pins_array[j], GPIO_OUT);
                        gpio_put(led_pins_array[j], false);
                        //setup_pwm_gpio(j, 000);
                    }
                    init_leds = false;
                }
                light_led_for_temperature(temp_F);
                break;

            case NAP_MODE:

                break;
        }
    }
}
