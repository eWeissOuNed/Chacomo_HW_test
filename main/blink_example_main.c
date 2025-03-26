/* PWM Example for ESP32-C6

   This example configures GPIO 19 as a PWM output with 1000Hz frequency
   and a selectable duty cycle. GPIO 23 is set as an output and can be controlled via serial commands.
   Keyboard commands:
    - 'x' -> Activate Bypass
    - 'y' -> deactivate Bypass
    - 'm' -> Toggle ZD1
    - 'n' -> Toggle ZD2
    - 'c' -> Toggle chacomo_state
    - 'v' -> Read ADC voltage
    - '1' to '9' -> Set PWM duty cycle to 10% to 90%
    */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"


#define ZD1_GPIO 22                // GPIO for Z Diode 1
#define ZD2_GPIO 21                // GPIO for Z Diode 2

#define PWM_GPIO 19                // GPIO for PWM output
#define PWM_FREQUENCY 1000         // Frequency in Hz
#define PWM_RESOLUTION LEDC_TIMER_10_BIT  // Resolution (10-bit for better precision)
#define PWM_CHANNEL LEDC_CHANNEL_0 // LEDC channel
#define OUTPUT_GPIO 23             // GPIO 23 set as output
#define ANALOG_GPIO ADC_CHANNEL_3  // GPIO 3 is ADC1_CH3
#define ADC_MAX_VALUE 3300.0f      // Calibrated max ADC value based on measurements
#define ADC_REF_VOLTAGE 3.3f       // Max measurable voltage with selected attenuation

#define UART_NUM UART_NUM_0
#define BUF_SIZE (1024)

static const char *TAG = "PWM_Example";
static uint32_t duty_cycle = 512;   // Default duty cycle (50%)
static adc_oneshot_unit_handle_t adc_handle;
static int vehicle_state = 0;    // Default vehicle state is 0
static int chacomo_state = 0;    // 0 -> chacomo is inactive, 1 -> chacomo is active

void configure_pwm(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num = PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = duty_cycle,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void configure_gpio(void) {
    gpio_reset_pin(OUTPUT_GPIO);                        //Reset GPIO 23
    gpio_set_direction(OUTPUT_GPIO, GPIO_MODE_OUTPUT);  //Set GPIO 23 as output
    gpio_set_level(OUTPUT_GPIO, 0);                     //Set GPIO 23 to LOW initially

    gpio_reset_pin(ZD1_GPIO);
    gpio_set_direction(ZD1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ZD1_GPIO, 0);

    gpio_reset_pin(ZD2_GPIO);
    gpio_set_direction(ZD2_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ZD2_GPIO, 0);
}

void configure_adc(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = (adc_atten_t)3  // Equivalent to ADC_ATTEN_DB_11
    };
    adc_oneshot_config_channel(adc_handle, ANALOG_GPIO, &config);
}

float read_adc_voltage(void) {
    int adc_raw = 0;
    adc_oneshot_read(adc_handle, ANALOG_GPIO, &adc_raw);
    return (adc_raw / ADC_MAX_VALUE) * ADC_REF_VOLTAGE  * 3.5641f;          // 3.5641 is the voltage divider factor
}

void set_pwm_duty(uint32_t percent) {
    if (percent > 100) percent = 100;
    duty_cycle = (uint32_t)((percent / 100.0f) * 1023);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, duty_cycle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL);
    ESP_LOGI(TAG, "Updated PWM duty cycle to: %lu (from %lu%%)", (unsigned long)duty_cycle, (unsigned long)percent);
}

void set_vehicle_state(uint8_t voltage) {   //set vehicle state based on voltage
    if(4.5 < voltage && voltage < 7.5) {
        vehicle_state = 1;     
    }
    else if(7.5 < voltage && voltage < 10.5) {
        vehicle_state = 2;
    }
    else if(10.5 < voltage && voltage < 13.5) {
        vehicle_state = 3;
    }
    else {
        vehicle_state = 0;
    }
}

void set_gpio_state(uint8_t state) {
    gpio_set_level(OUTPUT_GPIO, state);
    ESP_LOGI(TAG, "Set GPIO %d to: %s", OUTPUT_GPIO, state ? "HIGH" : "LOW");
}

void command_task(void *pvParameter) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0);

    uint8_t data[BUF_SIZE];
    char command_buffer[BUF_SIZE] = {0};
    int buffer_pos = 0;
    int zd1_state = 0;
    int zd2_state = 0;


    while (1) {
        int len = uart_read_bytes(UART_NUM, data, 1, pdMS_TO_TICKS(500));
        if (len > 0) {
            uart_write_bytes(UART_NUM, (const char *)data, len); // Echo back each character

            if(data[0] == 'x') {
                set_gpio_state(1);
            }
            if(data[0] == 'y') {
                set_gpio_state(0);
            }
            if(data[0] == 'm') {                        //Toggle ZD1 when 'm' is pressed
                zd1_state = !zd1_state;
                gpio_set_level(ZD1_GPIO, zd1_state);
            }
            if(data[0] == 'c') {                        //Toggle chacomo_state when 'c' is pressed   
                chacomo_state = !chacomo_state;
            }
            if(data[0] == 'n') {                        //Toggle ZD2 when 'n' is pressed    
                zd2_state = !zd2_state;
                gpio_set_level(ZD2_GPIO, zd2_state);
            }
            if(data[0] == 'v') {
                float voltage = read_adc_voltage();
                char msg[64];
                snprintf(msg, sizeof(msg), "\r\nVoltage: %.2f V\r\n", voltage);
                uart_write_bytes(UART_NUM, msg, strlen(msg));
            }
            if(data[0] >= '1' && data[0] <= '9') {                        //Set Duty cycle when a number is pressed    
                set_pwm_duty((data[0] - '0') * 10);
            }

            if (data[0] == '\n' || data[0] == '\r') {
                command_buffer[buffer_pos] = '\0';
                buffer_pos = 0;

                if (strncmp(command_buffer, "pwm", 3) == 0) {
                    int value;
                    if (sscanf(command_buffer + 4, "%d", &value) == 1) {
                        set_pwm_duty(value);
                    }
                } else if (strncmp(command_buffer, "gpio", 4) == 0) {
                    int state;
                    if (sscanf(command_buffer + 5, "%d", &state) == 1) {
                        set_gpio_state(state);
                    }
                } else if (strcmp(command_buffer, "V?") == 0) {
                    float voltage = read_adc_voltage();
                    char msg[64];
                    snprintf(msg, sizeof(msg), "\r\nVoltage: %.2f V\r\n", voltage);
                    uart_write_bytes(UART_NUM, msg, strlen(msg));
                }

                memset(command_buffer, 0, BUF_SIZE);
            } else if (buffer_pos < BUF_SIZE - 1) {
                command_buffer[buffer_pos++] = data[0];
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void chacomo_task(void *pvParameters) {
    while (1) {
        if(chacomo_state == 1){
            set_vehicle_state(read_adc_voltage());
            ESP_LOGI(TAG, "Chacomo Task adc: %f", read_adc_voltage());
            if(vehicle_state == 1) {
                gpio_set_level(ZD1_GPIO, 0);
                gpio_set_level(ZD2_GPIO, 1);
            }
            if(vehicle_state == 2) {
                gpio_set_level(ZD1_GPIO, 1);
                gpio_set_level(ZD2_GPIO, 0);
            }else {
                gpio_set_level(ZD1_GPIO, 0);
                gpio_set_level(ZD2_GPIO, 0);
            }
        }
        ESP_LOGI(TAG, "Chacomo Task Vehicle State: %d", vehicle_state);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1000 ms
    }
}

void app_main(void) {
    configure_pwm();
    configure_gpio();
    configure_adc();
    xTaskCreate(command_task, "command_task", 16384, NULL, 5, NULL);
    xTaskCreate(chacomo_task, "chacomo_task", 2048, NULL, 5, NULL);
}
