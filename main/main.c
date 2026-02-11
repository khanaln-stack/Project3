#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "freeetos/task.h"
#include "driver/gpio.h"

// Ignition Pins
#define D_SEAT GPIO_NUM_4
#define P_SEAT GPIO_NUM_5
#define D_SEATBELT GPIO_NUM_6
#define P_SEATBELT GPIO_NUM_7
#define IGNITION_LED GPIO_NUM_9
#define ENGINE_LED GPIO_NUM_10
#define BUZZER GPIO_NUM_11
#define IGNITION GPIO_NUM_12

// LCD Pins
#define LCD_RS GPIO_NUM_38
#define LCD_E  GPIO_NUM_37
#define LCD_D4 GPIO_NUM_36
#define LCD_D5 GPIO_NUM_35
#define LCD_D6 GPIO_NUM_48
#define LCD_D7 GPIO_NUM_47

// Servo 
#define SERVO_GPIO     GPIO_NUM_21

#define LEDC_TIMER     LEDC_TIMER_0
#define LEDC_MODE      LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL   LEDC_CHANNEL_0
#define LEDC_DUTY_RES  LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY 50

#define LEDC_DUTY_MIN  307
#define LEDC_DUTY_MAX  921

#define LOOP_MS        10

// ADC 
#define ADC_ATTEN   ADC_ATTEN_DB_12
#define BITWIDTH    ADC_BITWIDTH_12

#define MODE_ADC_CH   ADC_CHANNEL_2   // GPIO3
#define DELAY_ADC_CH  ADC_CHANNEL_7   // GPIO8

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t cali_mode_handle;
adc_cali_handle_t cali_delay_handle;

hd44780_t lcd;

void app_main(void)
{

    gpio_reset_pin(D_SEAT);
    gpio_set_direction(D_SEAT, GPIO_MODE_INPUT);
    gpio_pullup_en(D_SEAT);

    gpio_reset_pin(P_SEAT);
    gpio_set_direction(P_SEAT, GPIO_MODE_INPUT);
    gpio_pullup_en(P_SEAT);

    gpio_reset_pin(D_SEATBELT);
    gpio_set_direction(D_SEATBELT, GPIO_MODE_INPUT);
    gpio_pullup_en(D_SEATBELT);

    gpio_reset_pin(P_SEATBELT);
    gpio_set_direction(P_SEATBELT, GPIO_MODE_INPUT);
    gpio_pullup_en(P_SEATBELT);

    gpio_reset_pin(IGNITION);
    gpio_set_direction(IGNITION, GPIO_MODE_INPUT);
    gpio_pullup_en(IGNITION);

    gpio_reset_pin(BUZZER);
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER, 0);

    gpio_reset_pin (IGNITION_LED);
    gpio_set_direction(IGNITION_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(IGNITION_LED,0);

    gpio_reset_pin(ENGINE_LED);
    gpio_set_direction(ENGINE_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(ENGINE_LED,0);

    bool d_seat, p_seat; // Seat occupancy states
    bool d_belt, p_belt; // Seatbelt states

    bool ignit = false;            //Ignition button state
    bool ignition_enabled= false;  //Indicates ignition readiness
    bool engine_running = false;   //Indicates engine running

    bool prev_state = false;
    bool last_ignit = false;
 
    bool welcome_not_shown = true;  //Ensures welcome message prints once

   while (1)
{
    // Read all inputs (active-low)
    d_seat = (gpio_get_level(D_SEAT) == 0);
    p_seat = (gpio_get_level(P_SEAT) == 0);
    d_belt = (gpio_get_level(D_SEATBELT) == 0);
    p_belt = (gpio_get_level(P_SEATBELT) == 0);
    ignit  = (gpio_get_level(IGNITION) == 0);

    // Welcome message (once)
    if (d_seat && welcome_not_shown) {
        printf("Welcome to enhanced alarm system model 218-W26!\n");
        welcome_not_shown = false;
    }

    // Compute readiness once
    ignition_enabled = d_seat && p_seat && d_belt && p_belt;

    // LED logic (simple)
    gpio_set_level(IGNITION_LED, (!engine_running && ignition_enabled) ? 1 : 0);

    // Detect NEW ignition press (edge)
    bool ign_pressed = (ignit && !last_ignit);

    // ---- STAGE: ENGINE OFF ----
    if (!engine_running)
    {
        if (ign_pressed)
        {
            if (ignition_enabled)
            {
                engine_running = true;
                gpio_set_level(ENGINE_LED, 1);
                gpio_set_level(IGNITION_LED, 0);
                gpio_set_level(BUZZER, 0);
                printf("Engine started.\n");
            }
            else
            {
                printf("Ignition inhibited.\n");
                gpio_set_level(BUZZER, 1);

                if (!d_seat) printf("Driver seat not occupied.\n");
                if (!p_seat) printf("Passenger seat not occupied.\n");
                if (!d_belt) printf("Driver's seatbelt is not fastened.\n");
                if (!p_belt) printf("Passenger's seatbelt not fastened.\n");

                // simple “beep” instead of stuck buzzer
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(BUZZER, 0);
            }
        }
    }
    // STAGE: ENGINE ON
    else
    {
        if (ign_pressed)
        {
            engine_running = false;
            gpio_set_level(ENGINE_LED, 0);
            gpio_set_level(BUZZER, 0);
            printf("car stopped\n");
        }
    }

    last_ignit = ignit;

    vTaskDelay(25 / portTICK_PERIOD_MS);
}