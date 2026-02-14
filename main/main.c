#include <stdio.h>

#include "freertos/FreeRTOS.h"  //Use vTaskDelay()
#include "freertos/task.h"

#include "driver/gpio.h"    //read buttons and turn LED/buzzer on/off
#include "driver/ledc.h"    // make PWN signalsfor the servo

#include "esp_adc/adc_oneshot.h"    //read knob values
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include <hd44780.h>    // write words on LCS


// Ignition Pins
#define D_SEAT GPIO_NUM_4
#define P_SEAT GPIO_NUM_5
#define D_SEATBELT GPIO_NUM_6
#define P_SEATBELT GPIO_NUM_7
#define IGNITION GPIO_NUM_12

#define IGNITION_LED GPIO_NUM_9
#define ENGINE_LED GPIO_NUM_10
#define BUZZER GPIO_NUM_11

// LCD Pins
#define LCD_RS GPIO_NUM_38
#define LCD_E  GPIO_NUM_37
#define LCD_D4 GPIO_NUM_36
#define LCD_D5 GPIO_NUM_35
#define LCD_D6 GPIO_NUM_48
#define LCD_D7 GPIO_NUM_47

// Servo PWM
#define SERVO_GPIO     GPIO_NUM_21

#define LEDC_TIMER     LEDC_TIMER_0
#define LEDC_MODE      LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL   LEDC_CHANNEL_0
#define LEDC_DUTY_RES  LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY 50

#define LEDC_DUTY_MIN  307
#define LEDC_DUTY_MAX  921

// ADC 
#define ADC_ATTEN   ADC_ATTEN_DB_12
#define BITWIDTH    ADC_BITWIDTH_12

#define MODE_ADC_CH   ADC_CHANNEL_2   // GPIO3
#define DELAY_ADC_CH  ADC_CHANNEL_7   // GPIO8

#define LOOP_MS 10

#define HI_DEG_PER_SEC 150.0f
#define LO_DEG_PER_SEC 60.0f

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t cali_mode_handle;
adc_cali_handle_t cali_delay_handle;

hd44780_t lcd;


// gpio_init_all()
// - Purpose: sets up ALL GPIO directions and pullups for ignition hardware.
// - No parameters: it uses the #defines above.
// - What it does:
//   - Inputs: seat, belts, ignition button as INPUT with pullups
//   - Outputs: buzzer + LEDs as OUTPUT, default OFF

void gpio_init_all(void){
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
}

void lcd_init_simple(void)
{
    lcd = (hd44780_t){
        .write_cb = NULL,
        .font = HD44780_FONT_5X8,
        .lines = 2,
        .pins = {
            .rs = LCD_RS,
            .e  = LCD_E,
            .d4 = LCD_D4,
            .d5 = LCD_D5,
            .d6 = LCD_D6,
            .d7 = LCD_D7,
            .bl = HD44780_NOT_USED
        }
    };

    hd44780_init(&lcd);
    hd44780_clear(&lcd);
}

void servo_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_GPIO,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel);
}

void servo_set_angle(float angle)
{
    if (angle < 0) angle = 0;
    if (angle > 90) angle = 90;

    float frac = angle / 90.0;
    int duty = LEDC_DUTY_MIN + (int)((LEDC_DUTY_MAX - LEDC_DUTY_MIN) * frac);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void adc_init_simple(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };
    adc_oneshot_config_channel(adc1_handle, MODE_ADC_CH, &config);
    adc_oneshot_config_channel(adc1_handle, DELAY_ADC_CH, &config);

    adc_cali_curve_fitting_config_t cal_mode = {
        .unit_id = ADC_UNIT_1,
        .chan = MODE_ADC_CH,
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };
    adc_cali_create_scheme_curve_fitting(&cal_mode, &cali_mode_handle);

    adc_cali_curve_fitting_config_t cal_delay = {
        .unit_id = ADC_UNIT_1,
        .chan = DELAY_ADC_CH,
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };
    adc_cali_create_scheme_curve_fitting(&cal_delay, &cali_delay_handle);
}

void app_main(void)
{
    gpio_init_all();
    lcd_init_simple();
    servo_init();
    adc_init_simple();

    servo_set_angle(0);

    // ignition variables (int only)
    int d_seat, p_seat;
    int d_belt, p_belt;

    int ignit = 0;
    int ignition_enabled = 0;
    int engine_running = 0;

    int last_ignit = 0;
    int welcome_not_shown = 1;

    // wiper variables
    int wiper_mode = 0;   // 0=OFF, 1=LO, 2=HI, 3=INT
    int delay_ms = 1000;

    int last_mode_lcd = -1;
    int last_delay_lcd = -1;

    int w_state = 0;      // 0=PARKED, 1=UP, 2=DOWN, 3=INT_WAIT
    float w_angle = 0.0f;
    int int_wait_elapsed = 0;

    
      while (1)
    {
        // Read ignition inputs (active-low)  
        d_seat = (gpio_get_level(D_SEAT) == 0);
        p_seat = (gpio_get_level(P_SEAT) == 0);
        d_belt = (gpio_get_level(D_SEATBELT) == 0);
        p_belt = (gpio_get_level(P_SEATBELT) == 0);
        ignit  = (gpio_get_level(IGNITION) == 0);

        //  Welcome
        if (d_seat && welcome_not_shown)
        {
            hd44780_clear(&lcd);
            hd44780_gotoxy(&lcd, 0, 0);
            hd44780_puts(&lcd, "Welcome to enhanced");
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, "alarm system 218-W26");
            welcome_not_shown = 0;
        }

        ignition_enabled = (d_seat && p_seat && d_belt && p_belt);

        // NO TERNARY (lab style)
        if (!engine_running && ignition_enabled)
        {
            gpio_set_level(IGNITION_LED, 1);
        }
        else
        {
            gpio_set_level(IGNITION_LED, 0);
        }

        // edge detect ignition press
        int ign_pressed = 0;
        if (ignit && !last_ignit)
        {
            ign_pressed = 1;
        }

        // ENGINE OFF 
        if (!engine_running)
        {
            if (ign_pressed)
            {
                if (ignition_enabled)
                {
                    engine_running = 1;
                    gpio_set_level(ENGINE_LED, 1);
                    gpio_set_level(IGNITION_LED, 0);

                    hd44780_clear(&lcd);
                    hd44780_gotoxy(&lcd, 0, 0);
                    hd44780_puts(&lcd, "Engine started.");
                }
                else
                {
                    gpio_set_level(BUZZER, 1);

                    hd44780_clear(&lcd);
                    hd44780_gotoxy(&lcd, 0, 0);
                    hd44780_puts(&lcd, "Ignition inhibited");

                    if (!d_seat) {
                        hd44780_gotoxy(&lcd, 0, 1);
                        hd44780_puts(&lcd, "Driver seat not occ");
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                    if (!p_seat) {
                        hd44780_gotoxy(&lcd, 0, 1);
                        hd44780_puts(&lcd, "Passenger not occ");
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                    if (!d_belt) {
                        hd44780_gotoxy(&lcd, 0, 1);
                        hd44780_puts(&lcd, "Driver belt not on");
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                    if (!p_belt) {
                        hd44780_gotoxy(&lcd, 0, 1);
                        hd44780_puts(&lcd, "Pass belt not on");
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }

                    gpio_set_level(BUZZER, 0);
                }
            }
        }
        else
        {
            if (ign_pressed)
            {
                engine_running = 0;
                gpio_set_level(ENGINE_LED, 0);
            }
        }

        last_ignit = ignit;

        // ADC 
        int mode_bits;
        int delay_bits;

        adc_oneshot_read(adc1_handle, MODE_ADC_CH, &mode_bits);
        adc_oneshot_read(adc1_handle, DELAY_ADC_CH, &delay_bits);

        if (mode_bits < 1024) wiper_mode = 0;
        else if (mode_bits < 2048) wiper_mode = 1;
        else if (mode_bits < 3072) wiper_mode = 3;
        else wiper_mode = 2;

        if (delay_bits < 1365) delay_ms = 1000;
        else if (delay_bits < 2730) delay_ms = 3000;
        else delay_ms = 5000;

        //LCD wiper display 
        if (wiper_mode != last_mode_lcd || delay_ms != last_delay_lcd)
        {
            hd44780_clear(&lcd);
            hd44780_gotoxy(&lcd, 0, 0);

            if (wiper_mode == 0) hd44780_puts(&lcd, "WIPER: OFF");
            else if (wiper_mode == 1) hd44780_puts(&lcd, "WIPER: LO");
            else if (wiper_mode == 2) hd44780_puts(&lcd, "WIPER: HI");
            else hd44780_puts(&lcd, "WIPER: INT");

            if (wiper_mode == 3)
            {
                hd44780_gotoxy(&lcd, 0, 1);
                if (delay_ms == 1000) hd44780_puts(&lcd, "DELAY: SHORT");
                else if (delay_ms == 3000) hd44780_puts(&lcd, "DELAY: MEDIUM");
                else hd44780_puts(&lcd, "DELAY: LONG");
            }

            last_mode_lcd = wiper_mode;
            last_delay_lcd = delay_ms;
        }

        // Wiper motion logic 
        int shutdown_requested = 0;
        if (!engine_running) shutdown_requested = 1;
        if (wiper_mode == 0) shutdown_requested = 1;

        float deg_per_sec;
        if (wiper_mode == 2) deg_per_sec = HI_DEG_PER_SEC;
        else deg_per_sec = LO_DEG_PER_SEC;   // LO and INT use LO speed

        float step = deg_per_sec * ((float)LOOP_MS / 1000.0f);

        if (w_state == 0)   // PARKED
        {
            w_angle = 0.0f;
            servo_set_angle(w_angle);

            if (!shutdown_requested)
            {
                w_state = 1;
            }
        }
        else if (w_state == 1)  // UP
        {
            w_angle = w_angle + step;

            if (w_angle >= 90.0f)
            {
                w_angle = 90.0f;
                w_state = 2;
            }

            servo_set_angle(w_angle);
        }
        else if (w_state == 2)  // DOWN
        {
            w_angle = w_angle - step;

            if (w_angle <= 0.0f)
            {
                w_angle = 0.0f;
                servo_set_angle(w_angle);

                if (shutdown_requested)
                {
                    w_state = 0;   // park
                }
                else
                {
                    if (wiper_mode == 3)
                    {
                        int_wait_elapsed = 0;
                        w_state = 3;   // INT_WAIT
                    }
                    else
                    {
                        w_state = 1;   // keep sweeping
                    }
                }
            }
            else
            {
                servo_set_angle(w_angle);
            }
        }
        else if (w_state == 3)  // INT_WAIT at 0
        {
            if (shutdown_requested)
            {

                w_state = 0;
            }
            else
            {
                int_wait_elapsed = int_wait_elapsed + LOOP_MS;

                if (int_wait_elapsed >= delay_ms)
                {
                    w_state = 1;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}