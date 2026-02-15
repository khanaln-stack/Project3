#include <stdio.h>

#include "freertos/FreeRTOS.h"  //Use vTaskDelay()
#include "freertos/task.h"

#include "driver/gpio.h"    //read buttons and turn LED/buzzer on/off
#include "driver/ledc.h"    // make PWN signalsfor the servo

#include "esp_adc/adc_oneshot.h"    //read knob values
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include <hd44780.h>    // write words on LCD

// PINS

// Ignition inputs
#define D_SEAT      GPIO_NUM_4
#define P_SEAT      GPIO_NUM_5
#define D_SEATBELT  GPIO_NUM_6
#define P_SEATBELT  GPIO_NUM_7
#define IGNITION    GPIO_NUM_12

// Outputs
#define IGNITION_LED GPIO_NUM_9   // yellow
#define ENGINE_LED   GPIO_NUM_10  // blue
#define BUZZER       GPIO_NUM_11

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

#define LEDC_DUTY_MIN  205
#define LEDC_DUTY_MAX  1024

// ADC
#define ADC_ATTEN   ADC_ATTEN_DB_12
#define BITWIDTH    ADC_BITWIDTH_12

#define MODE_ADC_CH   ADC_CHANNEL_2   // GPIO3
#define DELAY_ADC_CH  ADC_CHANNEL_7   // GPIO8

// Loop timing
#define LOOP_MS 10

// Wiper speeds
#define HI_DEG_PER_SEC 150.0f
#define LO_DEG_PER_SEC 60.0f

// GLOBALS

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t cali_mode_handle;
adc_cali_handle_t cali_delay_handle;

hd44780_t lcd;

//INIT FUNCTIONS 

void gpio_init_all(void)
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

    gpio_reset_pin(IGNITION_LED);
    gpio_set_direction(IGNITION_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(IGNITION_LED, 0);

    gpio_reset_pin(ENGINE_LED);
    gpio_set_direction(ENGINE_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(ENGINE_LED, 0);
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

    float frac = angle / 180.0f;

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

// MAIN

void app_main(void)
{
    gpio_init_all();
    lcd_init_simple();
    servo_init();
    adc_init_simple();

    // start blank
    hd44780_clear(&lcd);

    // park wiper
    servo_set_angle(0);

    // ignition variables
    int d_seat, p_seat;
    int d_belt, p_belt;
    int ignit;

    int ignition_enabled = 0;
    int engine_running = 0;

    int last_ignit = 0;
    int last_d_seat = 0;

    // LCD control
    int lcd_lock_ms = 0;

    // lcd_state:
    // 0 = BLANK
    // 1 = WELCOME (hold until ignition press)
    // 2 = ENGINE_MSG (temporary)
    // 3 = INHIBIT_MSG (temporary)
    // 4 = WIPER
    int lcd_state = 0;

    int welcome_already_shown = 0;

    // inhibit sequence
    int inhibit_active = 0;
    int inhibit_step = 0;
    int inhibit_timer_ms = 0;

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
        // READ INPUTS (active-low)
        d_seat = (gpio_get_level(D_SEAT) == 0);
        p_seat = (gpio_get_level(P_SEAT) == 0);
        d_belt = (gpio_get_level(D_SEATBELT) == 0);
        p_belt = (gpio_get_level(P_SEATBELT) == 0);
        ignit  = (gpio_get_level(IGNITION) == 0);

        // WELCOME (SHOW ONLY ONCE)
        // LCD starts blank. First time driver sits down, show welcome and hold it until ignition press.
        if (!welcome_already_shown)
        {
            if (d_seat && !last_d_seat)
            {
                hd44780_clear(&lcd);
                hd44780_gotoxy(&lcd, 0, 0);
                hd44780_puts(&lcd, "Welcome to enhanced");
                hd44780_gotoxy(&lcd, 0, 1);
                hd44780_puts(&lcd, "alarm sys 218-W26");

                lcd_state = 1;
                welcome_already_shown = 1;
            }
        }
        last_d_seat = d_seat;

        // If driver not seated, engine off, and not showing inhibit show blank
        if (!engine_running && !d_seat && lcd_state != 3)
        {
            if (lcd_state != 0)
            {
                hd44780_clear(&lcd);
                lcd_state = 0;
            }
        }

        //IGNITION ENABLE + LEDs
        ignition_enabled = (d_seat && p_seat && d_belt && p_belt);

        if (!engine_running && ignition_enabled)
        {
            gpio_set_level(IGNITION_LED, 1);
        }
        else
        {
            gpio_set_level(IGNITION_LED, 0);
        }

        if (engine_running)
        {
            gpio_set_level(ENGINE_LED, 1);
        }
        else
        {
            gpio_set_level(ENGINE_LED, 0);
        }

        //IGNITION EDGE DETECT
        int ign_pressed = 0;
        if (ignit && !last_ignit)
        {
            ign_pressed = 1;
        }
        last_ignit = ignit;

        //ENGINE START/STOP
        if (!engine_running)
        {
            if (ign_pressed)
            {
                if (ignition_enabled)
                {
                    engine_running = 1;

                    hd44780_clear(&lcd);
                    hd44780_gotoxy(&lcd, 0, 0);
                    hd44780_puts(&lcd, "Engine started.");

                    lcd_state = 2;
                    lcd_lock_ms = 1200;

                    last_mode_lcd = -1;
                    last_delay_lcd = -1;

                    inhibit_active = 0;
                    gpio_set_level(BUZZER, 0);
                }
                else
                {
                    gpio_set_level(BUZZER, 1);

                    hd44780_clear(&lcd);
                    hd44780_gotoxy(&lcd, 0, 0);
                    hd44780_puts(&lcd, "Ignition inhibited");

                    lcd_state = 3;
                    inhibit_active = 1;
                    inhibit_step = 0;
                    inhibit_timer_ms = 0;

                    lcd_lock_ms = 4000;
                }
            }
        }
        else
        {
            if (ign_pressed)
            {
                engine_running = 0;

                hd44780_clear(&lcd);
                lcd_state = 0;
            }
        }

        //INHIBIT REASONS
        if (inhibit_active)
        {
            inhibit_timer_ms = inhibit_timer_ms - LOOP_MS;

            if (inhibit_timer_ms <= 0)
            {
                hd44780_gotoxy(&lcd, 0, 1);
                hd44780_puts(&lcd, "                ");
                hd44780_gotoxy(&lcd, 0, 1);

                int shown = 0;
                int tries = 0;

                while (!shown && tries < 4)
                {
                    if (inhibit_step == 0)
                    {
                        if (!p_seat) { hd44780_puts(&lcd, "Pass seat empty"); shown = 1; }
                    }
                    else if (inhibit_step == 1)
                    {
                        if (!d_seat) { hd44780_puts(&lcd, "Driver not seated"); shown = 1; }
                    }
                    else if (inhibit_step == 2)
                    {
                        if (!d_belt) { hd44780_puts(&lcd, "Driver belt off"); shown = 1; }
                    }
                    else
                    {
                        if (!p_belt) { hd44780_puts(&lcd, "Pass belt off"); shown = 1; }
                    }

                    inhibit_step = inhibit_step + 1;
                    if (inhibit_step > 3) inhibit_step = 0;

                    tries = tries + 1;
                }

                inhibit_timer_ms = 1000;
            }

            if (lcd_lock_ms == 0)
            {
                inhibit_active = 0;
                gpio_set_level(BUZZER, 0);

                // After showing reasons, allow another attempt.
                // Keep blank (specifically: welcome should not show again).
                hd44780_clear(&lcd);
                lcd_state = 0;
            }
        }

        //ADC READ (wiper knobs)
        int mode_bits = 0;
        int delay_bits = 0;

        adc_oneshot_read(adc1_handle, MODE_ADC_CH, &mode_bits);
        adc_oneshot_read(adc1_handle, DELAY_ADC_CH, &delay_bits);

        if (mode_bits < 1024) wiper_mode = 0;        // OFF
        else if (mode_bits < 2048) wiper_mode = 1;   // LO
        else if (mode_bits < 3072) wiper_mode = 3;   // INT
        else wiper_mode = 2;                         // HI

        if (delay_bits < 1365) delay_ms = 1000;
        else if (delay_bits < 2730) delay_ms = 3000;
        else delay_ms = 5000;

        //LCD SEQUENCE: allow WIPER after engine msg 
        if (engine_running && lcd_state == 2 && lcd_lock_ms == 0)
        {
            lcd_state = 4;
            last_mode_lcd = -1;
            last_delay_lcd = -1;
        }

        //LCD WIPER DISPLAY
        if (engine_running && lcd_state == 4)
        {
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
        }

        //WIPER MOTION LOGIC
        int run_request = 0;
        if (engine_running && (wiper_mode != 0))
        {
            run_request = 1;
        }

        float deg_per_sec;
        if (wiper_mode == 2) deg_per_sec = HI_DEG_PER_SEC;
        else deg_per_sec = LO_DEG_PER_SEC;

        float step = deg_per_sec * ((float)LOOP_MS / 1000.0f);

        if (w_state == 0) // PARKED
        {
            w_angle = 0.0f;
            servo_set_angle(w_angle);

            if (run_request)
            {
                w_state = 1;
            }
        }
        else if (w_state == 1) // UP
        {
            w_angle = w_angle + step;

            if (w_angle >= 90.0f)
            {
                w_angle = 90.0f;
                w_state = 2;
            }

            servo_set_angle(w_angle);
        }
        else if (w_state == 2) // DOWN
        {
            w_angle = w_angle - step;

            if (w_angle <= 0.0f)
            {
                w_angle = 0.0f;
                servo_set_angle(w_angle);

                if (!run_request)
                {
                    w_state = 0; // STOP parked
                }
                else
                {
                    if (wiper_mode == 3)
                    {
                        int_wait_elapsed = 0;
                        w_state = 3; // INT_WAIT
                    }
                    else
                    {
                        w_state = 1; // continue
                    }
                }
            }
            else
            {
                servo_set_angle(w_angle);
            }
        }
        else // INT_WAIT
        {
            if (run_request)
            {
                int_wait_elapsed = int_wait_elapsed + LOOP_MS;

                if (int_wait_elapsed >= delay_ms)
                {
                    w_state = 1;
                }
            }
        }

        // LCD LOCK COUNTDOWN
        if (lcd_lock_ms > 0)
        {
            lcd_lock_ms = lcd_lock_ms - LOOP_MS;
            if (lcd_lock_ms < 0) lcd_lock_ms = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}
