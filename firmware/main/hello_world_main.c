#include <stdio.h>
#include <inttypes.h>
#include <nvs_flash.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "ps3.h"

void controller_event_cb( ps3_t ps3, ps3_event_t event );

// MOTORS

typedef struct {
    int speed_pin;
    int dir_pin;
    ledc_channel_t ledc_channel;
} motor_t;


void motor_set(motor_t *m, int v){
    if(v > 0){
        gpio_set_level(m->dir_pin, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, m->ledc_channel, v);
    } else {
        if(v == -128)v = -127;
        v = 127 +v;
        gpio_set_level(m->dir_pin, 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, m->ledc_channel, v);
    }
    printf("Update duty: %d\n", v);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, m->ledc_channel);
}

void motor_init(motor_t *m){
    // gpio_set_direction(m->speed_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(m->dir_pin, GPIO_MODE_OUTPUT);
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = m->ledc_channel,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = m->speed_pin,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    motor_set(m, 0);
}


motor_t left_motor = {17, 16, LEDC_CHANNEL_0};
motor_t right_motor = {19, 18, LEDC_CHANNEL_1};


// SERVO

int get_servo_duty(int angle){
    double duty_per_ms = 16384.0 / 20;
    double duty_in_ms = angle / 180.0;
    return duty_per_ms + duty_in_ms * duty_per_ms;
}

void init_servo(){
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_1,
        .duty_resolution  = LEDC_TIMER_14_BIT,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_2,
        .timer_sel      = LEDC_TIMER_1,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = 23,
        .duty           = get_servo_duty(30),
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void servo_write(int angle){
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, get_servo_duty(angle));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

// MAIN

void app_main(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_7_BIT,
        .freq_hz          = 1000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    motor_init(&left_motor);
    motor_init(&right_motor);
    init_servo();
   


    printf("Hello world!\n");


    nvs_flash_init();

    uint8_t new_mac[8] = {0x70,0x66,0x55,0x6c,0x3c,0xf3};
    ps3SetBluetoothMacAddress(new_mac);

    ps3SetEventCallback(controller_event_cb);
    ps3Init();

    while (!ps3IsConnected()){
        // Prevent the Task Watchdog from triggering
        vTaskDelay(300 / portTICK_PERIOD_MS);
        printf("Waiting\n");
    }

    printf("Connected\n");



}

void controller_event_cb( ps3_t ps3, ps3_event_t event )
{

    if(event.analog_changed.stick.ly){
        printf("Ly: %d\n", ps3.analog.stick.ly);
        motor_set(&left_motor, ps3.analog.stick.ly);
    }
    if(event.analog_changed.stick.ry){
        printf("Ry: %d\n", ps3.analog.stick.ry);
        motor_set(&right_motor, ps3.analog.stick.ry);
    }
    if(event.button_up.r2){
        servo_write(30);
    }
    if(event.button_down.r2){
        servo_write(180 - 30);
    }
}

