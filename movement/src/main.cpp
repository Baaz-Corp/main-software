#include <Arduino.h>
#include <pwm.h>

uint32 period = 1000;
uint32 duty = 500;

uint32 io_info[3] = {PWM_0_OUT_IO_MUX, PWM_0_OUT_IO_FUNC, PWM_0_OUT_IO_NUM};

void setup() {
    pwm_init(period, &duty, 1, &io_info); // setting up PWM on GPIO 12
    pwm_start(); // starting PWM
}

void loop() {

}
