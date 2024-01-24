#define ENA 11
#define IN1 12 
#define IN2 13 
#define IN3 10
#define IN4 9
#define ENB 8

#include "hardware/pwm.h"
#include "pico/stdlib.h"

uint enb1_slice; 
uint enb2_slice;



void driver_init() {
    gpio_init(IN1);
    gpio_init(IN2);
    gpio_init(IN3);
    gpio_init(IN4);
    gpio_init(ENA);
    gpio_init(ENB);

    gpio_set_dir(IN1,GPIO_OUT);
    gpio_set_dir(IN2,GPIO_OUT);
    gpio_set_dir(IN3,GPIO_OUT);
    gpio_set_dir(IN4,GPIO_OUT);

    gpio_set_function(ENA,GPIO_FUNC_PWM);
    gpio_set_function(ENB,GPIO_FUNC_PWM);

    enb1_slice = pwm_gpio_to_slice_num(ENA);
    enb2_slice = pwm_gpio_to_slice_num(ENB);

    pwm_set_wrap(enb1_slice,255);
    pwm_set_wrap(enb2_slice,255);

    pwm_set_enabled(enb1_slice,true);
    pwm_set_enabled(enb2_slice,true);
}


void run_motor_one(int pwm) {


  if (pwm < 0) {
    pwm = -1 * pwm;

    gpio_put(IN3,0);
    gpio_put(IN4,1);
 
  } else {
    gpio_put(IN3,1);
    gpio_put(IN4,0);
    
  }


 pwm_set_chan_level(enb2_slice,PWM_CHAN_A,pwm);
}


void run_motor_two(int pwm) {


  if (pwm < 0) {
    pwm = -1 * pwm;

    gpio_put(IN1,1);
    gpio_put(IN2,0);

  } else {

    gpio_put(IN1,0);
    gpio_put(IN2,1);
  }


 pwm_set_chan_level(enb1_slice,PWM_CHAN_B,pwm);
}

