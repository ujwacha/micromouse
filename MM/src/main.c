#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/uart.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
//#include "nodedetect.c"
#include "PID.h"
#include "driver.h"
#include "differencial.h"

#define UART_ID uart1
#define BAUD_RATE 9600

#define LED_PIN 25

#define UART_TX_PIN 4
#define UART_RX_PIN 5

/* Controller parameters */
#define PID_KP  1.0f
#define PID_KI  0.0f
#define PID_KD  0.0f

#define PID_TAU 0.02f

#define PID_LIM_MIN -255.0f
#define PID_LIM_MAX  255.0f

#define PID_LIM_MIN_INT -150.0f
#define PID_LIM_MAX_INT  150.0f

#define SAMPLE_TIME_S 0.05f

#define S1 19
#define S2 18



PIDController pid;

int state = 1;

void toggle_led_state() {
  if (state) {
    state = 0;
  } else {
    state = 1;
  }
}


float weight(uint8_t inp) {
  float retval = 0;

  for (int i = 0; i < 8; i++) {
    int thing = ((inp & (1 << i)) >> i);

    retval += thing * (3.5 - i);

  }

  return retval;
}

void init_switches() {
  gpio_init(S1);
  gpio_set_dir(S1, GPIO_IN);

  gpio_init(S2);
  gpio_set_dir(S2, GPIO_IN);
}

void led_pin_init() {
  
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put(LED_PIN, state);

 
}

void uart_work() {
    // uart
  
  uart_init(UART_ID, 9600);

  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);

  uart_set_baudrate(UART_ID, 9600);

  
  bi_decl(bi_1pin_with_func(UART_TX_PIN, GPIO_FUNC_UART));
  bi_decl(bi_1pin_with_func(UART_RX_PIN, GPIO_FUNC_UART));



}


int main() {

  stdio_init_all();
  uart_work();
  driver_init();

  init_switches();
  

  /// set up pid

  
  /* Initialise PID controller */
  PIDController pid = { PID_KP, PID_KI, PID_KD,
    PID_TAU,
    PID_LIM_MIN, PID_LIM_MAX,
    PID_LIM_MIN_INT, PID_LIM_MAX_INT,
    SAMPLE_TIME_S };
  
  PIDController_Init(&pid);

  float setpoint = 0.0f;


  uint8_t data;
  
  float cum_data;

  float out;

  struct diff_motors motors;

  while (1) {
    if (gpio_get(S1) == gpio_get(S2)) {
      break;
    }

    sleep_ms(100);

  }

  led_pin_init();

  while (1) {


    uart_read_blocking(UART_ID, &data, 1);

    toggle_led_state();
    
    gpio_put(LED_PIN, state);

    cum_data = weight(data);

    
    out = PIDController_Update(&pid, setpoint, cum_data);

    motors = get_diff(out);

    //run_motor_one(motors.a);
    //run_motor_two(motors.b);

    printf("%f, %f, %f, %f\n", motors.a, motors.b, out, cum_data);

    sleep_ms(50);
  }



  return 0;
}
