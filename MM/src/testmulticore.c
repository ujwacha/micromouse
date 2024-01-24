#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/platform.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/uart.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "pico/types.h"

#define UART_ID uart1
#define BAUD_RATE 9600

#define LED_PIN 25

#define UART_TX_PIN 4
#define UART_RX_PIN 5

uint8_t class[7] =  {
  0b00011000,
  0b11111111,
  0b00011111,
  0b11111000,
  0b00000000,
  0b01100110,
};





enum cla {
  MID,
  FULL,
  RIG,
  LFT,
  NUL,
  TT,
  JPT,
};




int state_led = 0;

void state_toggle() {
  if (state_led == 0) {
    state_led = 1;
  } else {
    state_led = 0;
  }
}

int cntbits(int n) {
    int count = 0;
    
    while (n) {
        n = n & (n - 1);
        count++;
    }
    
    return count;
}


float weight(uint8_t inp) {
  float retval = 0;

  for (int i = 0; i < 8; i++) {
    int thing = ((inp & (1 << i)) >> i);

    retval += thing * (3.5 - i);

  }

  return retval;
}



enum cla get_thing(uint8_t inp) {


  if(cntbits(inp ^ class[NUL]) <= 1) {
    return NUL;
  }

  if(cntbits(inp ^ class[FULL]) <= 1) {
    return FULL;
  }




  if(cntbits(inp ^ class[RIG]) <= 2) {
    return RIG;
  }

  if(cntbits(inp ^ class[LFT]) <= 2) {
    return LFT;
  }



  if(cntbits(inp ^ class[MID]) <= 1) {
    return MID;
  }



  if(cntbits(inp ^ class[TT]) <= 1) {
    return TT;
  }

  return JPT;

  
}





void core1_entry_uart_reader() {

  uart_init(UART_ID, 9600);

  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);

  uart_set_baudrate(UART_ID, 9600);

  
  bi_decl(bi_1pin_with_func(UART_TX_PIN, GPIO_FUNC_UART));
  bi_decl(bi_1pin_with_func(UART_RX_PIN, GPIO_FUNC_UART));


  uint8_t data;

  absolute_time_t prevtime;
  absolute_time_t curtime;

  prevtime = get_absolute_time();

  while (1) {

    curtime = get_absolute_time();

    
    if (to_ms_since_boot(curtime) - to_ms_since_boot(prevtime) > 500) {

      uart_read_blocking(UART_ID, &data, 1);

      multicore_fifo_push_blocking((uint32_t) data);

      prevtime = curtime;
      
    }

    sleep_ms(10);

  }


}


float pos(float num) {


  if (num < 0) {
    return (-1 * num);
  } else {
    return num;
  }


}


void core1_interrupt_handler() {
  while (multicore_fifo_rvalid()) {
    uint32_t raw = multicore_fifo_pop_blocking();

    uint8_t data = (uint8_t) raw;

    state_toggle();

    gpio_put(LED_PIN, state_led);


    if (1) {

      switch (get_thing(data)){
      case FULL:
	printf("FULL\n");
	break;
      
      case MID:
	printf("MID\n");
	break;
	
      case LFT:
	printf("LFT\n");
	break;
	
      case RIG:
	printf("RIG\n");
	break;
	
      case NUL:
	printf("NUL\n");
	break;
	
      case TT:
	printf("TT\n");
	break;
	
      case JPT:
	printf("JPT\n");
	break;
      
      }
    }
    
    

  }
}


int main() {

  stdio_init_all();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put(LED_PIN, state_led);
  
  multicore_launch_core1(core1_entry_uart_reader);

 
  multicore_fifo_clear_irq();

  irq_set_exclusive_handler(SIO_IRQ_PROC0, core1_interrupt_handler);

  irq_set_enabled(SIO_IRQ_PROC0, true);


  while (1) {
    tight_loop_contents();
  }
  
  
  return 0;
}
