/*
 * main.c
 * Description: Use of UART(tx/rx), GPIO(input/out) and Systick drivers in code.
 *
 *  Created on: 31 Dec 2021
 *      Author: fahim
 */
#include <stdio.h>
#include "stm32f4xx.h"
#include "gpio.h"
#include "uart.h"
#include <sys_tick.h>

#define BAUD_RATE 115200

static void gpio_function_handler();

int main(void)
{
  uart2_config_init(BAUD_RATE);

  printf("Welcome to baremetal program, press Enter to start\r\n");

  // Enter
  while(uart2_read() != 0x0A)
  {
    printf("Try again!\r\n");
  }
  printf("Enter pressed, waiting 5 seconds\r\n");
  systick_delay_ms(5000);
  printf("5 seconds over, press switch button to turn off user LED\r\n");

  gpio_function_handler();
}

static void gpio_function_handler()
{
  tGPIO_Config gpio_config;

  // Look in block diagram what Bus is connected to GPIO port
  // Enable clock to GPIO port A
  RCC->AHB1ENR |= 1u;
  // Enable clock to GPIO port C
  RCC->AHB1ENR |= (1u<<2);

  // Set PA5 to output : set bit 11 to 0 and 10 to 1
  gpio_config.gpio_pin = 5;
  gpio_config.port = PORT_A;

  gpio_config.gpio_mode = OUTPUT_MODE;
  gpio_config.config = CONFIG_GPIO_MODE_PRESENT;

  gpio_config.output_type = push_pull;
  gpio_config.config |= CONFIG_OUTPUT_TYPE_PRESENT;

  // Set PA5 config
  gpio_init(&gpio_config);

  // Set PC13 to input
  gpio_config.gpio_pin = 13;
  gpio_config.port = PORT_C;
  gpio_config.gpio_mode = INPUT_MODE;
  gpio_config.config = CONFIG_GPIO_MODE_PRESENT;
  // Set PC13 config
  gpio_init(&gpio_config);

  /* Loop forever */
  for(;;)
  {
    gpio_write(PORT_A, 5, GPIO_VALUE_HIGH);
    while(gpio_read(PORT_C, 13) == GPIO_VALUE_LOW)
      gpio_write(PORT_A, 5, GPIO_VALUE_LOW);
  }
}
