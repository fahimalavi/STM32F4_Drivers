/*
 * main.c
 * Description: Use of UART(tx/rx), GPIO(input/out) and Systick drivers in code.
 *
 *  Created on: 31 Dec 2021
 *      Author: fahim
 *
 * Copyright (c) 2021-2022.
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#include <stdio.h>
#include "stm32f4xx.h"
#include "gpio.h"
#include "uart.h"
#include <sys_tick.h>

#define BAUD_RATE 115200
// Uncomment UART2_RX_INTERRUPT_ENABLE : Enable UART2 RX interrupt
//#define UART2_RX_INTERRUPT_ENABLE

// Uncomment TIMESTAMP_ENABLE : Enable UART2 RX interrupt
//#define SYSTICK_TIMESTAMP_ENABLE

static void gpio_function_handler();

#if defined(UART2_RX_INTERRUPT_ENABLE)
uint8_t uart_data = 0;
#endif //UART2_RX_INTERRUPT_ENABLE

int main(void)
{
#if defined(UART2_RX_INTERRUPT_ENABLE)
  // RX Interrupt Enable
  uart2_config_init(BAUD_RATE, true);
#else //
  uart2_config_init(BAUD_RATE, false);
#endif //UART2_RX_INTERRUPT_ENABLE

  printf("Welcome to baremetal program, press Enter to start\r\n");

  // Wait for Enter
#if defined(UART2_RX_INTERRUPT_ENABLE)
  while(uart_data != 0x0A) {}
  printf("Enter pressed, read from interrupt waiting 2 seconds\r\n");
#else //
  while(uart2_read() != 0x0A) {}
  printf("Enter pressed, waiting 2 seconds\r\n");
#endif //UART2_RX_INTERRUPT_ENABLE

  systick_delay_ms(2000);
  printf("2 seconds over, press switch button to toggle user LED\r\n");

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
  gpio_enable_irq(&gpio_config, RISING_EDGE, EXTI15_10_IRQn);
#if defined(SYSTICK_TIMESTAMP_ENABLE)
  systick_start();
#endif

  /* Loop forever */
  for(;;)
  {
//    gpio_write(PORT_A, 5, GPIO_VALUE_HIGH);
//    while(gpio_read(PORT_C, 13) == GPIO_VALUE_LOW)
//      gpio_write(PORT_A, 5, GPIO_VALUE_LOW);
  }
}

void EXTI15_10_IRQHandler()
{
  // selected trigger request occurred on PC13
  if(EXTI->PR & (1u<<13))
  {
    //This bit is cleared by programming it to ‘1’
    EXTI->PR |= (1u<<13);
#if defined(SYSTICK_TIMESTAMP_ENABLE)
    printf("%d: Button pressed, toggling LED\r\n", get_time_lapsed());
#else
    printf("Button pressed, toggling LED\r\n");
#endif
    //Toggle LED state
    gpio_toggle(PORT_A, 5);
  }
}

#if defined(UART2_RX_INTERRUPT_ENABLE)
void USART2_IRQHandler()
{
  if(USART2->SR & (1u<<5))
  {
    uart_data = (USART2->DR & 0xFF);
  }
}
#endif //UART2_RX_INTERRUPT_ENABLE
