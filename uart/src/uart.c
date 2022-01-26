/*
 * uart.c
 *
 *  Created on: 31 Dec 2021
 *      Author: fahim
 *
 * Copyright (c) 2021-2022.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include "gpio.h"
#include "stm32f4xx.h"
#include "uart.h"


void uart2_config_init(uint32_t baudrate, bool enable_interrupt)
{
  tGPIO_Config gpio_config;

  // Enable clock to GPIO port A
  RCC->AHB1ENR |= 1u;

  // Set PA2 to UART2 TX
  gpio_config.gpio_pin = 2;
  gpio_config.port = PORT_A;

  gpio_config.gpio_mode = ALTERNATE_MODE;
  gpio_config.config = CONFIG_GPIO_MODE_PRESENT;

  // Set alternate functionality on PA2 to AF7=UART2 see datasheet
  gpio_config.alternate_func = AF7;

  gpio_init(&gpio_config);

  // Set PA3 to UART2 RX
  gpio_config.gpio_pin = 3;
  gpio_config.port = PORT_A;

  gpio_config.gpio_mode = ALTERNATE_MODE;
  gpio_config.config = CONFIG_GPIO_MODE_PRESENT;

  // Set alternate functionality on PA3 to AF7=UART2 see datasheet
  gpio_config.alternate_func = AF7;

  gpio_init(&gpio_config);

  // Enable clock to USART2
  RCC->APB1ENR |= (1u<<17);

  set_baudrate(USART2, APB1_CLK, baudrate);

  // Enable TX
  USART2->CR1 = (1u<<3);

  // Enable RX
  USART2->CR1 |= (1u<<2);

  // rx interrupt Enable
  if(enable_interrupt)
  {
    USART2->CR1 |= (1u<<5);
    USART2->CR1 |= (1u<<6); // Transmission complete interrupt enable
    NVIC_EnableIRQ(USART2_IRQn);
  }

  // Enable UART
  USART2->CR1 |= (1u<<13);
}

uint16_t compute_uart_div(uint32_t periphera_clk, uint32_t baudrate)
{
  return ((periphera_clk + (baudrate/2u))/baudrate);
}

void set_baudrate(USART_TypeDef *USARTx, uint32_t peripheral_clk, uint32_t baudrate)
{
  USARTx->BRR = compute_uart_div(peripheral_clk, baudrate);
}

void uart2_write(int ch)
{
  while(!(USART2->SR & (1u<<7)))
  {
    // Wait
  }
  USART2->DR = ch & 0xFF;
}

uint8_t uart2_read()
{
  while(!(USART2->SR & (1u<<5)))
  {
    // Wait
  }
  return (USART2->DR & 0xFF);
}

int __io_putchar(int ch)
{
  uart2_write(ch);
  return ch;
}

// not working
int __io_getchar()
{
  return (int)uart2_read();
}


