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
#include "dma_config.h" // If you want to move to specific peripheral

#define BAUD_RATE 115200
#define TEST_DATA2_SIZE 11


char test_data2[TEST_DATA2_SIZE] = "0123456789";
char test_data3[TEST_DATA2_SIZE] = "ABCDEFGHIJ";
// Uncomment UART2_INTERRUPT_ENABLE : Enable UART2 RX interrupt
#define UART2_INTERRUPT_ENABLE

// Uncomment SYSTICK_TIMESTAMP_ENABLE : Enable UART2 RX interrupt
//#define SYSTICK_TIMESTAMP_ENABLE

// Uncomment CONFIGURE_DMA_UART2 : Enable DMA on UART2
#define CONFIGURE_DMA_UART2

#if defined(CONFIGURE_DMA_UART2)
#define UART2_INTERRUPT_ENABLE
#endif

void gpio_function_handler();
void program_sequence();

#if defined(CONFIGURE_DMA_UART2)
uint8_t wait_print = 0;
void data_communication_DMA();
#endif



#if defined(UART2_INTERRUPT_ENABLE)
uint8_t uart_data = 0;
#endif //UART2_INTERRUPT_ENABLE

int main(void)
{
#if defined(UART2_INTERRUPT_ENABLE)
  // RX Interrupt Enable
  uart2_config_init(BAUD_RATE, true);
#else //
  uart2_config_init(BAUD_RATE, false);
#endif //UART2_INTERRUPT_ENABLE

#if defined (CONFIGURE_DMA_UART2)
  data_communication_DMA();
#else
  program_sequence();
#endif
}

void program_sequence()
{
  printf("Welcome to baremetal program, press Enter to start\r\n");

  // Wait for Enter
#if defined(UART2_INTERRUPT_ENABLE)
  while(uart_data != 0x0A) {}
  printf("Enter pressed, read from interrupt waiting 2 seconds\r\n");
#else //
  while(uart2_read() != 0x0A) {}
  printf("Enter pressed, waiting 2 seconds\r\n");
#endif //UART2_INTERRUPT_ENABLE

  systick_delay_ms(2000);
  printf("2 seconds over, press switch button to toggle user LED\r\n");

  gpio_function_handler();
}

#if defined(CONFIGURE_DMA_UART2)
void data_communication_DMA()
{
  tGPIO_Config gpio_config;
  int i;
  printf("Data TX transferring after LED \r\n");

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

  for(i = 0; i < 10 ; i++)
  {
    systick_delay_ms(500);
    gpio_toggle(PORT_A, 5);
  }

  dma_uart2_init((uint32_t)&(USART2->DR), (uint32_t)test_data2, TEST_DATA2_SIZE, DMA1_Stream6_IRQn);

  while(wait_print == 0)
  {
    systick_delay_ms(1000);
    gpio_toggle(PORT_A, 5);
  }
  dma_uart2_init((uint32_t)&(USART2->DR), (uint32_t)test_data2, TEST_DATA2_SIZE, DMA1_Stream6_IRQn);
  for(;;)
  {

  }
}
#endif

void gpio_function_handler()
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

#if defined(UART2_INTERRUPT_ENABLE)
void USART2_IRQHandler()
{
  if(USART2->SR & (1u<<5))
  {
    //It is cleared by a read to the USART_DR register. The RXNE flag can also be cleared by writing a zero to it.
    uart_data = (USART2->DR & 0xFF);
  }
  if(USART2->SR & (1u<<6))
  {
    // The TC bit can also be cleared by writing a '0' to it.
    USART2->SR &= ~(1u<<6);
  }
}
#endif //UART2_INTERRUPT_ENABLE

void DMA1_Stream6_IRQHandler()
{
  //Stream x transfer complete interrupt flag
  if(DMA1->HISR & (1u<<21))
  {
#if defined(CONFIGURE_DMA_UART2)
    wait_print = 1;
#endif
    DMA1->HIFCR |= (1u<<21);
  }

  if (DMA1->HISR & 0x000C0000)    /* if an error occurred */
      wait_print = 2;
}
