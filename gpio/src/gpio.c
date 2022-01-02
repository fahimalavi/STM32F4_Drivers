/*
 * gpio.c
 *
 *  Created on: 29 Dec 2021
 *      Author: fahim
 *
 * Copyright (c) 2021-2022.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include "stm32f4xx.h"
#include "gpio.h"

static GPIO_TypeDef* get_memory_location(eGPIO_PORT port)
{
  switch(port)
  {
  case  PORT_A:
    return GPIOA;
    break;
  case  PORT_B:
    return GPIOB;
    break;
  case  PORT_C:
    return GPIOC;
    break;
  case  PORT_D:
    return GPIOD;
    break;
  case  PORT_E:
    return GPIOE;
    break;
  case  PORT_H:
    return GPIOH;
    break;
  default:
    return NULL;
    break;
  }
}

void gpio_init(tGPIO_Config *gpio_config)
{
  GPIO_TypeDef *GPIOx = get_memory_location(gpio_config->port);

  if(gpio_config->gpio_pin < GPIO_NUMBER)
  {
    if(gpio_config->config & CONFIG_GPIO_MODE_PRESENT)
    {
      switch(gpio_config->gpio_mode)
      {
      case INPUT_MODE:
        GPIOx->MODER &= ~(1u<<(gpio_config->gpio_pin*2 + 1));
        GPIOx->MODER &= ~(1u<<(gpio_config->gpio_pin*2));
        break;
      case OUTPUT_MODE:
        GPIOx->MODER &= ~(1u<<(gpio_config->gpio_pin*2 + 1));
        GPIOx->MODER |= (1u<<(gpio_config->gpio_pin*2));
        break;
      case ALTERNATE_MODE:
      {
        uint8_t   index = ((gpio_config->gpio_pin < 8)?0:1);

        // First set Mode to alternate function
        GPIOx->MODER |= (1u<<(gpio_config->gpio_pin*2 + 1));
        GPIOx->MODER &= ~(1u<<(gpio_config->gpio_pin*2));

        // Then set alternate function
        switch(gpio_config->alternate_func)
        {
        case AF7:
          GPIOx->AFR[index] &= ~(1u<<(gpio_config->gpio_pin*4 + 3));
          GPIOx->AFR[index] |= (1u<<(gpio_config->gpio_pin*4 + 2));
          GPIOx->AFR[index] |= (1u<<(gpio_config->gpio_pin*4 + 1));
          GPIOx->AFR[index] |= (1u<<(gpio_config->gpio_pin*4));
          break;
        default:
          break;
        }
      }
      break;
      case ANALOG_MODE:
        break;
      default:
        break;
      }
    }
    if(gpio_config->config & CONFIG_OUTPUT_TYPE_PRESENT)
    {
      switch(gpio_config->output_type)
      {
      case push_pull:
        GPIOx->OTYPER &= ~(1u<<(gpio_config->gpio_pin));
        break;
      case open_drain:
        GPIOx->OTYPER |= (1u<<(gpio_config->gpio_pin));
        break;
      default:
        break;
      }
    }
    if(gpio_config->config & CONFIG_PUPD_CONFIG_PRESENT)
    {
      switch(gpio_config->pupd_config)
      {
      case pull_up:
        GPIOx->PUPDR &= ~(1u<<(gpio_config->gpio_pin*2 + 1));
        GPIOx->PUPDR |= (1u<<(gpio_config->gpio_pin*2));
        break;
      case pull_down:
        GPIOx->PUPDR |= (1u<<(gpio_config->gpio_pin*2 + 1));
        GPIOx->PUPDR &= ~(1u<<(gpio_config->gpio_pin*2));
        break;
      case reserved:
        // ERROR CASE
        break;
      case no_pupd:
      default:
        GPIOx->PUPDR &= ~(1u<<(gpio_config->gpio_pin*2 + 1));
        GPIOx->PUPDR &= ~(1u<<(gpio_config->gpio_pin*2));
        break;
      }
    }
  }
}

void gpio_enable_irq(tGPIO_Config *gpio_config, eGPIO_trigger_select trigger, int16_t interrupt_number)
{
  // Disable global IRQ
  __disable_irq();

  // Enable SYSCFG Clock
  RCC->APB2ENR |= (1u<<14);

  switch(gpio_config->port)
  {
  case PORT_A:
    // Set SYSCFG external interrupt configuration register PAx
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] &= ~(1u<<((gpio_config->gpio_pin & 0b11)*4+0));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] &= ~(1u<<((gpio_config->gpio_pin & 0b11)*4+1));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] &= ~(1u<<((gpio_config->gpio_pin & 0b11)*4+2));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] &= ~(1u<<((gpio_config->gpio_pin & 0b11)*4+3));
    break;
  case PORT_B:
    // Set SYSCFG external interrupt configuration register PBx
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] |= (1u<<((gpio_config->gpio_pin & 0b11)*4+0));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] &= ~(1u<<((gpio_config->gpio_pin & 0b11)*4+1));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] &= ~(1u<<((gpio_config->gpio_pin & 0b11)*4+2));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] &= ~(1u<<((gpio_config->gpio_pin & 0b11)*4+3));
    break;
  case PORT_C:
    // Set SYSCFG external interrupt configuration register PCx
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] &= ~(1u<<((gpio_config->gpio_pin & 0b11)*4+0));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] |= (1u<<((gpio_config->gpio_pin & 0b11)*4+1));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] &= ~(1u<<((gpio_config->gpio_pin & 0b11)*4+2));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] &= ~(1u<<((gpio_config->gpio_pin & 0b11)*4+3));
    break;
  case PORT_D:
    // Set SYSCFG external interrupt configuration register PDx
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] |= (1u<<((gpio_config->gpio_pin & 0b11)*4+0));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] |= (1u<<((gpio_config->gpio_pin & 0b11)*4+1));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] &= ~(1u<<((gpio_config->gpio_pin & 0b11)*4+2));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] &= ~(1u<<((gpio_config->gpio_pin & 0b11)*4+3));
    break;
  case PORT_E:
    // Set SYSCFG external interrupt configuration register PEx
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] &= ~(1u<<((gpio_config->gpio_pin & 0b11)*4+0));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] &= ~(1u<<((gpio_config->gpio_pin & 0b11)*4+1));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] |= (1u<<((gpio_config->gpio_pin & 0b11)*4+2));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] &= ~(1u<<((gpio_config->gpio_pin & 0b11)*4+3));
    break;
  case PORT_H:
    // Set SYSCFG external interrupt configuration register PHx
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] |= (1u<<((gpio_config->gpio_pin & 0b11)*4+0));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] |= (1u<<((gpio_config->gpio_pin & 0b11)*4+1));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] |= (1u<<((gpio_config->gpio_pin & 0b11)*4+2));
    SYSCFG->EXTICR[(gpio_config->gpio_pin >> 2)] &= ~(1u<<((gpio_config->gpio_pin & 0b11)*4+3));
    break;
  }

  // Interrupt request from line x is not masked
  EXTI->IMR |= (1u<<gpio_config->gpio_pin);

  switch(trigger)
  {
  case FALLING_EDGE:
    //Falling trigger enabled (for Event and Interrupt) for input line.
    EXTI->FTSR |= (1u<<gpio_config->gpio_pin);
    break;
  case RISING_EDGE:
    //Rising trigger enabled (for Event and Interrupt) for input line.
    EXTI->RTSR |= (1u<<gpio_config->gpio_pin);
    break;
  }

  // Argument is STM32 specific interrupt number
  NVIC_EnableIRQ(interrupt_number);

  // Enable Global interrupts -> Remember NVIC_EnableIRQ is different
  __enable_irq();
}

void gpio_write(eGPIO_PORT port, uint8_t gpio_pin, eGPIO_Value value)
{
  GPIO_TypeDef *GPIOx = get_memory_location(port);
  if(value == GPIO_VALUE_LOW)
  {
    GPIOx->BSRR |= (1u<<(gpio_pin+GPIO_NUMBER));
  }
  else
  {
    GPIOx->BSRR |= (1u<<gpio_pin);
  }
}

void gpio_toggle(eGPIO_PORT port, uint8_t gpio_pin)
{
  GPIO_TypeDef *GPIOx = get_memory_location(port);

  GPIOx->ODR ^= (1u<<(gpio_pin));
}

eGPIO_Value gpio_read(eGPIO_PORT port, uint8_t gpio_pin)
{
  GPIO_TypeDef *GPIOx = get_memory_location(port);

  if(GPIOx->IDR & (1u<<gpio_pin))
  {
    return GPIO_VALUE_HIGH;
  }
  else
  {
    return GPIO_VALUE_LOW;
  }
}
