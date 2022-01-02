/*
 * sys_tick.c
 *
 *  Created on: 1 Jan 2022
 *      Author: fahim
 * Copyright (c) 2021-2022.
 * SPDX-License-Identifier: Apache-2.0
 */


#include "stm32f4xx.h"
#include <stdint.h>
#include <sys_tick.h>

// Processor clock divided by 1000 for a ms
#define SYSTICK_RELOAD_VALUE    16000

#define STK_CTRL_COUNT_FLAG (1u<<16)
#define STK_CTRL_CLK_SOURCE (1u<<2)
#define STK_CTRL_ENABLE     (1u)
#define STK_CTRL_TICK       (1u<<1) //Enable counting down to zero to asserts the SysTick exception request.

uint32_t time_logged_ms = 0;

void systick_delay_ms(uint32_t ms)
{
  uint32_t index;

  // Ensure 24 bit value
  SysTick->LOAD = SYSTICK_RELOAD_VALUE & 0xFFFFFF;

  // Set Clock to Processor clock
  SysTick->CTRL |= STK_CTRL_CLK_SOURCE;
  // Enable systick
  SysTick->CTRL |= STK_CTRL_ENABLE;

  for(index=0; index<ms; index++)
  {
    while(!(SysTick->CTRL &  STK_CTRL_COUNT_FLAG))
    {
    }
  }
  // Disable systick
  SysTick->CTRL &= ~STK_CTRL_ENABLE;
}

void systick_start()
{
  uint32_t index;

  // Ensure 24 bit value
  SysTick->LOAD = SYSTICK_RELOAD_VALUE & 0xFFFFFF;

  // Set Clock to Processor clock
  SysTick->CTRL |= STK_CTRL_CLK_SOURCE;
  // Enable systick
  SysTick->CTRL |= STK_CTRL_ENABLE;
  // Enable counting down to zero to asserts the SysTick exception request.
  SysTick->CTRL |= STK_CTRL_TICK;
}

void SysTick_Handler()
{
  if(SysTick->CTRL &  STK_CTRL_COUNT_FLAG)
  {
    time_logged_ms++;
  }
}

uint32_t get_time_lapsed()
{
  return time_logged_ms;
}


