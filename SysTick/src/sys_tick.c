/*
 * sys_tick.c
 *
 *  Created on: 1 Jan 2022
 *      Author: fahim
 */


#include "stm32f4xx.h"
#include <stdint.h>
#include <sys_tick.h>

// Processor clock divided by 1000 for a ms
#define SYSTICK_RELOAD_VALUE    16000

#define STK_CTRL_COUNT_FLAG (1u<<16)
#define STK_CTRL_CLK_SOURCE (1u<<2)
#define STK_CTRL_ENABLE     (1u)

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


