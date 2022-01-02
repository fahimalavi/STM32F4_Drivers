/*
 * sys_tick.h
 *
 *  Created on: 1 Jan 2022
 *      Author: fahim
 *
 * Copyright (c) 2021-2022.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef INC_SYS_TICK_H_
#define INC_SYS_TICK_H_

extern uint32_t time_logged_ms;

/*
 * @Description  Delay in ms. Will stop systick and may impact get_time_lapsed()
 * @param  uint32_t time to wait in ms
 * @return  void
 */
void systick_delay_ms(uint32_t ms);

/*
 * @Description  Start SYSTICK continuously and get time lapsed from calling get_time_lapsed
 * @param  void
 * @return  void
 */
void systick_start();

/*
 * @Description  Get time lapsed since main hit
 * @return  eGPIO_Value
 */
uint32_t get_time_lapsed();

#endif /* INC_SYS_TICK_H_ */
