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
#include "utility.h"

int main(void)
{
  uart_init_util();

#if defined (CONFIGURE_DMA_UART2)
  data_communication_DMA();
#else
  program_sequence();
#endif
}
