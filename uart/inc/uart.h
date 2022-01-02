/*
 * uart.h
 *
 *  Created on: 31 Dec 2021
 *      Author: fahim
 *
 * Copyright (c) 2021-2022.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#define SYS_FREQ  16000000
#define APB1_CLK  SYS_FREQ

void uart2_config_init(uint32_t baudrate, bool enable_rx_interrupt);
void set_baudrate(USART_TypeDef *USARTx, uint32_t peripheral_clk, uint32_t baudrate);
void uart2_write(int ch);
uint8_t uart2_read();

#endif /* INC_UART_H_ */
