/*
 * utility.h
 *
 *  Created on: 30 Jan 2022
 *      Author: fahim
 */

#ifndef INC_UTILITY_H_
#define INC_UTILITY_H_

// Uncomment UART2_INTERRUPT_ENABLE : Enable UART2 RX interrupt
//#define UART2_INTERRUPT_ENABLE

// Uncomment SYSTICK_TIMESTAMP_ENABLE : Enable UART2 RX interrupt
//#define SYSTICK_TIMESTAMP_ENABLE

// Uncomment CONFIGURE_DMA_UART2 : Enable DMA on UART2
#define CONFIGURE_DMA_UART2

/*
 * Function name : uart_init_util
 * Description : UART2 initialization depending on interrupt or polling based
 * Arguments : none
 * Return : none
 */
void uart_init_util();

/*
 * Function name : program_sequence
 * Description : Basic UART tx/rx, systick and GPIO (input & output) examples
 * Arguments : none
 * Return : none
 */
void program_sequence();

/*
 * Function name : data_communication_DMA
 * Description : DMA UART TX/RX example with systick and GPIO.
 * Arguments : none
 * Return : none
 */
void data_communication_DMA();

#endif /* INC_UTILITY_H_ */
