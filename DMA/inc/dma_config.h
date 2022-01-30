/*
 * dma_config.h
 *
 *  Created on: 26 Jan 2022
 *      Author: fahim
 */

#ifndef INC_DMA_CONFIG_H_
#define INC_DMA_CONFIG_H_

typedef uint8_t DMA_UART_Handle_t;

  /*
  The following sequence should be followed to configure a DMA stream x (where x is the
  stream number):
  1. If the stream is enabled, disable it by resetting the EN bit in the DMA_SxCR register,
  then read this bit in order to confirm that there is no ongoing stream operation. Writing
  this bit to 0 is not immediately effective since it is actually written to 0 once all the
  current transfers have finished. When the EN bit is read as 0, this means that the
  stream is ready to be configured. It is therefore necessary to wait for the EN bit to be
  cleared before starting any stream configuration. All the stream dedicated bits set in the
  status register (DMA_LISR and DMA_HISR) from the previous data block DMA
  transfer should be cleared before the stream can be re-enabled.
  2. Set the peripheral port register address in the DMA_SxPAR register. The data will be
  moved from/ to this address to/ from the peripheral port after the peripheral event.
  3. Set the memory address in the DMA_SxMA0R register (and in the DMA_SxMA1R
  register in the case of a double buffer mode). The data will be written to or read from
  this memory after the peripheral event.
  4. Configure the total number of data items to be transferred in the DMA_SxNDTR
  register. After each peripheral event or each beat of the burst, this value is
  decremented.
  5. Select the DMA channel (request) using CHSEL[2:0] in the DMA_SxCR register.
  6. If the peripheral is intended to be the flow controller and if it supports this feature, set
  the PFCTRL bit in the DMA_SxCR register.
  7. Configure the stream priority using the PL[1:0] bits in the DMA_SxCR register.
  8. Configure the FIFO usage (enable or disable, threshold in transmission and reception)
  9. Configure the data transfer direction, peripheral and memory incremented/fixed mode,
  single or burst transactions, peripheral and memory data widths, Circular mode,
  Double buffer mode and interrupts after half and/or full transfer, and/or errors in the
  DMA_SxCR register.
  10. Activate the stream by setting the EN bit in the DMA_SxCR register.
  */
bool dma_uart2_init(DMA_UART_Handle_t *handle);

/*
 * Function name : dma_uart2_send
 * Description : Non-blocking call to send data on UART
 * Arguments : DMA_UART_Handle_t * , uint8_t* address of data to sent and uint16_t size of data to sent
 * Return : TRUE if successful else FALSE
 */
bool dma_uart2_send(DMA_UART_Handle_t *handle, uint8_t *mem_addr, uint16_t size);

/*
 * Function name : dma_uart2_receive
 * Description : Non-blocking call to receive data on UART
 * Arguments : DMA_UART_Handle_t * , uint8_t* Memory Address and uint16_t size of buffer
 * Return : TRUE if successful else FALSE
 */
bool dma_uart2_receive(DMA_UART_Handle_t *handle, uint8_t *mem_addr, uint16_t size);

#endif /* INC_DMA_CONFIG_H_ */
