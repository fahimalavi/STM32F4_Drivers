/*
 * dma_config.c
 *
 *  Created on: 26 Jan 2022
 *      Author: fahim
 */
#include <stdint.h>
#include <stdio.h>
#include "dma_config.h"
#include "stm32f4xx.h"

#define DMA1EN_AHB1ENR (1u<<21)
#define DMA_STREAMx_EN (1u)

#define DMA_STREAMx_DMEIE (1u<<1)
#define DMA_STREAMx_TEIE  (1u<<2)
#define DMA_STREAMx_HTIE  (1u<<3)
#define DMA_STREAMx_TCIE  (1u<<4)

#define DMA_STREAMx_MINC  (1u<<10) //Memory increment mode
#define DMA_STREAMx_DMDIS (1u<<2) //Direct mode disable DMA_SxFCR : This bit is protected and can be written only if EN is ‘0’. & not allowed in Mem 2 Mem

#define USART2_DMAT       (1u<<7) //DMA enable transmitter

void dma_uart2_init(uint32_t peripheral_data_register_addr, uint32_t mem_addr, uint16_t size, int32_t interrupt_number)
{
  printf("dma_uart2_init called\r\n");
  // Disable global IRQ
  //__disable_irq();

  //Enable clock access to DMA1
  RCC->AHB1ENR |= DMA1EN_AHB1ENR;

  // Disable stream by resetting the EN bit in the DMA_SxCR register (RM: DMA1 request mapping)
  DMA1_Stream6->CR &= ~DMA_STREAMx_EN;

  while((DMA1_Stream6->CR & DMA_STREAMx_EN) != 0)
  {

  }

  // Disable interrupt flags of Stream6 (UART_TX)
  DMA1->HIFCR |= (1u<<16);
  DMA1->HIFCR |= (1u<<18);
  DMA1->HIFCR |= (1u<<19);
  DMA1->HIFCR |= (1u<<20);
  DMA1->HIFCR |= (1u<<21);

  DMA1_Stream6->CR &= ~DMA_STREAMx_DMEIE;
  DMA1_Stream6->CR &= ~DMA_STREAMx_TEIE;
  DMA1_Stream6->CR &= ~DMA_STREAMx_HTIE;
  DMA1_Stream6->CR &= ~DMA_STREAMx_TCIE;

  // Set the peripheral port register address in the DMA_SxPAR register (Stream6)
  DMA1_Stream6->PAR = peripheral_data_register_addr;

  // Set the memory address in the DMA_SxMA0R register
  DMA1_Stream6->M0AR = mem_addr;

  // Configure the total number of data items to be transferred in the DMA_SxNDTR register.
  //    Number of data items to be transferred (0 up to 65535). This register can be written only
  //    When the stream is enabled, this register is read-only, indicating the remaining data items to be transmitted.
  DMA1_Stream6->NDTR = size;

  // Select the DMA channel in the DMA_SxCR register. (Select Channel 4, may be later generalize)
  DMA1_Stream6->CR |= (1u<<27);
  DMA1_Stream6->CR &= ~(1u<<26);
  DMA1_Stream6->CR &= ~(1u<<25);

  // Set the PFCTRL bit in the DMA_SxCR register if the peripheral is intended to be the flow controller (We have 3 wire UART)
  // Configure the stream priority in the DMA_SxCR register. (Don't know if I even need it)

  // Configure the FIFO usage (Or direct mode) in DMA_SxFCR
  DMA1_Stream6->FCR = 0;

  // Configure the data transfer direction (01 memory to peripheral)
  DMA1_Stream6->CR |= (1u<<6);
  DMA1_Stream6->CR &= ~(1u<<7);

  // Configure Peripheral and memory incremented/fixed mode
  DMA1_Stream6->CR |= DMA_STREAMx_MINC;

  // Configure interrupts after half and/or full transfer, and/or errors in the DMA_SxCR register
  DMA1_Stream6->CR |= DMA_STREAMx_TCIE; // Unsure if Interrupt handler in STM32F401RE

  // Argument is STM32 specific interrupt number
  NVIC_EnableIRQ(interrupt_number);

  // Activate the stream by setting the EN bit in the DMA_SxCR register.
  DMA1_Stream6->CR |= DMA_STREAMx_EN;

  // Enable UART2 transmitter
  USART2->CR3 |= USART2_DMAT;

  printf("dma_uart2_init : Activated the stream6\r\n");

  // Enable Global interrupts -> Remember NVIC_EnableIRQ is different
  //__enable_irq();
}
