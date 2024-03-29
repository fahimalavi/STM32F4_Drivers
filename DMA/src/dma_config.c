/*
 * dma_config.c
 *
 *  Created on: 26 Jan 2022
 *      Author: fahim
 */
#include <stdint.h>
#include <stdio.h>
#ifndef INC_PINMAP_H_
#include "pinmap.h"
#endif
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

//#define DMA_HIFCR_CDMEIF5 (1u<<8) //
//#define DMA_HIFCR_CTEIF5  (1u<<9) //
//#define DMA_HIFCR_CHTIF5 (1u<<10) // Stream 5 clear half transfer interrupt flag
//#define DMA_HIFCR_CTCIF5 (1u<<11) // Stream 5 clear transfer complete interrupt flag
//#define DMA_HIFCR_CTCIF6 (1u<<21) // Stream 6 clear transfer complete interrupt flag
//#define DMA_HISR_HTIF5 DMA_HIFCR_CHTIF5 // Stream 5 half transfer interrupt flag. This bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the DMA_HIFCR register.
//#define DMA_HISR_TCIF5 DMA_HIFCR_CTCIF5 // Stream x transfer complete interrupt flag

#define USART_DMAT       (1u<<7) //DMA enable transmitter
#define USART_DMAR       (1u<<6) //DMA enable Receiver
#define USART_IDLEIE     (1u<<4) // USART enable IDLE interrupt

#define set_uart2_handle() (dma_uart2_handle++)
#define clear_uart2_handle() (dma_uart2_handle = NULL)
#define get_uart2_handle() (dma_uart2_handle)

static DMA_UART_Handle_t dma_uart2_handle = 0;

static bool dma_uart2_tx_stream_init(uint32_t peripheral_data_register_addr);
static bool dma_uart2_rx_stream_init(uint32_t peripheral_data_register_addr);

bool dma_uart2_init(DMA_UART_Handle_t *handle)
{
  uint32_t peripheral_data_register_addr = (uint32_t)&(USART2->DR);

  printf("dma_uart2_init called\r\n");

  // DMA handle in use
  if(get_uart2_handle() != 0)
  {
    printf("dma_uart2_init: DMA UART2 already configured\r\n");
    return FALSE;
  }

  // Disable global IRQ
  //__disable_irq();

  //Enable clock access to DMA1
  RCC->AHB1ENR |= DMA1EN_AHB1ENR;

  if(dma_uart2_rx_stream_init(peripheral_data_register_addr) == FALSE)
  {
    printf("dma_uart2_init: DMA UART2 RX configuration failure\r\n");
    return FALSE;
  }

  if(dma_uart2_tx_stream_init(peripheral_data_register_addr) == FALSE)
  {
    printf("dma_uart2_init: DMA UART2 TX configuration failure\r\n");
    return FALSE;
  }

  printf("dma_uart2_init : Activated the stream6\r\n");

  set_uart2_handle();

  *handle = get_uart2_handle();

  return TRUE;

  // Enable Global interrupts -> Remember NVIC_EnableIRQ is different
  //__enable_irq();
}

static bool dma_uart2_tx_stream_init(uint32_t peripheral_data_register_addr)
{
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
  // FFS : Put interrupt in specific DMA handle
  DMA1_Stream6->CR |= DMA_STREAMx_TCIE;

  // Argument is STM32 specific interrupt number
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);

  // Enabled UART2 transmitter (Enable DMA mode for transmission.)
  USART2->CR3 |= USART_DMAT;

  return TRUE;
}

static bool dma_uart2_rx_stream_init(uint32_t peripheral_data_register_addr)
{
  // Disable stream by resetting the EN bit in the DMA_SxCR register (RM: DMA1 request mapping)
  DMA1_Stream5->CR &= ~DMA_STREAMx_EN;

  while((DMA1_Stream5->CR & DMA_STREAMx_EN) != 0)
  {
  }

  // Disable interrupt flags of Stream5 (UART_TX)
  DMA1->HIFCR |= (1u<<6);
  DMA1->HIFCR |= (1u<<8);
  DMA1->HIFCR |= (1u<<9);
  DMA1->HIFCR |= (1u<<10);
  DMA1->HIFCR |= (1u<<11);

  DMA1_Stream5->CR &= ~DMA_STREAMx_DMEIE;
  DMA1_Stream5->CR &= ~DMA_STREAMx_TEIE;
  DMA1_Stream5->CR &= ~DMA_STREAMx_HTIE;
  DMA1_Stream5->CR &= ~DMA_STREAMx_TCIE;

  // Set the peripheral port register address in the DMA_SxPAR register (Stream6)
  DMA1_Stream5->PAR = peripheral_data_register_addr;

  // Select the DMA channel in the DMA_SxCR register. (Select Channel 4, may be later generalize)
  DMA1_Stream5->CR |= (1u<<27);
  DMA1_Stream5->CR &= ~(1u<<26);
  DMA1_Stream5->CR &= ~(1u<<25);

  // Set the PFCTRL bit in the DMA_SxCR register if the peripheral is intended to be the flow controller (We have 3 wire UART)
  // Configure the stream priority in the DMA_SxCR register. (Don't know if I even need it)

  // Configure the FIFO usage (Or direct mode) in DMA_SxFCR
  DMA1_Stream5->FCR = 0;

  // Configure the data transfer direction (00 peripheral to memory)
  DMA1_Stream5->CR &= ~(1u<<6);
  DMA1_Stream5->CR &= ~(1u<<7);

  // Configure Peripheral and memory incremented/fixed mode
  DMA1_Stream5->CR |= DMA_STREAMx_MINC;

  // Configure interrupts after half and/or full transfer, and/or errors in the DMA_SxCR register
  // FFS : Put interrupt in specific DMA handle
  DMA1_Stream5->CR |= DMA_STREAMx_TCIE; // Don't know yet what DMA interrupt to use upon reception of data

  // Argument is STM32 specific interrupt number
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  return TRUE;
}

bool dma_uart2_send(DMA_UART_Handle_t *handle, uint8_t *mem_addr, uint16_t size)
{
  // DMA handle in use
  if(get_uart2_handle() == 0)
  {
    printf("dma_uart2_send: DMA UART2 not configured or invalid handle\r\n");
    return FALSE;
  }
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

  // Set the memory address in the DMA_SxMA0R register
  DMA1_Stream6->M0AR = (uint32_t) mem_addr;

  // Configure the total number of data items to be transferred in the DMA_SxNDTR register.
  //    Number of data items to be transferred (0 up to 65535). This register can be written only
  //    When the stream is enabled, this register is read-only, indicating the remaining data items to be transmitted.
  DMA1_Stream6->NDTR = size;

  // Configure interrupts after half and/or full transfer, and/or errors in the DMA_SxCR register
  // FFS : Put interrupt in specific DMA handle
  DMA1_Stream6->CR |= DMA_STREAMx_TCIE;

  // Activate the stream by setting the EN bit in the DMA_SxCR register.
  DMA1_Stream6->CR |= DMA_STREAMx_EN;

  return TRUE;
}

bool dma_uart2_receive(DMA_UART_Handle_t *handle, uint8_t *mem_addr, uint16_t size)
{
  NVIC_DisableIRQ(USART2_IRQn);

  // DMA handle in use
  if(get_uart2_handle() == 0)
  {
    printf("dma_uart2_receive: DMA UART2 not configured or invalid handle\r\n");
    return FALSE;
  }
  // Disable stream by resetting the EN bit in the DMA_SxCR register (RM: DMA1 request mapping)
  DMA1_Stream5->CR &= ~DMA_STREAMx_EN;

  while((DMA1_Stream5->CR & DMA_STREAMx_EN) != 0)
  {
  }
  // Disable interrupt flags of Stream6 (UART_TX)
  DMA1->HIFCR |= (1u<<6);
  DMA1->HIFCR |= (1u<<8);
  DMA1->HIFCR |= (1u<<9);
  DMA1->HIFCR |= (1u<<10);
  DMA1->HIFCR |= (1u<<11);

  DMA1_Stream5->CR &= ~DMA_STREAMx_DMEIE;
  DMA1_Stream5->CR &= ~DMA_STREAMx_TEIE;
  DMA1_Stream5->CR &= ~DMA_STREAMx_HTIE;
  DMA1_Stream5->CR &= ~DMA_STREAMx_TCIE;

  // Set the memory address in the DMA_SxMA0R register
  DMA1_Stream5->M0AR = (uint32_t)mem_addr;

  // Configure the total number of data items to be transferred in the DMA_SxNDTR register.
  //    Number of data items to be transferred (0 up to 65535). This register can be written only
  //    When the stream is enabled, this register is read-only, indicating the remaining data items to be transmitted.
  DMA1_Stream5->NDTR = size;

  // Configure interrupts after half and/or full transfer, and/or errors in the DMA_SxCR register
  // FFS : Put interrupt in specific DMA handle
  DMA1_Stream5->CR |= DMA_STREAMx_TCIE;
  //DMA1_Stream5->CR |= DMA_STREAMx_HTIE;
  DMA1_Stream5->CR |= DMA_STREAMx_DMEIE;
  DMA1_Stream5->CR |= DMA_STREAMx_TEIE;

  // Activate the stream by setting the EN bit in the DMA_SxCR register.
  DMA1_Stream5->CR |= DMA_STREAMx_EN;

  // Enable Idle interrupt & set/cleared by software
  USART2->CR1 |= USART_IDLEIE;

  // Enable UART2 Receiver
  USART2->CR3 |= USART_DMAR;

  NVIC_EnableIRQ(USART2_IRQn);

  return TRUE;
}

