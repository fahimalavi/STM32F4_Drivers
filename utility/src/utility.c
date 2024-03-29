/*
 * utility.c
 *
 *  Created on: 30 Jan 2022
 *      Author: fahim
 */

#include <stdio.h>
#include "utility.h"
#include "stm32f4xx.h"
#include "gpio.h"
#include "uart.h"
#include "sys_tick.h"
#include "dma_config.h" // If you want to move to specific peripheral

// todo : https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices

#define BAUD_RATE 115200
#define DATA1_SIZE 12
#define DATA2_SIZE 3387
#define DATA3_SIZE 14

#define RX_DATA_BUFFER_SIZE 512
uint16_t data_size = 0;
uint8_t data[RX_DATA_BUFFER_SIZE]={0};

uint8_t DATA1[DATA1_SIZE] = "0123456789\r\n";
uint8_t DATA2[DATA2_SIZE] = "vitae, posuere at, velit. Cras lorem lorem, luctus ut, pellentesque eget, dictum placerat, augue. Sed molestie. Sed id risus quis diam luctus lobortis. Class aptent taciti sociosqu ad litora torquent per conubia nostra, per inceptos hymenaeos. Mauris ut quam vel sapien imperdiet ornare. In faucibus. Morbi vehicula. Pellentesque tincidunt tempus risus. Donec egestas. Duis ac arcu. Nunc mauris. Morbi non sapien molestie orci tincidunt adipiscing. Mauris molestie pharetra nibh. Aliquam ornare, libero at auctor ullamcorper, nisl arcu iaculis enim, sit amet ornare lectus justo eu arcu. Morbi sit amet massa. Quisque porttitor eros nec tellus. Nunc lectus pede, ultrices a, auctor non, feugiat nec, diam. Duis mi enim, condimentum eget, volutpat ornare, facilisis eget, ipsum. Donec sollicitudin adipiscing ligula. Aenean gravida nunc sed pede. Cum sociis natoque penatibus et magnis dis parturient montes, nascetur ridiculus mus. Proin vel arcu eu odio tristique pharetra. Quisque ac libero nec ligula consectetuer rhoncus. Nullam velit dui, semper et, lacinia vitae, sodales at, velit. Pellentesque ultricies dignissim lacus. Aliquam rutrum lorem ac risus. Morbi metus. Vivamus euismod urna. Nullam lobortis quam a felis ullamcorper viverra. Maecenas iaculis aliquet diam. Sed diam lorem, auctor quis, tristique ac, eleifend vitae, erat. Vivamus nisi. Mauris nulla. Integer urna. Vivamus molestie dapibus ligula. Aliquam erat volutpat. Nulla dignissim. Maecenas ornare egestas ligula. Nullam feugiat placerat velit. Quisque varius. Nam porttitor scelerisque neque. Nullam nisl. Maecenas malesuada fringilla est. Mauris eu turpis. Nulla aliquet. Proin velit. Sed malesuada augue ut lacus. Nulla tincidunt, neque vitae semper egestas, urna justo faucibus lectus, a sollicitudin orci sem eget massa. Suspendisse eleifend. Cras sed leo. Cras vehicula aliquet libero. Integer in magna. Phasellus dolor elit, pellentesque a, facilisis non, bibendum sed, est. Nunc laoreet lectus quis massa. Mauris vestibulum, neque sed dictum eleifend, nunc risus varius orci, in consequat enim diam vel arcu. Curabitur ut odio vel est tempor bibendum. Donec felis orci, adipiscing non, luctus sit amet, faucibus ut, nulla. Cras eu tellus eu augue porttitor interdum. Sed auctor odio a purus. Duis elementum, dui quis accumsan convallis, ante lectus convallis est, vitae sodales nisi magna sed dui. Fusce aliquam, enim nec tempus scelerisque, lorem ipsum sodales purus, in molestie tortor nibh sit amet orci. Ut sagittis lobortis mauris. Suspendisse aliquet molestie tellus. Aenean egestas hendrerit neque. In ornare sagittis felis. Donec tempor, est ac mattis semper, dui lectus rutrum urna, nec luctus felis purus ac tellus. Suspendisse sed dolor. Fusce mi lorem, vehicula et, rutrum eu, ultrices sit amet, risus. Donec nibh enim, gravida sit amet, dapibus id, blandit at, nisi. Cum sociis natoque penatibus et magnis dis parturient montes, nascetur ridiculus mus. Proin vel nisl. Quisque fringilla euismod enim. Etiam gravida molestie arcu. Sed eu nibh vulputate mauris sagittis placerat. Cras dictum ultricies ligula. Nullam enim. Sed nulla ante, iaculis nec, eleifend non, dapibus rutrum, justo. Praesent luctus. Curabitur egestas nunc sed libero. Proin sed turpis nec mauris blandit mattis. Cras eget nisi dictum augue malesuada malesuada. Integer id magna et ipsum cursus vestibulum. Mauris magna.\r\n";
uint8_t DATA3[DATA3_SIZE] = "Toggling LED\r\n";

void gpio_function_handler();


#if defined(CONFIGURE_DMA_UART2)
uint8_t wait_print = 0;
volatile uint8_t data_rxd = 0;
void receive_dma_uart2_data(DMA_UART_Handle_t *handle);
#endif

#if defined(UART2_INTERRUPT_ENABLE)
uint8_t uart_data = 0;
#endif //UART2_INTERRUPT_ENABLE

void uart_init_util()
{
#if defined(UART2_INTERRUPT_ENABLE)
  // RX Interrupt Enable
  uart2_config_init(BAUD_RATE, TRUE);
#else //
  uart2_config_init(BAUD_RATE, FALSE);
#endif //UART2_INTERRUPT_ENABLE
}

void program_sequence()
{
  printf("Welcome to baremetal program, press Enter to start\r\n");

  // Wait for Enter
#if defined(UART2_INTERRUPT_ENABLE)
  while(uart_data != 0x0A) {}
  printf("Enter pressed, read from interrupt waiting 2 seconds\r\n");
#else //
  while(uart2_read() != 0x0A) {}
  printf("Enter pressed, waiting 2 seconds\r\n");
#endif //UART2_INTERRUPT_ENABLE

  systick_delay_ms(2000);
  printf("2 seconds over, press switch button to toggle user LED\r\n");

  gpio_function_handler();
}

#if defined(CONFIGURE_DMA_UART2)
void data_communication_DMA()
{
  tGPIO_Config gpio_config;
  uint8_t i;
  DMA_UART_Handle_t handle;

  printf("Data TX transferring after LED \r\n");

  // Look in block diagram what Bus is connected to GPIO port
  // Enable clock to GPIO port A
  RCC->AHB1ENR |= 1u;
  // Enable clock to GPIO port C
  RCC->AHB1ENR |= (1u<<2);

  // Set PA5 to output : set bit 11 to 0 and 10 to 1
  gpio_config.gpio_pin = 5;
  gpio_config.port = PORT_A;

  gpio_config.gpio_mode = OUTPUT_MODE;
  gpio_config.config = CONFIG_GPIO_MODE_PRESENT;

  gpio_config.output_type = push_pull;
  gpio_config.config |= CONFIG_OUTPUT_TYPE_PRESENT;

  // Set PA5 config
  gpio_init(&gpio_config);

  dma_uart2_init(&handle);
  dma_uart2_send(&handle, DATA1, DATA1_SIZE);

  for(i = 0; i < 50 ; i++)
  {
    systick_delay_ms(100);
    gpio_toggle(PORT_A, 5);
  }

#if defined(CONFIGURE_DMA_UART2)
  //FFS
  if(wait_print & 4)
  {
    printf("DMA Error occured \r\n");
  }

  if(wait_print & 1)
  {
    printf("DMA UART TC interrupt \r\n");
  }

  if(wait_print & 2)
  {
    printf("UART TC interrupt \r\n");
  }
#endif

  dma_uart2_send(&handle, DATA2, DATA2_SIZE);

  // Check if uart receive data 3 times
  receive_dma_uart2_data(&handle);
  receive_dma_uart2_data(&handle);
  receive_dma_uart2_data(&handle);

  for(;;)
  {
    systick_delay_ms(2000);
    dma_uart2_send(&handle, DATA3, DATA3_SIZE);
    gpio_toggle(PORT_A, 5);
  }
}

void receive_dma_uart2_data(DMA_UART_Handle_t *handle)
{
  uint16_t i;

  data_rxd = 0;

  printf("Send data on UART upto %d bytes\r\n", RX_DATA_BUFFER_SIZE);

  dma_uart2_receive(handle, data, RX_DATA_BUFFER_SIZE);

  while(data_rxd == 0)
  {
    printf("Data not received, sleeping 1 second\r\n");
    systick_delay_ms(1000);
  }
  printf("Data received data_size:%d, data_rxd:%d\r\n", data_size, data_rxd);

  if(data_rxd == 0xFF)
  {
    printf("ERROR occurred\r\n");
  }
  for(i=0; i<data_size && data_size <= RX_DATA_BUFFER_SIZE;i++)
  {
    printf("%d:%c\t",i,(char)data[i]);
  }
  printf("\r\n");
}
#endif

void gpio_function_handler()
{
  tGPIO_Config gpio_config;

  // Look in block diagram what Bus is connected to GPIO port
  // Enable clock to GPIO port A
  RCC->AHB1ENR |= 1u;
  // Enable clock to GPIO port C
  RCC->AHB1ENR |= (1u<<2);

  // Set PA5 to output : set bit 11 to 0 and 10 to 1
  gpio_config.gpio_pin = 5;
  gpio_config.port = PORT_A;

  gpio_config.gpio_mode = OUTPUT_MODE;
  gpio_config.config = CONFIG_GPIO_MODE_PRESENT;

  gpio_config.output_type = push_pull;
  gpio_config.config |= CONFIG_OUTPUT_TYPE_PRESENT;

  // Set PA5 config
  gpio_init(&gpio_config);

  // Set PC13 to input
  gpio_config.gpio_pin = 13;
  gpio_config.port = PORT_C;
  gpio_config.gpio_mode = INPUT_MODE;
  gpio_config.config = CONFIG_GPIO_MODE_PRESENT;
  // Set PC13 config
  gpio_init(&gpio_config);
  gpio_enable_irq(&gpio_config, RISING_EDGE, EXTI15_10_IRQn);
#if defined(SYSTICK_TIMESTAMP_ENABLE)
  systick_start();
#endif

  /* Loop forever */
  for(;;)
  {
//    gpio_write(PORT_A, 5, GPIO_VALUE_HIGH);
//    while(gpio_read(PORT_C, 13) == GPIO_VALUE_LOW)
//      gpio_write(PORT_A, 5, GPIO_VALUE_LOW);
  }
}

void EXTI15_10_IRQHandler()
{
  // selected trigger request occurred on PC13
  if(EXTI->PR & (1u<<13))
  {
    //This bit is cleared by programming it to ‘1’
    EXTI->PR |= (1u<<13);
#if defined(SYSTICK_TIMESTAMP_ENABLE)
    printf("%u ms: Button pressed, toggling LED\r\n", (unsigned int)get_time_lapsed());
#else
    printf("Button pressed, toggling LED\r\n");
#endif
    //Toggle LED state
    gpio_toggle(PORT_A, 5);
  }
}


void USART2_IRQHandler()
{
  if(USART2->SR & (1u<<5))
  {
#if defined(UART2_INTERRUPT_ENABLE)
    //It is cleared by a read to the USART_DR register. The RXNE flag can also be cleared by writing a zero to it.
    uart_data = (USART2->DR & 0xFF);
#endif //UART2_INTERRUPT_ENABLE
  }
  if(USART2->SR & (1u<<6))
  {
#if defined(CONFIGURE_DMA_UART2)
    wait_print |= 0b10;
#endif
    // The TC bit can also be cleared by writing a '0' to it.
    USART2->SR &= ~(1u<<6);
  }
  // Idle interrupt
  if(USART2->SR & (1u<<4))
  {
    // It is cleared by a software sequence (an read to the USART_SR register followed by a read to the USART_DR register).
    // Note: The IDLE bit will not be set again until the RXNE bit has been set itself
    USART2->DR;

    USART2->CR1 &= ~(1u<<4); //
    DMA1_Stream5->CR &= ~(1u); // Disable DMA1 Stream5

    data_size = RX_DATA_BUFFER_SIZE - DMA1_Stream5->NDTR; // Number of bytes received
  }
}

void DMA1_Stream6_IRQHandler()
{
  //Stream x transfer complete interrupt flag
  if(DMA1->HISR & (1u<<21))
  {
#if defined(CONFIGURE_DMA_UART2)
    wait_print |= 0b01;
#endif
    DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
  }
#if defined(CONFIGURE_DMA_UART2)
  // FFS
  if (DMA1->HISR & 0x000C0000)    /* if an error occurred */
      wait_print = 0b100;
#endif
}

void DMA1_Stream5_IRQHandler()
{
  //Stream x HTIF5 (Half transfer)
  if(DMA1->HISR & DMA_HISR_HTIF5)
  {
#if defined(CONFIGURE_DMA_UART2)
    data_rxd |= 2;
#endif
    // Stream 5 half transfer interrupt flag. This bit is set by hardware. It is cleared by software writing 1 to the corresponding bit in the DMA_HIFCR register.
    DMA1->HIFCR |= DMA_HIFCR_CHTIF5;
  }

  //Stream x TCIF5 (Full transfer)
  if(DMA1->HISR & DMA_HISR_TCIF5)
  {
#if defined(CONFIGURE_DMA_UART2)
    data_rxd |= 1;
#endif
    // It is cleared by software writing 1 to the corresponding bit in the DMA_HIFCR register.
    DMA1->HIFCR |= DMA_HIFCR_CTCIF5;
  }
#if defined(CONFIGURE_DMA_UART2)
  // FFS
  if (DMA1->HISR & (DMA_HISR_DMEIF5|DMA_HISR_TEIF5))    /* if an error occurred */
  {
    data_rxd = 0xFF;
    DMA1->HIFCR |= DMA_HIFCR_CDMEIF5;
    DMA1->HIFCR |= DMA_HIFCR_CTEIF5;
    //FFS what to do in rx ERROR
  }
#endif
}

