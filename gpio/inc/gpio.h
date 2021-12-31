/*
 * gpio.h
 *
 *  Created on: 29 Dec 2021
 *      Author: fahim
 */

#ifndef GPIO_H_
#define GPIO_H_
#ifndef INC_PINMAP_H_
#include "pinmap.h"
#endif
/*
 * @Description  With open drain you need internal/external pull up resistor as
 * pin will be in floating state for output 1
 * @param  tGPIO_Config specifies the configuration of GPIO pin need to be set before use
 */
void gpio_init(tGPIO_Config *gpio_config);

/*
 * @Description  Write value to specified GPIO pin
 * @param  eGPIO_PORT
 * @param  uint8_t gpio_pin
 * @param  eGPIO_Value
 */
void gpio_write(eGPIO_PORT port, uint8_t gpio_pin, eGPIO_Value value);

/*
 * @Description  Read value from specified GPIO pin
 * @param  eGPIO_PORT
 * @param  uint8_t gpio_pin
 * @return  eGPIO_Value
 */
eGPIO_Value gpio_read(eGPIO_PORT port, uint8_t gpio_pin);

#endif /* GPIO_H_ */
