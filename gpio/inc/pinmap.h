/*
 * pinmap.h
 *
 *  Created on: 29 Dec 2021
 *      Author: fahim
 *
 * Copyright (c) 2021-2022.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef INC_PINMAP_H_
#define INC_PINMAP_H_
// NULL may create redefinition warning with stdio
#ifndef NULL
#define NULL          ( (void *) 0)
#endif
typedef enum {FALSE, TRUE} bool;
#define GPIO_NUMBER   16U

// Standardized LED and button names
#define LED1     PA_5   // LD2 [Green Led]
#define BUTTON1  PC_13  // B1 [Blue PushButton]

typedef enum {
  PORT_A,
  PORT_B,
  PORT_C,
  PORT_D,
  PORT_E,
  PORT_H

} eGPIO_PORT;

typedef enum {
  GPIO_VALUE_LOW =0,
  GPIO_VALUE_HIGH =1

} eGPIO_Value;

typedef enum {
  push_pull=0,      // reset state (Usually for most of pins)
  open_drain=1      // You may need internal or external pull up resistor

}eOutputType;

typedef enum {
  INPUT_MODE = 0b00,
  OUTPUT_MODE = 0b01,
  ALTERNATE_MODE = 0b10,
  ANALOG_MODE = 0b11

} eGPIO_MODE;

typedef enum {
  AF0 = 0b0000,
  AF1 = 0b0001,
  AF2 = 0b0010,
  AF3 = 0b0011,
  AF4 = 0b0100,
  AF5 = 0b0101,
  AF6 = 0b0110,
  AF7 = 0b0111,
  AF8 = 0b1000,
  AF9 = 0b1001,
  AF10 = 0b1010,
  AF11 = 0b1011,
  AF12 = 0b1100,
  AF13 = 0b1101,
  AF14 = 0b1110,
  AF15 = 0b1111

} eALTERNATE_MODE;

typedef enum {
  no_pupd = 0,
  pull_up = 1,
  pull_down = 2,
  reserved = 3

}eGPIO_PUPD_Config;

typedef struct {
  uint8_t             config;
  eGPIO_PORT          port;
  //This parameter can be one of (0..15).
  uint8_t             gpio_pin;
#define CONFIG_GPIO_MODE_PRESENT    0x01
#define CONFIG_OUTPUT_TYPE_PRESENT  0x02
#define CONFIG_PUPD_CONFIG_PRESENT  0x04
  eGPIO_MODE          gpio_mode;
  eALTERNATE_MODE     alternate_func;
  eOutputType         output_type;
  eGPIO_PUPD_Config   pupd_config;

}tGPIO_Config;

typedef enum {
  FALLING_EDGE,
  RISING_EDGE,

}eGPIO_trigger_select;
//typedef enum {
//    ALT0  = 0x100,
//} ALTx;

//typedef enum {
//    PA_0       = 0x00,
//    PA_1       = 0x01,
//    PA_2       = 0x02,
//    PA_3       = 0x03,
//    PA_4       = 0x04,
//    PA_4_ALT0  = PA_4  | ALT0, // same pin used for alternate HW
//    PA_5       = 0x05,
//    PA_6       = 0x06,
//    PA_7       = 0x07,
//    PA_7_ALT0  = PA_7  | ALT0, // same pin used for alternate HW
//    PA_8       = 0x08,
//    PA_9       = 0x09,
//    PA_10      = 0x0A,
//    PA_11      = 0x0B,
//    PA_12      = 0x0C,
//    PA_13      = 0x0D,
//    PA_14      = 0x0E,
//    PA_15      = 0x0F,
//    PA_15_ALT0 = PA_15 | ALT0, // same pin used for alternate HW
//    PB_0       = 0x10,
//    PB_0_ALT0  = PB_0  | ALT0, // same pin used for alternate HW
//    PB_1       = 0x11,
//    PB_1_ALT0  = PB_1  | ALT0, // same pin used for alternate HW
//    PB_2       = 0x12,
//    PB_3       = 0x13,
//    PB_3_ALT0  = PB_3  | ALT0, // same pin used for alternate HW
//    PB_4       = 0x14,
//    PB_4_ALT0  = PB_4  | ALT0, // same pin used for alternate HW
//    PB_5       = 0x15,
//    PB_5_ALT0  = PB_5  | ALT0, // same pin used for alternate HW
//    PB_6       = 0x16,
//    PB_7       = 0x17,
//    PB_8       = 0x18,
//    PB_8_ALT0  = PB_8  | ALT0, // same pin used for alternate HW
//    PB_9       = 0x19,
//    PB_9_ALT0  = PB_9  | ALT0, // same pin used for alternate HW
//    PB_10      = 0x1A,
//    PB_12      = 0x1C,
//    PB_13      = 0x1D,
//    PB_14      = 0x1E,
//    PB_15      = 0x1F,
//    PC_0       = 0x20,
//    PC_1       = 0x21,
//    PC_2       = 0x22,
//    PC_3       = 0x23,
//    PC_4       = 0x24,
//    PC_5       = 0x25,
//    PC_6       = 0x26,
//    PC_7       = 0x27,
//    PC_8       = 0x28,
//    PC_9       = 0x29,
//    PC_10      = 0x2A,
//    PC_11      = 0x2B,
//    PC_12      = 0x2C,
//    PC_13      = 0x2D,
//    PC_14      = 0x2E,
//    PC_15      = 0x2F,
//    PD_2       = 0x32,
//    PH_0       = 0x70,
//    PH_1       = 0x71,
//
//    /**** ADC internal channels ****/
//    ADC_TEMP = 0xF0,
//    ADC_VREF = 0xF1,
//    ADC_VBAT = 0xF2,
//
//    /**** USB FS pins ****/
//    USB_OTG_FS_DM = PA_11,
//    USB_OTG_FS_DP = PA_12,
//    USB_OTG_FS_ID = PA_10,
//    USB_OTG_FS_SOF = PA_8,
//    USB_OTG_FS_VBUS = PA_9,
//
//    /**** OSCILLATOR pins ****/
//    RCC_OSC32_IN = PC_14,
//    RCC_OSC32_OUT = PC_15,
//    RCC_OSC_IN = PH_0,
//    RCC_OSC_OUT = PH_1,
//
//    /**** DEBUG pins ****/
//    SYS_JTCK_SWCLK = PA_14,
//    SYS_JTDI = PA_15,
//    SYS_JTDO_SWO = PB_3,
//    SYS_JTMS_SWDIO = PA_13,
//    SYS_JTRST = PB_4,
//    SYS_WKUP = PA_0,
//
//    // Not connected
//    NC = (int)0xFFFFFFFF
//} PinName;


#endif /* INC_PINMAP_H_ */
