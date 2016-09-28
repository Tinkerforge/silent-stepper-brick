/* silent-stepper-brick
 * Copyright (C) 2015 Olaf Lüke <olaf@tinkerforge.com>
 *
 * config.h: Silent Stepper Brick specific configuration
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "communication.h"
#include "bricklib/drivers/board/sam3s/SAM3S.h"
#include "silent-stepper.h"

#define BRICK_FIRMWARE_VERSION_MAJOR 2
#define BRICK_FIRMWARE_VERSION_MINOR 0
#define BRICK_FIRMWARE_VERSION_REVISION 1

#define BRICK_HARDWARE_VERSION_MAJOR 1
#define BRICK_HARDWARE_VERSION_MINOR 0
#define BRICK_HARDWARE_VERSION_REVISION 0

#define BRICK_DEVICE_IDENTIFIER 19

// ************** DEBUG SETTINGS **************
#define DEBUG_SPI_STACK 1
#define DEBUG_I2C_EEPROM 0
#define DEBUG_STARTUP 1
#define DEBUG_BRICKLET 1
#define DEBUG_STEPPER 1
//#define PROFILING
//#define PROFILING_TIME 100 // After how many seconds profiling is printed

#define DISABLE_JTAG_ON_STARTUP
#define LOGGING_SERIAL
#define LOGGING_LEVEL LOGGING_DEBUG
//#define LOGGING_LEVEL LOGGING_NONE


// ************** BRICK SETTINGS **************

// Frequencies
#define BOARD_MCK      64000000 // Frequency of brick
#define BOARD_MAINOSC  16000000 // Frequency of oscillator
#define BOARD_ADC_FREQ 16000000 // Frequency of ADC
#define BOARD_OSC_EXTERNAL      // Use external oscillator
#define SPI_FREQ       2000000  // Frequency of SPI communication to TMC2130


// UART for console output (printf)
#define PIN_CONSOLE_RXD  {1 << 21, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_CONSOLE_TXD  {1 << 22, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}

#define CONSOLE_BAUDRATE 115200
#define CONSOLE_USART    USART1
#define CONSOLE_ID       ID_USART1
#define CONSOLE_PINS     {PIN_CONSOLE_RXD, PIN_CONSOLE_TXD}

#define PINS_UART        {PIN_CONSOLE_RXD, PIN_CONSOLE_TXD}

// TWI
// TWI version
#define TWI_V3XX

// TWI stack definitions (for reading of eeproms from Bricks in stack)
#define TWI_STACK           TWI1
#define PIN_TWI_TWD_STACK   {1 << 4, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_TWI_TWCK_STACK  {1 << 5, PIOB, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}
#define PINS_TWI_STACK      PIN_TWI_TWD_STACK, PIN_TWI_TWCK_STACK

// TWI bricklet definitions (for bricklets, spi select and brick specific
//                           functions)
#define TWI_BRICKLET           TWI0
#define PIN_TWI_TWD_BRICKLET   {1 << 3, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_TWI_TWCK_BRICKLET  {1 << 4, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PINS_TWI_BRICKLET      PIN_TWI_TWD_BRICKLET, PIN_TWI_TWCK_BRICKLET

// USB
// USB VBUS monitoring pin for USB plug and play
#define PIN_USB_DETECT  {1 << 26, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
// USB product descriptor (name of brick)
#define PRODUCT_DESCRIPTOR { \
	USBStringDescriptor_LENGTH(20), \
    USBGenericDescriptor_STRING, \
    USBStringDescriptor_UNICODE('S'), \
    USBStringDescriptor_UNICODE('i'), \
    USBStringDescriptor_UNICODE('l'), \
    USBStringDescriptor_UNICODE('e'), \
    USBStringDescriptor_UNICODE('n'), \
    USBStringDescriptor_UNICODE('t'), \
    USBStringDescriptor_UNICODE(' '), \
    USBStringDescriptor_UNICODE('S'), \
    USBStringDescriptor_UNICODE('t'), \
    USBStringDescriptor_UNICODE('e'), \
    USBStringDescriptor_UNICODE('p'), \
    USBStringDescriptor_UNICODE('p'), \
    USBStringDescriptor_UNICODE('e'), \
    USBStringDescriptor_UNICODE('r'), \
    USBStringDescriptor_UNICODE(' '), \
    USBStringDescriptor_UNICODE('B'), \
    USBStringDescriptor_UNICODE('r'), \
    USBStringDescriptor_UNICODE('i'), \
    USBStringDescriptor_UNICODE('c'), \
    USBStringDescriptor_UNICODE('k') \
}


// SPI
#define PIN_SPI_MISO        {1 << 12, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT} //
#define PIN_SPI_MOSI        {1 << 13, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT} //
#define PIN_SPI_SPCK        {1 << 14, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT} //

#define PINS_SPI            PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_SPCK

#define PIN_SPI_SELECT_SLAVE  {1 << 11, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT} //

// LED
#define PIN_LED_STD_BLUE    {1 << 10, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT} //
#define PIN_LED_STD_RED     {1 << 9, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT} //
#define PINS_STD_LED        PIN_LED_STD_BLUE, PIN_LED_STD_RED 
#define PINS_LED            PINS_STD_LED

#define LED_STD_BLUE        0
#define LED_STD_RED         1


// Brick Detect
// Set low by master
// TODO: Change name
#define PIN_DETECT        {1 << 6, PIOB, ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT} //

// ************** INTERRUPT PRIORITIES ***********
#define PRIORITY_EEPROM_MASTER_TWI0  1
#define PRIORITY_EEPROM_SLAVE_TWI1   1
#define PRIORITY_STACK_SLAVE_SPI     5
#define PRIORITY_PROFILING_TC0       0
#define PRIORITY_STEPPER_TC0         6

// ************** BRICKLET SETTINGS **************

// Number of bricklet ports
#define BRICKLET_NUM 2

// BRICKLET A
#define BRICKLET_A_ADDRESS 84
#define BRICKLET_A_ADC_CHANNEL 4

#define BRICKLET_A_PIN_1_AD   {1 << 0, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT} //
#define BRICKLET_A_PIN_2_DA   {1 << 2, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT} //
#define BRICKLET_A_PIN_3_PWM  {1 << 0, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT} //
#define BRICKLET_A_PIN_4_IO   {1 << 25, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT} //
#define BRICKLET_A_PIN_SELECT {0, 0, 0, 0, 0}

// BRICKLET B
#define BRICKLET_B_ADDRESS 80
#define BRICKLET_B_ADC_CHANNEL 5

#define BRICKLET_B_PIN_1_AD   {1 << 1, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT} //
#define BRICKLET_B_PIN_2_DA   {1 << 3, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT} //
#define BRICKLET_B_PIN_3_PWM  {1 << 1, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT} //
#define BRICKLET_B_PIN_4_IO   {1 << 16, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT} //
#define BRICKLET_B_PIN_SELECT {0, 0, 0, 0, 0}

// *************** STEPPER DEBUGGING **************
#if(DEBUG_STEPPER)
#define logstepperd(str, ...) do{logd("stepper: " str, ##__VA_ARGS__);}while(0)
#define logstepperi(str, ...) do{logi("stepper: " str, ##__VA_ARGS__);}while(0)
#define logstepperw(str, ...) do{logw("stepper: " str, ##__VA_ARGS__);}while(0)
#define logsteppere(str, ...) do{loge("stepper: " str, ##__VA_ARGS__);}while(0)
#define logstepperf(str, ...) do{logf("stepper: " str, ##__VA_ARGS__);}while(0)
#else
#define logstepperd(str, ...) {}
#define logstepperi(str, ...) {}
#define logstepperw(str, ...) {}
#define logsteppere(str, ...) {}
#define logstepperf(str, ...) {}
#endif
#endif


// *************** TMC2130 PINS ***************
#define PIN_ENABLE      {1 << 23, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT} //
#define PIN_STEP        {1 << 15, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT} //
#define PIN_DIRECTION   {1 << 28, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT} //
#define PIN_VREF        {1 << 13, PIOB, ID_PIOB, PIO_INPUT,    PIO_DEFAULT} //

#define PIN_CLK         {1 << 27, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT} //use internal clock
#define PIN_PWR_SW_3V3  {1 << 8,  PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT} //enable 3V3

#define PIN_PWR_SDO     {1 << 5,  PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT} // SDO_CFG0 chopper off time default
#define PIN_PWR_SDI     {1 << 6,  PIOA, ID_PIOA, PIO_INPUT,    PIO_DEFAULT} // SDI_CFG1, set 16 uSteps, stealth
#define PIN_PWR_SCK     {1 << 2,  PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT} // SCK_CFG2, set 16 uSteps, stealth
#define PIN_PWR_CS      {1 << 7,  PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT} // CS_CFG3, Current Setting external sense resistors with analog input enabled
#define PIN_CFG4     	{1 << 17, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT} // DCEN_CFG4, set chopper hysteresis default
#define PIN_CFG5       	{1 << 24, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT} // DCIN_CFG5, set chopper blank time best for stealth
#define PIN_DIAG1       {1 << 30, PIOA, ID_PIOA, PIO_INPUT,    PIO_PULLUP}  // DIAG1, Index
#define PIN_DIAG0       {1 << 31, PIOA, ID_PIOA, PIO_INPUT,    PIO_PULLUP}  // DIAG0, Error

#define PINS_CONFIG     PIN_PWR_SDO, PIN_PWR_SDI, PIN_PWR_SCK, PIN_PWR_CS, PIN_CFG4, \
                        PIN_CFG5, PIN_DIAG1, PIN_DIAG0, PIN_PWR_SW_3V3

#define PINS_ACTIVE     PIN_PWR_SDO, PIN_PWR_SDI, PIN_PWR_SCK, PIN_PWR_CS, PIN_CFG4, \
                        PIN_CFG5, PIN_DIAG1, PIN_DIAG0, PIN_PWR_SW_3V3, PIN_CLK, PIN_ENABLE, \
                        PIN_STEP, PIN_DIRECTION, PIN_VREF

#define PWR_SDO        0 // SPI Data Output
#define PWR_SDI        1 // SPI Data Input
#define PWR_SCK        2 // SPI serial clock input
#define PWR_CS         3 // SPI chip select input (negative active)
#define CFG_4          4 // dcStep enable input (SPI_MODE=1) - tie to GND for normal operation (no dcStep)
#define CFG_5          5 // dcStep gating input for axis synchronization (SPI_MODE=1)
#define CFG_DIAG1      6 // Diagnostic Outputs
#define CFG_DIAG0      7 //     "        "
#define PWR_SW_3V3     8 // *
#define PWR_CLK        9 // *
#define PWR_ENABLE    10 // *
#define PWR_STEP      11 // * ToDo
#define PWR_DIRECTION 12 // *
#define PWR_VREF      13 // *

#define CFG_DCO        14 // dcStep ready output
//ToDo: dcStep DCO Pin for load dependet speed control -> Page 3 (datasheet)
//#define CFG_SPI_MODE   15 // SPI mode selector =1 when activated

// ************** POWER MANAGEMENT **************
#define VOLTAGE_MAX_VALUE 4095
#define VOLTAGE_MIN_VALUE 0
#define VOLTAGE_MIN_DAC (3300/6)
#define VOLTAGE_MAX_DAC (3300*5/6)

#define VOLTAGE_STACK_CHANNEL 3
#define VOLTAGE_STACK_MULTIPLIER 11
#define VOLTAGE_STACK_REFERENCE 3300
#define VOLTAGE_STACK_PIN {1 << 20, PIOA, ID_PIOA, PIO_INPUT, PIO_PULLDOWN}
#define VOLTAGE_STACK_SWITCH_PIN {1 << 29, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}

#define VOLTAGE_EXTERN_CHANNEL 2
#define VOLTAGE_EXTERN_MULTIPLIER 11
#define VOLTAGE_EXTERN_REFERENCE 3300
#define VOLTAGE_EXTERN_PIN {1 << 19, PIOA, ID_PIOA, PIO_INPUT, PIO_PULLDOWN}

#define VREF_CHANNEL 0
#define VREF_MULTIPLIER 1
#define VREF_REFERENCE 2750
#define VREF_DEFAULT_CURRENT 800

#define MIN_EXTERN_VOLTAGE 8000
#define MIN_EXTERN_VALUE ((MIN_EXTERN_VOLTAGE * VOLTAGE_MAX_VALUE) / \
                          (VOLTAGE_EXTERN_MULTIPLIER* \
                           VOLTAGE_EXTERN_REFERENCE))

#define STEPPER_CURRENT_CHANNEL 1
#define STEPPER_CURRENT_MULTIPLIER 2
#define STEPPER_CURRENT_REFERENCE 3300
#define STEPPER_CURRENT_PIN {1 << 18, PIOA, ID_PIOA, PIO_INPUT, PIO_PULLDOWN} //????? gibt es nicht mehr!

// 15: 0.012 Ohm Resistor
// 1/6: Minimum value for DAC
// 5/6: Maximim value for DAC
#define VREF_MIN_CURRENT 0
#define VREF_MAX_CURRENT 1640*3.3/2.5

