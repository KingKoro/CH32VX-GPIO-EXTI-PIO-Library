/**
 *  CH32VX GPIO Library
 *
 *  file         : ch32v_gpio.h
 *  description  : ch32v gpio library main header
 *  Created on: Jan 12, 2023
 *  Author: larry
 * 	https://github.com/bitbank2/CH32V_Experiments/tree/master
 *
 */

#ifndef __CH32V_GPIO_H
#define __CH32V_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "debug.h"

#define BITBANG

// GPIO pin states
enum {
	OUTPUT = 0,
	INPUT,
	INPUT_PULLUP,
	INPUT_PULLDOWN
};

#define PROGMEM
#define memcpy_P memcpy
#define pgm_read_byte(s) *(uint8_t *)s
#define pgm_read_word(s) *(uint16_t *)s

// Wrapper methods
void delay(int i);
//
// Digital pin functions use a numbering scheme to make it easier to map the
// pin number to a port name and number
// The GPIO ports A-D become the most significant nibble of the pin number
// for example, to use Port C pin 7 (C7), use the pin number 0xC7
//
void pinMode(uint8_t u8Pin, int iMode);
uint8_t digitalRead(uint8_t u8Pin);
void digitalWrite(uint8_t u8Pin, uint8_t u8Value);
void pinInterrupt(uint8_t u8Pin, int iMode, FunctionalState NewState, EXTITrigger_TypeDef TriggerType, void (*func)(void));

// The Wire library is a C++ class; I've created a work-alike to my
// BitBang_I2C API which is a set of C functions to simplify I2C
#ifdef BITBANG
void I2CSetSpeed(int iSpeed);
int I2CReadRegister(uint8_t iAddr, uint8_t u8Register, uint8_t *pData, int iLen);
void I2CInit(uint8_t u8SDA, uint8_t u8SCL, int iSpeed);
#else
void I2CInit(int iSpeed);
#endif

void I2CWrite(uint8_t u8Addr, uint8_t *pData, int iLen);
void I2CRead(uint8_t u8Addr, uint8_t *pData, int iLen);
int I2CTest(uint8_t u8Addr);

// SPI1 (polling mode)
void SPI_write(uint8_t *pData, int iLen);
void SPI_begin(int iSpeed, int iMode);


// Random stuff
void Standby82ms(uint8_t iTicks);
void breatheLED(uint8_t u8Pin, int iPeriod);

#ifdef __cplusplus
}
#endif

#endif