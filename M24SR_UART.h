/*
 * M24SR_UART.h
 *
 *  Created on: 22. 3. 2019
 *      Author: xgrome00
 */

#ifndef M24SR_UART_H_
#define M24SR_UART_H_

#include <stdlib.h>
#include <string.h>
#include "stm32l1xx_hal.h"

#include "M24SR.h"

// pointer to HAL UART handle
UART_HandleTypeDef* M24SR_huart;

// pointer to HAL I2C handle
I2C_HandleTypeDef* M24SR_hi2c;

// interface constants
#define M24SR_UART_INTERFACE_HEADER_LEN     5
#define M24SR_UART_INTERFACE_CMD_INIT       'I'
#define M24SR_UART_INTERFACE_CMD_READ       'R'
#define M24SR_UART_INTERFACE_CMD_WRITE      'W'

// function prototypes
void printChar(char c);
void print(char* msg);
void println(char* msg);
void printShortHex(unsigned short s);
void printCharHex(unsigned char c);

// user-level API
void M24SR_UART_Init(UART_HandleTypeDef* huart, I2C_HandleTypeDef* hi2c);
void M24SR_UART_ReadCmd(uint32_t timeout);

#endif /* M24SR_UART_H_ */
