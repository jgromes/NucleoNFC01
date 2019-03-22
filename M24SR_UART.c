/*
 * M24SR_UART.c
 *
 *  Created on: 22. 3. 2019
 *      Author: xgrome00
 */

#include "M24SR_UART.h"

/**
  * @brief  Prints a single ASCII character to UART.
  * @param  c:  Character to print.
  * @retval None
  */
void printChar(char c) {
  HAL_UART_Transmit(M24SR_huart, (unsigned char *)(&c), 1, HAL_MAX_DELAY);
}

/**
  * @brief  Prints a null-terminated string of characters to UART.
  * @param  msg:  Null-terminated string to print.
  * @retval None
  */
void print(char* msg) {
  HAL_UART_Transmit(M24SR_huart, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/**
  * @brief  Prints a null-terminated string of characters to UART, with appended CR-LF.
  * @param  msg:  Null-terminated string to print.
  * @retval None
  */
void println(char* msg) {
  print(msg);
  print("\r\n");
}

/**
  * @brief  Prints 16-bit number formatted as hexadecimal to UART.
  * @param  msg:  16-bit number to print.
  * @retval None
  */
void printShortHex(unsigned short s) {
  char buf[5];
  sprintf(buf, "%04x", s);
  print(buf);
}

/**
  * @brief  Prints 8-bit number formatted as hexadecimal to UART.
  * @param  msg:  8-bit number to print.
  * @retval None
  */
void printCharHex(unsigned char c) {
  char buf[3];
  sprintf(buf, "%02x", c);
  print(buf);
}

/**
  * @brief  Initializes M24SR and the UART interface.
  * @param  huart:  Pointer to HAL UART handle.
  * @param  hi2c:   Pointer to HAL I2C handle.
  * @retval None
  */
void M24SR_UART_Init(UART_HandleTypeDef* huart, I2C_HandleTypeDef* hi2c) {
  // set handles
  M24SR_huart = huart;
  M24SR_hi2c = hi2c;

  // initialize M24SR
  M24SR_Init(M24SR_hi2c);
}

/**
  * @brief  Waits for UART command and if one is received, executes it.
  *         NOTE: This is a blocking function!
  * @param  timeout:  Timeout in ms.
  * @retval None
  */
void M24SR_UART_ReadCmd(uint32_t timeout) {
  unsigned char header[M24SR_UART_INTERFACE_HEADER_LEN + 1];
  if(HAL_UART_Receive(M24SR_huart, header, M24SR_UART_INTERFACE_HEADER_LEN, timeout) != HAL_TIMEOUT) {
    // received header, parse it
    header[M24SR_UART_INTERFACE_HEADER_LEN] = '\0';
    unsigned char cmd = header[0];
    unsigned char payloadLen = strtol((char *)header + 1, NULL, 10);
    switch(cmd) {
      case M24SR_UART_INTERFACE_CMD_INIT: {
        // initialization command, no payload
        unsigned short state = M24SR_Init(M24SR_hi2c);

        // send response
        printChar(M24SR_UART_INTERFACE_CMD_INIT);
        printShortHex(state);

        // send UID
        if(state == M24SR_OK) {
          unsigned char uid[M24SR_UID_LENGTH];
          M24SR_GetUid(uid);
          for(int i = 0; i < M24SR_UID_LENGTH; i++) {
            printCharHex(uid[i]);
          }
        }
        println("");

      } break;
      case M24SR_UART_INTERFACE_CMD_WRITE: {
        // write command, read the payload
        char* payload = (char*) malloc(payloadLen + 1);
        if(HAL_UART_Receive(M24SR_huart, (unsigned char*)payload, payloadLen, timeout) != HAL_TIMEOUT) {
          // write data to tag
          payload[payloadLen] = '\0';
          unsigned short state = M24SR_WriteTag(payload);

          // send response
          printChar(M24SR_UART_INTERFACE_CMD_WRITE);
          printShortHex(state);
          println("");

          free(payload);
        }
      } break;
      case M24SR_UART_INTERFACE_CMD_READ: {
        // read command, no payload
        unsigned char buf[256];
        unsigned short state = M24SR_ReadTag(buf);

        // send response
        printChar(M24SR_UART_INTERFACE_CMD_READ);
        printShortHex(state);

        // send data read from tag
        println((char*)buf);

      } break;
    }
  }
}
