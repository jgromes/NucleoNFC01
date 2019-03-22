/*
 * M24SR.h
 *
 *  Created on: 8. 3. 2019
 *      Author: xgrome00
 */

#ifndef M24SR_H_
#define M24SR_H_

#include <stdlib.h>
#include <string.h>
#include "stm32l1xx_hal.h"

// pointer to HAL I2C handle
I2C_HandleTypeDef* M24SR_hi2c;

// M24SR I2C commands
#define M24SR_I2C_ADDR                              0xAC
#define M24SR_I2C_READ                              0x00
#define M24SR_I2C_WRITE                             0x01
#define M24SR_I2C_RELEASE_TIMEOUT                   41
#define M24SR_KILL_RF_SESSION                       0x52
#define M24SR_GET_I2C_SESSION                       0x26

// M24SR file IDs
#define M24SR_SYSTEM_FILE                           0xE101
#define M24SR_CC_FILE                               0xE103
#define M24SR_NDEF_FILE                             0x0001

// M24SR status codes
#define M24SR_OK                                    0x9000
#define M24SR_ERR_FILE_OVERFLOW_LE                  0x6280
#define M24SR_ERR_END_OF_FILE_REACHED               0x6282
#define M24SR_ERR_PASSWORD_REQUIRED                 0x6300
#define M24SR_ERR_PASSWORD_INCORRECT                0x63C0
#define M24SR_ERR_UPDATE_FAILED                     0x6581
#define M24SR_ERR_INCORRECT_LENGTH                  0x6700
#define M24SR_ERR_INCOMPATIBLE_CMD                  0x6981
#define M24SR_ERR_SECURITY_NOT_SATISFIED            0x6982
#define M24SR_ERR_REFERENCE_UNUSABLE                0x6984
#define M24SR_ERR_INCORRECT_LE_LC                   0x6A80
#define M24SR_ERR_FILE_APP_NOT_FOUND                0x6A82
#define M24SR_ERR_FILE_OVERFLOW_LC                  0x6A84
#define M24SR_ERR_INCORRECT_P1_P2                   0x6A86
#define M24SR_ERR_INCORRECT_INS                     0x6D00
#define M24SR_ERR_INCORRECT_CLA                     0x6E00

// custom status codes
#define M24SR_ERR_I2C_TIMEOUT                       0xFF00
#define M24SR_ERR_CRC_MISMATCH                      0xFF01
#define M24SR_ERR_WRITE_FAILED                      0xFF02

// M24SR file and field lengths
#define M24SR_SYSTEM_FILE_LENGTH                    0x12
#define M24SR_UID_LENGTH                            0x07
#define M24SR_CC_FILE_LENGTH                        0x0F

// initial CRC-A value
#define M24SR_CRC_INIT                              0x6363

// default I2C timeout in ms; used in user-level API, can be overridden when using low-level methods
#define M24SR_DEFAULT_I2C_TIMEOUT                   5000

// function prototypes
unsigned short getCRC(const unsigned char* data, unsigned long len, unsigned short crc);
void appendCRC(unsigned char* data, unsigned long len);
void killRFSession(void);
void getI2CSession(void);
void releaseI2CSession(void);
void sendCommand(unsigned char* cmd, unsigned long len);
unsigned short getResponse(unsigned char* buf, unsigned char len, unsigned long timeout);
unsigned short getResponseData(unsigned long len, unsigned long codePos, unsigned char* data, unsigned char dataLen, unsigned long timeout);
unsigned short getResponseCode(unsigned long len, unsigned long codePos, unsigned long timeout);
unsigned short NDEFtagApplicationSelect(void);
unsigned short selectFile(unsigned short file);
unsigned short readBinary(unsigned short offset, unsigned char* buf, unsigned char len);
unsigned short updateBinary(unsigned short offset, unsigned char* data, unsigned char len, unsigned long timeout);
void deselect(void);

// user-level API
unsigned short M24SR_Init(I2C_HandleTypeDef* hi2c);
void M24SR_GetUid(unsigned char* buf);
unsigned short M24SR_WriteTag(char* payload);
unsigned short M24SR_ReadTag(unsigned char* buf);

#endif /* M24SR_H_ */
