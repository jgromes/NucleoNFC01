# NucleoNFC01
C library for X-NUCLEO-NFC01A1 shield, based on M24SR-Y dynamic NFC tag. Tested on STM32 NUCLEO-L152RE board.

## STM32 CubeMX Setup for STM32 NUCLEO-L152RE board
1. Enable I2C1 in Pinout & Configuration > Connectivity > I2C1
2. Enable I2C1 Event interrupt in Pinout & Configuration > Connectivity > I2C1 > NVIC Settings
3. Change I2C1 in pins in Pinout view from PB6/PB7 (default) to PB8/PB9: Click pins PB8/PB9 and select I2C1_SCL/I21_SDA.

## Library API Reference
Unless specified otherwise, all of the API functions return M24SR status code. See `M24SR.h` for details.

`unsigned short M24SR_Init(I2C_HandleTypeDef* hi2c);`  
Initialize M24SR using the provided HAL I2C handle.

`void M24SR_GetUid(unsigned char* buf);`  
Write M24SR's unique ID as a byte array into the provided buffer. UID length is defined in `M24SR.h` as M24SR_UID_LENGTH (7 bytes).

`unsigned short M24SR_WriteTag(char* payload);`  
Write `payload` null-terminated string into M24SR tag. The payload will be saved as `text/plain` MIME type, maximum paxload length is 233 bytes.

`unsigned short M24SR_ReadTag(unsigned char* buf);`  
Read contents of the M24SR tag into provided buffer `buf`. This will include not only the payload, but also contents of the whole NDEF file (i.e. length, type, etc.).

## UART Interface
Included in the library is a simple UART interface to control the NFC tag via terminal. To use the UART interface, call function `M24SR_UART_Init(huart, hi2c)`, and provide the appropriate HAL handles to UART and I2C. After calling this function, it is not necessary to call `M24SR_Init(hi2c)`. To read UART command, call `M24SR_UART_ReadCmd(timeout)` with specified timeout.

### UART command/response structure
| Byte 0       | Byte 1 - 4     | Byte 5 - n         |
|:------------:|:--------------:|:------------------:|
| Command type | Length or Code | Payload (optional) |

* **Command type**: ASCII character specifying which command to execute.   
`I` to initialize M24SR, `W` to write to tag and `R` to read tag.
* **Length or Code**: For commands, this field specifies the length of payload if it is present (0 otherwise).  
For responses, this field contains the M24SR response code.  
Format is ASCII-encoded decimal in both cases.  
* **Payload**: Optional field, only present in write command and read response.

**UART Command Examples**  
* `I0000`  
Initialize M24SR.
* `W0012Hello World!`  
Write the string "Hello World!" into the tag NDEF message.
* `R0000`  
Read NDEF message from tag.
