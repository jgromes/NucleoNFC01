# NucleoNFC01
C library for X-NUCLEO-NFC01A1 shield, based on M24SR-Y dynamic NFC tag. Tested on STM32 NUCLEO-L152RE board.

## API Reference
Unless specified otherwise, all of the API functions return M24SR status code. See `M24SR.h` for details.

`unsigned short M24SR_Init(I2C_HandleTypeDef* hi2c);`  
Initialize M24SR using the provided HAL I2C handle.

`void M24SR_GetUid(unsigned char* buf);`  
Write M24SR's unique ID as a byte array into the provided buffer. UID length is defined in `M24SR.h` as M24SR_UID_LENGTH (7 bytes).

`unsigned short M24SR_WriteTag(char* payload);`  
Write `payload` null-terminated string into M24SR tag. The payload will be saved as `text/plain` MIME type, maximum paxload length is 233 bytes.

`unsigned short M24SR_ReadTag(unsigned char* buf);`  
Read contents of the M24SR tag into provided buffer `buf`. This will include not only the payload, but also contents of the whole NDEF file (i.e. length, type, etc.).
