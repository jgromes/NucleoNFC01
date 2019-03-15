/*
 * M24SR.c
 *
 *  Created on: 8. 3. 2019
 *      Author: xgrome00
 */

#include "M24SR.h"

// block counter
static unsigned char M24SR_blockNumber;

// array to save M24SR UID
static unsigned char M24SR_Uid[M24SR_UID_LENGTH];

// lookup table for CRC-A calculation
static const unsigned short crc_table[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

unsigned short getCRC(const unsigned char* data, unsigned long len, unsigned short crc) {
  for(unsigned long i = 0; i < len; i++) {
    crc = (crc >> 8) ^ crc_table[(crc ^ data[i]) & 0x00FF];
  }

  return crc;
}

void appendCRC(unsigned char* data, unsigned long len) {
  unsigned long crc = getCRC(data, len, CRC_A_INIT);

  data[len] = (unsigned char)(crc & 0xFF);
  data[len + 1] = (unsigned char)((crc >> 8) & 0xFF);
}

void getI2CSession(void) {
  // single-byte command with no CRC
  uint8_t data[] = {0x26};

  // send the command
  HAL_I2C_Master_Transmit(M24SR_hi2c, M24SR_I2C_ADDR | M24SR_I2C_WRITE, data, 1, HAL_MAX_DELAY);

  // there is no response for this command

  // wait a while (prevents some reset-related errors)
  HAL_Delay(1);
}

void releaseI2CSession(void) {
  // see section 7.4 of MS24SR datasheet

  // send start condition and address
  HAL_I2C_Master_Sequential_Transmit_IT(M24SR_hi2c, M24SR_I2C_ADDR | M24SR_I2C_WRITE, NULL, 0, I2C_FIRST_FRAME);

  // wait for timeout
  HAL_Delay(M24SR_I2C_RELEASE_TIMEOUT);

  // send stop condition
  SET_BIT(M24SR_hi2c->Instance->CR1, I2C_CR1_STOP);
}

void sendCommand(unsigned char* cmd, unsigned long len) {
  // allocate memory for the message
  unsigned char* msg = (unsigned char*) malloc(len + 2);

  // copy data to allocated memory
  memcpy(msg, cmd, len);

  // append CRC
  appendCRC(msg, len);

  // send the message
  HAL_I2C_Master_Transmit(M24SR_hi2c, M24SR_I2C_ADDR | M24SR_I2C_WRITE, msg, len + 2, HAL_MAX_DELAY);

  // flip block counter
  M24SR_blockNumber ^= 0x01;

  // free allocated memory
  free(msg);
}

void getResponse(unsigned char* buf, unsigned char len) {
  // poll device ready
  // TODO timeout
  do {
    HAL_I2C_Master_Transmit(M24SR_hi2c, M24SR_I2C_ADDR | M24SR_I2C_WRITE, NULL, 0, HAL_MAX_DELAY);
  } while(M24SR_hi2c->ErrorCode == HAL_I2C_ERROR_AF);

  // read response
  HAL_I2C_Master_Receive(M24SR_hi2c, M24SR_I2C_ADDR | M24SR_I2C_READ, buf, len, HAL_MAX_DELAY);
}

unsigned short getResponseData(unsigned long len, unsigned long codePos, unsigned char* data, unsigned char dataLen) {
  // allocate memory for the message
  unsigned char* msg = (unsigned char*) malloc(len);

  // get response
  getResponse(msg, len);

  // TODO verify CRC

  // get status code
  unsigned short statusCode = (msg[codePos] << 8 ) | msg[codePos + 1];

  // copy response data
  if(dataLen > 0) {
    memcpy(data, msg + 1, dataLen);
  }

  // free allocated memory
  free(msg);

  return(statusCode);
}

unsigned short getResponseCode(unsigned long len, unsigned long codePos) {
  return(getResponseData(len, codePos, NULL, 0));
}

unsigned short NDEFtagApplicationSelect(void) {
  // build the command
  unsigned char cmd[] = {
    0x02 | M24SR_blockNumber,   // PCB field
    0x00,                       // C-APDU SelectFile class byte (CLA)
    0xA4,                       // C-APDU SelectFile instruction (INS)
    0x04,                       // C-APDU SelectFile P1
    0x00,                       // C-APDU SelectFile P2
    0x07,                       // C-APDU SelectFile length of data (Lc)
    0xD2,                       // C-APDU SelectFile NTAG Application ID (Data)
    0x76,                       // C-APDU SelectFile NTAG Application ID (Data)
    0x00,                       // C-APDU SelectFile NTAG Application ID (Data)
    0x00,                       // C-APDU SelectFile NTAG Application ID (Data)
    0x85,                       // C-APDU SelectFile NTAG Application ID (Data)
    0x01,                       // C-APDU SelectFile NTAG Application ID (Data)
    0x01                        // C-APDU SelectFile NTAG Application ID (Data)
  };

  // send command
  sendCommand(cmd, 13);

  // get response
  return(getResponseCode(5, 1));
}

unsigned short selectFile(unsigned short file) {
  // build the command
  unsigned char cmd[] = {
    0x02 | M24SR_blockNumber,   // PCB field
    0x00,                       // C-APDU SelectFile class byte (CLA)
    0xA4,                       // C-APDU SelectFile instruction (INS)
    0x00,                       // C-APDU SelectFile P1
    0x0C,                       // C-APDU SelectFile P2
    0x02,                       // C-APDU SelectFile length of data (Lc)
    (unsigned char)((file >> 8) & 0xFF),  // C-APDU SelectFile System File ID (Data)
    (unsigned char)(file & 0xFF)          // C-APDU SelectFile System File ID (Data)
  };

  // send command
  sendCommand(cmd, 8);

  // get response
  return(getResponseCode(5, 1));
}

unsigned short readBinary(unsigned short offset, unsigned char* buf, unsigned char len) {
  // build the command
  unsigned char cmd[] = {
    0x02 | M24SR_blockNumber,   // PCB field
    0x00,                       // C-APDU ReadBinary class byte (CLA)
    0xB0,                       // C-APDU ReadBinary instruction (INS)
    (unsigned char)((offset >> 8) & 0xFF),  // C-APDU ReadBinary offset in file (P1)
    (unsigned char)(offset & 0xFF),         // C-APDU ReadBinary offset in file (P2)
    len                         // C-APDU ReadBinary number of bytes to read (Le)
  };

  // send command
  sendCommand(cmd, 6);

  // get response
  return(getResponseData(len + 5, len + 1, buf, len));
}

unsigned short updateBinary(unsigned short offset, unsigned char* data, unsigned char len) {
  // build the command
  unsigned char cmdPtr = 0;
  unsigned char* cmd = (unsigned char*) malloc(6 + len);
  cmd[cmdPtr++] = 0x02 | M24SR_blockNumber;   // PCB field;
  cmd[cmdPtr++] = 0x00;                       // C-APDU UpdateBinary class byte (CLA)
  cmd[cmdPtr++] = 0xD6;                       // C-APDU UpdateBinary instruction (INS)
  cmd[cmdPtr++] = (unsigned char)((offset >> 8) & 0xFF);  // C-APDU UpdateBinary offset in file (P1)
  cmd[cmdPtr++] = (unsigned char)(offset & 0xFF);         // C-APDU UpdateBinary offset in file (P2)
  cmd[cmdPtr++] = len;                        // C-APDU ReadBinary number of bytes to write (Lc)

  // copy data to write
  memcpy(cmd + cmdPtr, data, len);

  // send command
  sendCommand(cmd, 6 + len);

  // repeat until M24SR finishes update
  // TODO timeout
  while(1) {
    // check the whole response
    unsigned char resp[5];
    getResponse(resp, 5);

    // check waiting time extension request
    if(resp[0] == 0xF2) {
      // accept the request
      unsigned char wtxCmd[] = {0xF2, resp[1]};
      appendCRC(wtxCmd, 2);
      HAL_I2C_Master_Transmit(M24SR_hi2c, M24SR_I2C_ADDR | M24SR_I2C_WRITE, wtxCmd, 4, HAL_MAX_DELAY);
    } else {
      // return status code
      return((resp[1] << 8 ) | resp[2]);
    }
  }
}

void deselect(void) {
  // build the command (single byte with CRC)
  unsigned char cmd[] = {
    0xC2,                 // DESELECT command
    0xE0,                 // CRC LSB
    0xB4                  // CRC MSB
  };

  // send the command
  HAL_I2C_Master_Transmit(M24SR_hi2c, M24SR_I2C_ADDR | M24SR_I2C_WRITE, cmd, 3, HAL_MAX_DELAY);

  // response for DESELECT command has no status code
  getResponse(NULL, 3);
}

unsigned short M24SR_Init(I2C_HandleTypeDef* hi2c) {
  // set I2C handle
  M24SR_hi2c = hi2c;

  // reset block counter
  M24SR_blockNumber = 0;

  // start I2C session
  getI2CSession();

  // select NDEF application
  unsigned short state = NDEFtagApplicationSelect();
  if(state != M24SR_OK) {
    return(state);
  }

  // select system file
  state = selectFile(M24SR_SYSTEM_FILE);
  if(state != M24SR_OK) {
    return(state);
  }

  // read system file contents
  unsigned char buf[M24SR_SYSTEM_FILE_LENGTH];
  state = readBinary(0, buf, 18);
  if(state != M24SR_OK) {
    return(state);
  }

  // save UID
  memcpy(M24SR_Uid, buf + 8, M24SR_UID_LENGTH);

  // deselect file
  deselect();

  // release I2C session
  releaseI2CSession();

  return(state);
}

void M24SR_GetUid(unsigned char* buf) {
  memcpy(buf, M24SR_Uid, M24SR_UID_LENGTH);
}

unsigned short M24SR_WriteTag(char* payload) {
  // start I2C session
  getI2CSession();

  // select NDEF application
  unsigned short state = NDEFtagApplicationSelect();
  if(state != M24SR_OK) {
    return(state);
  }

  // select CC file
  state = selectFile(M24SR_CC_FILE);
  if(state != M24SR_OK) {
    return(state);
  }

  // read CC file
  unsigned char ccFile[M24SR_CC_FILE_LENGTH];
  state = readBinary(0, ccFile, M24SR_CC_FILE_LENGTH);
  if(state != M24SR_OK) {
    return(state);
  }

  // select NDEF file
  state = selectFile(M24SR_NDEF_FILE);
  if(state != M24SR_OK) {
    return(state);
  }

  // set length to 0
  unsigned char ndefLen[] = {0x00, 0x00};
  state = updateBinary(0, ndefLen, 2);
  if(state != M24SR_OK) {
    return(state);
  }

  // build the NDEF record
  const char* type = "text/plain";
  unsigned short recordLen = 3 + strlen(type) + strlen(payload);// + 1;
  unsigned char* record = (unsigned char*) malloc(recordLen);
  unsigned short recordPtr = 0;
  record[recordPtr++] = 0b11010010;                       // message begin and end, record not chunked, short record, no ID, MIME type
  record[recordPtr++] = strlen(type);                     // type length
  record[recordPtr++] = strlen(payload);                  // payload length
  memcpy(record + recordPtr, type, strlen(type));         // type field
  recordPtr += strlen(type);
  //record[recordPtr++] = ' ';                              // leading space
  memcpy(record + recordPtr, payload, strlen(payload));   // payload field

  // write record
  state = updateBinary(2, (unsigned char*)record, recordLen);
  if(state != M24SR_OK) {
    return(state);
  }

  // set correct length
  ndefLen[1] = recordLen;
  state = updateBinary(0, ndefLen, 2);
  if(state != M24SR_OK) {
    return(state);
  }

  // TODO verify successful read by checking length

  // deselect file
  deselect();

  // release I2C session
  releaseI2CSession();

  return(state);
}

unsigned short M24SR_ReadTag(unsigned char* buf) {
  // start I2C session
  getI2CSession();

  // select NDEF application
  unsigned short state = NDEFtagApplicationSelect();
  if(state != M24SR_OK) {
    return(state);
  }

  // select CC file
  state = selectFile(M24SR_CC_FILE);
  if(state != M24SR_OK) {
    return(state);
  }

  // read CC file
  unsigned char ccFile[M24SR_CC_FILE_LENGTH];
  state = readBinary(0, ccFile, M24SR_CC_FILE_LENGTH);
  if(state != M24SR_OK) {
    return(state);
  }

  // select NDEF file
  state = selectFile(M24SR_NDEF_FILE);
  if(state != M24SR_OK) {
    return(state);
  }

  // read record length
  unsigned char ndefLen[2];
  state = readBinary(0, ndefLen, 2);
  if(state != M24SR_OK) {
    return(state);
  }

  // read record
  unsigned char* record = (unsigned char*) malloc(ndefLen[1]);
  state = readBinary(2, record, ndefLen[1]);
  if(state != M24SR_OK) {
    return(state);
  }
  memcpy(buf, record, ndefLen[1]);
  buf[ndefLen[1]] = '\0';

  // deselect file
  deselect();

  // release I2C session
  releaseI2CSession();

  return(state);
}
