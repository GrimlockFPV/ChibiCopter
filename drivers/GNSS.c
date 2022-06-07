/*
 * GNSS.c
 *
 *  Created on: 03.10.2020
 *      Author: SimpleMethod
 *
 *Copyright 2020 SimpleMethod
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy of
 *this software and associated documentation files (the "Software"), to deal in
 *the Software without restriction, including without limitation the rights to
 *use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *of the Software, and to permit persons to whom the Software is furnished to do
 *so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *THE SOFTWARE.
 ******************************************************************************
 */

#include "GNSS.h"
#include "ch.h"
#include "hal.h"


msg_t msg;

/*!
 * Structure initialization.
 * @param GNSS Pointer to main GNSS structure.
 * @param huart Pointer to uart handle.
 */
void GNSS_Init(GNSS_StateHandle *GNSS/*, UARTDriver *uartp*/) {
  //&UARTD2 = uartp;
  GNSS->year = 0;
  GNSS->month = 0;
  GNSS->day = 0;
  GNSS->hour = 0;
  GNSS->min = 0;
  GNSS->sec = 0;
  GNSS->fixType = 0;
  GNSS->lon = 0;
  GNSS->lat = 0;
  GNSS->height = 0;
  GNSS->hMSL = 0;
  GNSS->hAcc = 0;
  GNSS->vAcc = 0;
  GNSS->gSpeed = 0;
  GNSS->headMot = 0;
}

/*!
 *  Sends the basic configuration: Activation of the UBX standard, change of NMEA version to 4.10 and turn on of the Galileo system.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_LoadConfig(GNSS_StateHandle *GNSS) {
  msg = uartSendTimeout(&UARTD2, sizeof(configUBX) / sizeof(uint8_t), configUBX, TIME_INFINITE);
  if (msg != MSG_OK)
    chSysHalt("invalid return code");
  chThdSleepMilliseconds(250);
  msg = uartSendTimeout(&UARTD2, sizeof(setNMEA410) / sizeof(uint8_t), setNMEA410, TIME_INFINITE);
  if (msg != MSG_OK)
    chSysHalt("invalid return code");
  chThdSleepMilliseconds(250);
  msg = uartSendTimeout(&UARTD2, sizeof(setGNSS) / sizeof(uint8_t), setGNSS, TIME_INFINITE);
  if (msg != MSG_OK)
    chSysHalt("invalid return code");
  chThdSleepMilliseconds(250);
}
/*!
 * Searching for a header in data buffer and matching class and message ID to buffer data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParseBuffer(GNSS_StateHandle *GNSS) {

  for (int var = 0; var <= 100; ++var) {
    if (GNSS->uartWorkingBuffer[var] == 0xB5  && GNSS->uartWorkingBuffer[var + 1] == 0x62) {
      if (GNSS->uartWorkingBuffer[var + 2] == 0x27 && GNSS->uartWorkingBuffer[var + 3] == 0x03) { //Look at: 32.19.1.1 u-blox 8 Receiver description
        GNSS_ParseUniqID(GNSS);
      }
      else if (GNSS->uartWorkingBuffer[var + 2] == 0x01 && GNSS->uartWorkingBuffer[var + 3] == 0x21) { //Look at: 32.17.14.1 u-blox 8 Receiver description
        GNSS_ParseNavigatorData(GNSS);
      }
      else if (GNSS->uartWorkingBuffer[var + 2] == 0x01 && GNSS->uartWorkingBuffer[var + 3] == 0x07) { //ook at: 32.17.30.1 u-blox 8 Receiver description
        GNSS_ParsePVTData(GNSS);
      }
      else if (GNSS->uartWorkingBuffer[var + 2] == 0x01 && GNSS->uartWorkingBuffer[var + 3] == 0x02) { // Look at: 32.17.15.1 u-blox 8 Receiver description
        GNSS_ParsePOSLLHData(GNSS);
      }
    }
  }
}

/*!
 * Make request for unique chip ID data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetUniqID(GNSS_StateHandle *GNSS) {
  msg = uartSendTimeout(&UARTD2, sizeof(getDeviceID) / sizeof(uint8_t), getDeviceID, TIME_INFINITE);
  if (msg != MSG_OK)
    chSysHalt("invalid return code");
  uartReceiveTimeout(&UARTD2, 17, GNSS->uartWorkingBuffer, TIME_INFINITE);
}

/*!
 * Parse data to unique chip ID standard.
 * Look at: 32.19.1.1 u-blox 8 Receiver description
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParseUniqID(GNSS_StateHandle *GNSS) {
  for (int var = 0; var < 5; ++var) {
    GNSS->uniqueID[var] = GNSS->uartWorkingBuffer[10 + var];
  }
}

/*!
 * Make request for UTC time solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetNavigatorData(GNSS_StateHandle *GNSS) {
  msg = uartSendTimeout(&UARTD2, sizeof(getNavigatorData) / sizeof(uint8_t), getNavigatorData, TIME_INFINITE);
  if (msg != MSG_OK)
    chSysHalt("invalid return code");
  uartReceiveTimeout(&UARTD2, 28, GNSS->uartWorkingBuffer, TIME_INFINITE);
}

/*!
 * Parse data to UTC time solution standard.
 * Look at: 32.17.30.1 u-blox 8 Receiver description.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParseNavigatorData(GNSS_StateHandle *GNSS) {
  //uShort.bytes[0] = GNSS->uartWorkingBuffer[18];
  //uShort.bytes[1] = GNSS->uartWorkingBuffer[19];
  GNSS->year = extractInt(18, GNSS->uartWorkingBuffer);
  GNSS->month = GNSS->uartWorkingBuffer[20];
  GNSS->day = GNSS->uartWorkingBuffer[21];
  GNSS->hour = GNSS->uartWorkingBuffer[22];
  GNSS->min = GNSS->uartWorkingBuffer[23];
  GNSS->sec = GNSS->uartWorkingBuffer[24];
}

/*!
 * Make request for geodetic position solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetPOSLLHData(GNSS_StateHandle *GNSS) {
  msg = uartSendTimeout(&UARTD2, (sizeof(getPOSLLHData) / sizeof(uint8_t)), getPOSLLHData, TIME_INFINITE);
  if (msg != MSG_OK)
    chSysHalt("invalid return code");
  uartReceiveTimeout(&UARTD2, 36, GNSS->uartWorkingBuffer, TIME_INFINITE);
}

/*!
 * Parse data to geodetic position solution standard.
 * Look at: 32.17.14.1 u-blox 8 Receiver description.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParsePOSLLHData(GNSS_StateHandle *GNSS) {
  //for (int var = 0; var < 4; ++var) {
  //iLong.bytes[var] = GNSS->uartWorkingBuffer[var + 10];
  //}
  GNSS->lon = extractSignedLong(10, GNSS->uartWorkingBuffer);
  GNSS->fLon = (float)GNSS->lon / 10000000.0;

  //for (int var = 0; var < 4; ++var) {
  //iLong.bytes[var] = GNSS->uartWorkingBuffer[var + 14];
  //}
  GNSS->lat = extractSignedLong(14, GNSS->uartWorkingBuffer);
  GNSS->fLat = (float)GNSS->lat / 10000000.0;

  //for (int var = 0; var < 4; ++var) {
  //iLong.bytes[var] = GNSS->uartWorkingBuffer[var + 18];
  //}
  GNSS->height = extractSignedLong(18, GNSS->uartWorkingBuffer);

  //for (int var = 0; var < 4; ++var) {
  //iLong.bytes[var] = GNSS->uartWorkingBuffer[var + 22];
  //}
  GNSS->hMSL = extractSignedLong(22, GNSS->uartWorkingBuffer);

  //for (int var = 0; var < 4; ++var) {
  //uLong.bytes[var] = GNSS->uartWorkingBuffer[var + 26];
  //}
  GNSS->hAcc = extractLong(26, GNSS->uartWorkingBuffer);

  //for (int var = 0; var < 4; ++var) {
  //uLong.bytes[var] = GNSS->uartWorkingBuffer[var + 30];
  //}
  GNSS->vAcc = extractLong(30, GNSS->uartWorkingBuffer);
}

/*!
 * Make request for navigation position velocity time solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetPVTData(GNSS_StateHandle *GNSS) {
  msg = uartSendTimeout(&UARTD2, sizeof(getPVTData) / sizeof(uint8_t), getPVTData, TIME_INFINITE);
  if (msg != MSG_OK)
    chSysHalt("invalid return code");
  uartReceiveTimeout(&UARTD2, 100, GNSS->uartWorkingBuffer, TIME_INFINITE);
}

/*!
 * Parse data to navigation position velocity time solution standard.
 * Look at: 32.17.15.1 u-blox 8 Receiver description.
 * @param GNSS Pointer to main GNSS struccture.
 */
void GNSS_ParsePVTData(GNSS_StateHandle *GNSS) {
  //uShort.bytes[0] = GNSS->uartWorkingBuffer[10];
  GNSS->yearBytes[0] = GNSS->uartWorkingBuffer[10];
  //uShort.bytes[1] = GNSS->uartWorkingBuffer[11];
  GNSS->yearBytes[1] = GNSS->uartWorkingBuffer[11];
  GNSS->year = extractInt(10, GNSS->uartWorkingBuffer);
  GNSS->month = GNSS->uartWorkingBuffer[12];
  GNSS->day = GNSS->uartWorkingBuffer[13];
  GNSS->hour = GNSS->uartWorkingBuffer[14];
  GNSS->min = GNSS->uartWorkingBuffer[15];
  GNSS->sec = GNSS->uartWorkingBuffer[16];
  GNSS->fixType = GNSS->uartWorkingBuffer[26];

  for (int var = 0; var < 4; ++var) {
    //iLong.bytes[var] = GNSS->uartWorkingBuffer[var + 30];
    GNSS->lonBytes[var] = GNSS->uartWorkingBuffer[var + 30];
  }
  GNSS->lon = extractSignedLong(30, GNSS->uartWorkingBuffer);
  GNSS->fLon = (float)GNSS->lon / 10000000.0;
  for (int var = 0; var < 4; ++var) {
    //iLong.bytes[var] = GNSS->uartWorkingBuffer[var + 34];
    GNSS->latBytes[var] = GNSS->uartWorkingBuffer[var + 34];
  }
  GNSS->lat = extractSignedLong(34, GNSS->uartWorkingBuffer);
  GNSS->fLat = (float)GNSS->lat / 10000000.0;
  //for (int var = 0; var < 4; ++var) {
  //iLong.bytes[var] = GNSS->uartWorkingBuffer[var + 38];
  //}
  GNSS->height = extractSignedLong(38, GNSS->uartWorkingBuffer);

  for (int var = 0; var < 4; ++var) {
    //iLong.bytes[var] = GNSS->uartWorkingBuffer[var + 42];
    GNSS->hMSLBytes[var] = GNSS->uartWorkingBuffer[var + 42];
  }
  GNSS->hMSL = extractSignedLong(42, GNSS->uartWorkingBuffer);

  //for (int var = 0; var < 4; ++var) {
  //    uLong.bytes[var] = GNSS->uartWorkingBuffer[var + 46];
  //}
  GNSS->hAcc = extractLong(46, GNSS->uartWorkingBuffer);
  //for (int var = 0; var < 4; ++var) {
  //    uLong.bytes[var] = GNSS->uartWorkingBuffer[var + 50];
  //}
  GNSS->vAcc = extractLong(50, GNSS->uartWorkingBuffer);

  for (int var = 0; var < 4; ++var) {
    //iLong.bytes[var] = GNSS->uartWorkingBuffer[var + 66];
    GNSS->gSpeedBytes[var] = GNSS->uartWorkingBuffer[var + 66];
  }
  GNSS->gSpeed = extractSignedLong(66, GNSS->uartWorkingBuffer);

  //for (int var = 0; var < 4; ++var) {
  //iLong.bytes[var] = GNSS->uartWorkingBuffer[var + 70];
  //}
  GNSS->headMot = extractSignedLong(70, GNSS->uartWorkingBuffer) * 1e-5; // todo I'm not sure this good options.
}


/*!
 * Changing the GNSS mode.
 * Look at: 32.10.19 u-blox 8 Receiver description
 */
void GNSS_SetMode(GNSS_StateHandle *GNSS, uint16_t gnssMode) {
  if (gnssMode == 0) {
    msg = uartSendTimeout(&UARTD2, sizeof(setPortableMode) / sizeof(uint8_t), setPortableMode, TIME_INFINITE);
    if (msg != MSG_OK)
      chSysHalt("invalid return code");
  }
  else if (gnssMode == 1) {
    msg = uartSendTimeout(&UARTD2, sizeof(setStationaryMode) / sizeof(uint8_t), setStationaryMode, TIME_INFINITE);
    if (msg != MSG_OK)
      chSysHalt("invalid return code");
  }
  else if (gnssMode == 2) {
    msg = uartSendTimeout(&UARTD2, sizeof(setPedestrianMode) / sizeof(uint8_t), setPedestrianMode, TIME_INFINITE);
    if (msg != MSG_OK)
      chSysHalt("invalid return code");
  }
  else if (gnssMode == 3) {
    msg = uartSendTimeout(&UARTD2, sizeof(setAutomotiveMode) / sizeof(uint8_t), setAutomotiveMode, TIME_INFINITE);
    if (msg != MSG_OK)
      chSysHalt("invalid return code");
  }
  else if (gnssMode == 4) {
    msg = uartSendTimeout(&UARTD2, sizeof(setAutomotiveMode) / sizeof(uint8_t), setAutomotiveMode, TIME_INFINITE);
    if (msg != MSG_OK)
      chSysHalt("invalid return code");
  }
  else if (gnssMode == 5) {
    msg = uartSendTimeout(&UARTD2, sizeof(setAirbone1GMode) / sizeof(uint8_t), setAirbone1GMode, TIME_INFINITE);
    if (msg != MSG_OK)
      chSysHalt("invalid return code");
  }
  else if (gnssMode == 6) {
    msg = uartSendTimeout(&UARTD2, sizeof(setAirbone2GMode) / sizeof(uint8_t), setAirbone2GMode, TIME_INFINITE);
    if (msg != MSG_OK)
      chSysHalt("invalid return code");
  }
  else if (gnssMode == 7) {
    msg = uartSendTimeout(&UARTD2, sizeof(setAirbone4GMode) / sizeof(uint8_t), setAirbone4GMode, TIME_INFINITE);
    if (msg != MSG_OK)
      chSysHalt("invalid return code");
  }
  else if (gnssMode == 8) {
    msg = uartSendTimeout(&UARTD2, sizeof(setWirstMode) / sizeof(uint8_t), setWirstMode, TIME_INFINITE);
    if (msg != MSG_OK)
      chSysHalt("invalid return code");
  }
  else if (gnssMode == 9) {
    msg = uartSendTimeout(&UARTD2, sizeof(setBikeMode) / sizeof(uint8_t), setBikeMode, TIME_INFINITE);
    if (msg != MSG_OK)
      chSysHalt("invalid return code");
  }
}

//Given a spot in the payload array, extract four bytes and build a long
uint32_t extractLong(uint8_t spotToStart, uint8_t *payloadCfg) {
  uint32_t val = 0;
  val |= (uint32_t)payloadCfg[spotToStart + 0] << 8 * 0;
  val |= (uint32_t)payloadCfg[spotToStart + 1] << 8 * 1;
  val |= (uint32_t)payloadCfg[spotToStart + 2] << 8 * 2;
  val |= (uint32_t)payloadCfg[spotToStart + 3] << 8 * 3;
  return (val);
}

//Just so there is no ambiguity about whether a uint32_t will cast to a int32_t correctly...
int32_t extractSignedLong(uint8_t spotToStart, uint8_t *payloadCfg) {
  int32_t val = 0;
  val |= (int32_t)payloadCfg[spotToStart + 0] << 8 * 0;
  val |= (int32_t)payloadCfg[spotToStart + 1] << 8 * 1;
  val |= (int32_t)payloadCfg[spotToStart + 2] << 8 * 2;
  val |= (int32_t)payloadCfg[spotToStart + 3] << 8 * 3;
  return (val);
}

//Given a spot in the payload array, extract two bytes and build an int
uint16_t extractInt(uint8_t spotToStart, uint8_t *payloadCfg) {
  uint16_t val = 0;
  val |= (uint16_t)payloadCfg[spotToStart + 0] << 8 * 0;
  val |= (uint16_t)payloadCfg[spotToStart + 1] << 8 * 1;

  return (val);
}

//Just so there is no ambiguity about whether a uint16_t will cast to a int16_t correctly...
int16_t extractSignedInt(uint8_t spotToStart, uint8_t *payloadCfg) {
  int16_t val = 0;
  val |= (int16_t)payloadCfg[spotToStart + 0] << 8 * 0;
  val |= (int16_t)payloadCfg[spotToStart + 1] << 8 * 1;

  return (val);
}


