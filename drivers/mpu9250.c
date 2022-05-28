/*
 * mpu9250.c
 *
 *  Created on: Apr 26, 2015
 *      Author: Pablo
 *
 *  This file is part of ChibiFlight.

 ChibiFlight is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 ChibiFlight is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with ChibiFlight.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "config.h"
#include "mpu9250.h"
#include "filter.h"

/***********************************/
/* Global variables in this module */
/***********************************/

CC_ALIGN_DATA(32) static uint8_t TxBuffer[3];
CC_ALIGN_DATA(32) static uint8_t RxBuffer[3];
CC_ALIGN_DATA(32) static uint8_t GyroReadCommandBuffer[GYRO_DATA_SIZE] = { MPU9250_GYRO_XOUT_H | 0x80, 0, 0, 0, 0, 0, 0 };
CC_ALIGN_DATA(32) static uint8_t GyroReadDataBuffer[GYRO_DATA_SIZE];
CC_ALIGN_DATA(32) static uint8_t AccelReadCommandBuffer[ACCEL_DATA_SIZE] = { MPU9250_ACCEL_XOUT_H | 0x80, 0, 0, 0, 0, 0, 0 };
CC_ALIGN_DATA(32) static uint8_t AccelReadDataBuffer[ACCEL_DATA_SIZE];


struct SensorGyroData RawGyroData;
struct SensorGyroData RawAccelData;

int32_t CalibrationCycles = 0;
int32_t GyroBias[3];
int16_t GyroData[3] = {0, 0, 0};
int16_t AccelData[3] = {0, 0, 0};

void spi_error_cb(SPIDriver *spip) {

  (void)spip;

  chSysHalt("SPI error");
}

/*
 * Maximum speed SPI configuration (10MHz (SYSCLK/8), CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig MPUSPIConfig = {.circular = false, .slave = false,
                                       .data_cb = NULL,
                                       .error_cb = spi_error_cb,
                                       .ssport = GPIOA, .sspad = 4U, .cr1 =
                                           SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA,
                                       .cr2 = SPI_CR2_DS_2 | SPI_CR2_DS_1
                                           | SPI_CR2_DS_0};

/*
 * Low speed SPI configuration (625kHz (SYSCLK/128), CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig LSSpiConfig = {.circular = false, .slave = false,
                                      .data_cb = NULL, .error_cb = spi_error_cb,
                                      .ssport = GPIOA, .sspad = 4U, .cr1 =
                                          SPI_CR1_BR_2 | SPI_CR1_BR_1 |
                                          SPI_CR1_CPOL | SPI_CR1_CPHA,
                                      .cr2 = SPI_CR2_DS_2 | SPI_CR2_DS_1
                                          | SPI_CR2_DS_0};

static int ReadGyroData(void) {

  CC_ALIGN_DATA(32) static uint8_t TransmitBuffer[ACCEL_DATA_SIZE] = {MPU9250_ACCEL_XOUT_H | 0x80, 0, 0, 0, 0, 0, 0 };
  CC_ALIGN_DATA(32) static uint8_t ReceiveBuffer[ACCEL_DATA_SIZE];

  spiStart(&SPID1, &MPUSPIConfig); /* Setup transfer parameters.       */
  spiSelect(&SPID1); /* Slave Select assertion.          */
  spiExchange(&SPID1, GYRO_DATA_SIZE, GyroReadCommandBuffer, GyroReadDataBuffer); /* Atomic transfer operations.      */
  spiUnselect(&SPID1); /* Slave Select de-assertion.       */
  spiStop(&SPID1);
  return 0;
}

static int ReadAccelData(void) {

  CC_ALIGN_DATA(32) static uint8_t TransmitBuffer[ACCEL_DATA_SIZE] = {MPU9250_ACCEL_XOUT_H | 0x80, 0, 0, 0, 0, 0, 0 };
  CC_ALIGN_DATA(32) static uint8_t ReceiveBuffer[ACCEL_DATA_SIZE];
  uint8_t buf[ACCEL_DATA_SIZE];

  spiStart(&SPID1, &MPUSPIConfig); /* Setup transfer parameters.       */
  spiSelect(&SPID1); /* Slave Select assertion.          */
  //spiExchange(&SPID1, ACCEL_DATA_SIZE, TransmitBuffer, ReceiveBuffer); /* Atomic transfer operations.      */
  spiExchange(&SPID1, ACCEL_DATA_SIZE, AccelReadCommandBuffer, AccelReadDataBuffer); /* Atomic transfer operations.      */
  spiUnselect(&SPID1); /* Slave Select de-assertion.       */
  spiStop(&SPID1);
  for (int i = 0; i < 0; i++) buf[i] = AccelReadDataBuffer[i];
  return 0;
}
/* MPU9250ReadRegister:
 * Access SPI bus and read a register of the MPU9250 sensor
 * Register address is given in 'Address'. Read value is returned.
 */

static uint8_t MPU9250ReadRegister(uint8_t Address) {

  TxBuffer[0] = Address | 0x80;
  TxBuffer[1] = 0;
  TxBuffer[2] = 0;
  spiStart(&SPID1, &LSSpiConfig); /* Setup transfer parameters.       */
  spiSelect(&SPID1); /* Slave Select assertion.          */
  spiExchange(&SPID1, 3, TxBuffer, RxBuffer); /* Atomic transfer operations.      */
  spiUnselect(&SPID1); /* Slave Select de-assertion.       */
  spiStop(&SPID1);
  return RxBuffer[1];
}

/* MPU9250WriteRegister:
 * Access SPI bus and writes a register of the MPU9250 sensor
 * Register address is given in 'Address'.
 * Value to be written to the register is given in 'Value'.
 */

static void MPU9250WriteRegister(uint8_t Address, uint8_t Value) {

  TxBuffer[0] = Address;
  TxBuffer[1] = Value; // 8kHz => 19 (dec) devider = 400Hz
  spiStart(&SPID1, &LSSpiConfig); /* Setup transfer parameters.       */
  spiSelect(&SPID1); /* Slave Select assertion.          */
  spiSend(&SPID1, 2, TxBuffer); /* Send command                     */
  spiUnselect(&SPID1); /* Slave Select de-assertion.       */
  spiStop(&SPID1);
}

/* MPU9250Reset:
 * Resets the MPU9250 sensor
 */

static void MPU9250Reset(void) {
  MPU9250WriteRegister(MPU9250_PWR_MGMT_1, 0x0);
  chThdSleepMilliseconds(10);
  //MPU9250WriteRegister(MPU9250_SIGNAL_PATH_RESET, 0x7);
  chThdSleepMilliseconds(10);
}

/* MPU9250GetWhoAmI:
 * Reads the 'Who am I' register
 * Returns 0 if values is as expected
 * or -1 in case of error
 */

static uint8_t MPU9250GetWhoAmI(void) {
  if (MPU9250ReadRegister(MPU9250_WHO_AM_I) != MPU9250_WHO_AM_I_REPLY) {
    return 1;
  }
  return 0;
}

/* MPU9250SetSampleRate:
 * Sets the sample rate of the MPU9250 sensor
 * This is going to be the sample rate of the
 * MPU9250 interrupt, and thus, the loop rate
 */

static void MPU9250SetSampleRate(uint16_t SampleRateHz) {
  uint16_t FilterFreq = 1000;
  int32_t Divisor;
  uint8_t Config;

  Config = MPU9250ReadRegister(MPU9250_CONFIG);
  if ((Config == GYRO_LPF_250) || (Config == GYRO_LPF_3600))
    return;

  // limit samplerate to filter frequency
  if (SampleRateHz > FilterFreq)
    SampleRateHz = FilterFreq;

  // calculate divisor, round to nearest integeter
  Divisor = (int32_t)(((float)FilterFreq / SampleRateHz) + 0.5f) - 1;

  // limit resulting divisor to register value range
  if (Divisor < 0)
    Divisor = 0;

  if (Divisor > 0xff)
    Divisor = 0xff;

  MPU9250WriteRegister(MPU9250_SMPLRT_DIV, (uint8_t)Divisor);
}

/* MPU9250Init:
 * Initialization of the MPU600 sensor
 * Returns 0 if OK, -1 if error (sensor not found).
 */
int MPUInit(void) {
  uint8_t ReturnValue;

  /*
   * Reset the device.
   */
  MPU9250Reset();

  ReturnValue = MPU9250GetWhoAmI();
  if (ReturnValue != 0)
    return ReturnValue;

  //  Select clock source
  //MPU9250WriteRegister(MPU9250_PWR_MGMT_1, 0x1);
  chThdSleepMilliseconds(10);
  // Disable i2c
  MPU9250WriteRegister(MPU9250_USER_CTRL, 0x10);
  chThdSleepMilliseconds(10);
  // Gyro range
  MPU9250WriteRegister(MPU9250_GYRO_CONFIG, GYRO_RANGE);
  chThdSleepMilliseconds(10);
  // Accel Range 8G
  MPU9250WriteRegister(MPU9250_ACCEL_CONFIG, ACCEL_RANGE);
  chThdSleepMilliseconds(10);
  // LPF
  MPU9250WriteRegister(MPU9250_CONFIG, ACCEL_LPF_44);
  chThdSleepMilliseconds(10);
  // Sample rate
  //MPU9250SetSampleRate(GYRO_RATE);
  chThdSleepMilliseconds(10);
  // Interrupt configuration
  // Active, rising edge interrupt
  MPU9250WriteRegister(MPU9250_INT_PIN_CFG, 0x10);
  chThdSleepMilliseconds(10);
  // Interrupt enable
  MPU9250WriteRegister(MPU9250_INT_ENABLE, 0x01);
  return 0;
}

void DisableMPUInt(void) {
  //MPU9250WriteRegister(MPU9250_INT_ENABLE, 0x00);
}

/* StartCalibration:
 * Satrts the Gyro calibration
 */

void StartCalibration(void) {
  //ResetPID();
  if (CalibrationCycles == 0)
    CalibrationCycles = CALIB_CYCLES;
}

float GyroFilterIns[2 * 3] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float GyroFilterOuts[2 * 3] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float AccelFilterIns[2 * 3] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float AccelFilterOuts[2 * 3] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

/* GetMPUData:
 * Get raw data from sensor
 * substract Sensor bias, scale the data
 * and gives the new data in 'GyroData'
 */
uint8_t SampleCounter = 0;

bool GetMPUData(void) {

  ReadGyroData();

  RawGyroData.roll = (int16_t)(GyroReadDataBuffer[GYRO_YOUT_H] << 8
      | GyroReadDataBuffer[GYRO_YOUT_L]);
  RawGyroData.roll /= (-1 * GYRO_SCALE_INT);
  RawGyroData.pitch = (int16_t)(GyroReadDataBuffer[GYRO_XOUT_H] << 8
      | GyroReadDataBuffer[GYRO_XOUT_L]);
  RawGyroData.pitch /= (-1 * GYRO_SCALE_INT);
  RawGyroData.yaw = (int16_t)(GyroReadDataBuffer[GYRO_ZOUT_H] << 8
      | GyroReadDataBuffer[GYRO_ZOUT_L]);
  RawGyroData.yaw /= GYRO_SCALE_INT;


  GyroData[ROLL] = Filter2ndOrder(RawGyroData.roll, &(GyroFilterIns[0]),
                                  &(GyroFilterOuts[0]),
                                  &(Butter2_04_ACoeffs[0]),
                                  &(Butter2_04_BCoeffs[0]));
  GyroData[PITCH] = Filter2ndOrder(RawGyroData.pitch, &(GyroFilterIns[2]),
                                   &(GyroFilterOuts[2]),
                                   &(Butter2_04_ACoeffs[0]),
                                   &(Butter2_04_BCoeffs[0]));
  GyroData[YAW] = Filter2ndOrder(RawGyroData.yaw, &(GyroFilterIns[4]),
                                 &(GyroFilterOuts[4]), &(Butter2_04_ACoeffs[0]),
                                 &(Butter2_04_BCoeffs[0]));

  if (CalibrationCycles == 0) {
    GyroData[ROLL] -= GyroBias[ROLL];
    GyroData[PITCH] -= GyroBias[PITCH];
    GyroData[YAW] -= GyroBias[YAW];

    ReadAccelData();
    RawAccelData.roll = (int16_t)(AccelReadDataBuffer[ACCEL_YOUT_H] << 8 | AccelReadDataBuffer[ACCEL_YOUT_L]);
    //RawAccelData.roll /= (-1 * ACCEL_SCALE_INT);
    RawAccelData.pitch = (int16_t)(AccelReadDataBuffer[ACCEL_XOUT_H] << 8 | AccelReadDataBuffer[ACCEL_XOUT_L]);
    //RawAccelData.pitch /= (-1 * ACCEL_SCALE_INT);
    RawAccelData.yaw = (int16_t)(AccelReadDataBuffer[ACCEL_ZOUT_H] << 8 | AccelReadDataBuffer[ACCEL_ZOUT_L]);
    //RawAccelData.yaw /= ACCEL_SCALE_INT;
    /*AccelData[ROLL] = Filter2ndOrder(RawAccelData.roll, &(AccelFilterIns[0]),
                                    &(AccelFilterOuts[0]),
                                    &(Butter2_04_ACoeffs[0]),
                                    &(Butter2_04_BCoeffs[0]));
    AccelData[PITCH] = Filter2ndOrder(RawAccelData.pitch, &(AccelFilterIns[2]),
                                     &(AccelFilterOuts[2]),
                                     &(Butter2_04_ACoeffs[0]),
                                     &(Butter2_04_BCoeffs[0]));
    AccelData[YAW] = Filter2ndOrder(RawAccelData.yaw, &(AccelFilterIns[4]),
                                   &(AccelFilterOuts[4]), &(Butter2_04_ACoeffs[0]),
                                   &(Butter2_04_BCoeffs[0]));
*/
    AccelData[ROLL] = RawAccelData.roll;
    AccelData[PITCH] = RawAccelData.pitch;
    AccelData[YAW] = RawAccelData.yaw;
  }
  else if (CalibrationCycles == CALIB_CYCLES) {
    palSetLine(LINE_LED_RED);
    GyroBias[ROLL] = 0;
    GyroBias[PITCH] = 0;
    GyroBias[YAW] = 0;
    CalibrationCycles--;
  }
  else {// (CalibrationCycles > 0)
    GyroBias[ROLL] += GyroData[ROLL];
    GyroBias[PITCH] += GyroData[PITCH];
    GyroBias[YAW] += GyroData[YAW];
    palToggleLine(LINE_LED_RED);
    if (CalibrationCycles == 1) {
      palClearLine(LINE_LED_RED);
      GyroBias[ROLL] /= CALIB_CYCLES;
      GyroBias[PITCH] /= CALIB_CYCLES;
      GyroBias[YAW] /= CALIB_CYCLES;
    }
    GyroData[ROLL] = 0;
    GyroData[PITCH] = 0;
    GyroData[YAW] = 0;
    AccelData[ROLL] = 0;
    AccelData[PITCH] = 0;
    AccelData[YAW] = 0;
    CalibrationCycles--;
  }
  SampleCounter++;
  if ((SampleCounter % SAMPLES_PER_LOOP) == 0)
    return TRUE;
  else
    return FALSE;

}

