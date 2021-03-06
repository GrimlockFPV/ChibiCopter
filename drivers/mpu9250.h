/*
 * mpu9250.h
 *
 *  Created on: May 5, 2015
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

#ifndef MPU9250_H_
#define MPU9250_H_

/*
 * Register names according to the datasheet.
 * According to the InvenSense document
 * "MPU-9250 and MPU-9250 Register Map
 * and Descriptions Revision 3.2", there are no registers
 * at 0x02 ... 0x18, but according other information
 * the registers in that unknown area are for gain
 * and offsets.
 */

#define MPU9250_SMPLRT_DIV           25   // R/W
#define MPU9250_CONFIG               26   // R/W
#define MPU9250_GYRO_CONFIG          27   // R/W
#define MPU9250_ACCEL_CONFIG         28   // R/W
#define MPU9250_ACCEL_CONFIG2        29   // R/W

#define MPU9250_INT_PIN_CFG          55   // R/W
#define MPU9250_INT_ENABLE           56   // R/W

#define MPU9250_ACCEL_XOUT_H         59   // R
#define MPU9250_ACCEL_XOUT_L         60   // R
#define MPU9250_ACCEL_YOUT_H         61   // R
#define MPU9250_ACCEL_YOUT_L         62   // R
#define MPU9250_ACCEL_ZOUT_H         63   // R
#define MPU9250_ACCEL_ZOUT_L         64   // R

#define MPU9250_TEMP_ZOUT_H          65   // R
#define MPU9250_TEMP_ZOUT_L          66   // R

#define MPU9250_GYRO_XOUT_H          67   // R
#define MPU9250_GYRO_XOUT_L          68   // R
#define MPU9250_GYRO_YOUT_H          69   // R
#define MPU9250_GYRO_YOUT_L          70   // R
#define MPU9250_GYRO_ZOUT_H          71   // R
#define MPU9250_GYRO_ZOUT_L          72   // R

#define MPU9250_SIGNAL_PATH_RESET   104   // R/W
#define MPU9250_USER_CTRL           106   // R/W
#define MPU9250_PWR_MGMT_1          107   // R/W
#define MPU9250_PWR_MGMT_2          108   // R/W
#define MPU9250_WHO_AM_I            117   // R

/*
 * Values of the different configurations
 *
 */

#define GYRO_LPF_250          0
#define GYRO_LPF_184          1
#define GYRO_LPF_92           2
#define GYRO_LPF_41           3
#define GYRO_LPF_20           4
#define GYRO_LPF_10           5
#define GYRO_LPF_5            6
#define GYRO_LPF_3600         7

#define ACCEL_LPF_218         1
#define ACCEL_LPF_99          2
#define ACCEL_LPF_44          3
#define ACCEL_LPF_21          4
#define ACCEL_LPF_10          5
#define ACCEL_LPF_5           6
#define ACCEL_LPF_420         7

#define GYRO_RANGE_250        (0<<3)
#define GYRO_RANGE_500        (1<<3)
#define GYRO_RANGE_1000       (2<<3)
#define GYRO_RANGE_2000       (3<<3)

#define ACCEL_RANGE_2G        (0<<3)
#define ACCEL_RANGE_4G        (1<<3)
#define ACCEL_RANGE_8G        (2<<3)
#define ACCEL_RANGE_16G       (3<<3)

#if (GYRO_RANGE == GYRO_RANGE_2000)
#define GYRO_SCALE            (-1.0f / 16.4f)
#define GYRO_SCALE_INT        (-16)
#elif (GYRO_RANGE == GYRO_RANGE_1000)
#define GYRO_SCALE            (-1.0f / 32.8f)
#define GYRO_SCALE_INT        (-32)
#elif (GYRO_RANGE == GYRO_RANGE_500)
#define GYRO_SCALE            (-1.0f / 65.5f)
#define GYRO_SCALE_INT        (-64)
#elif (GYRO_RANGE == GYRO_RANGE_250)
#define GYRO_SCALE            (-1.0f / 131.0f)
#define GYRO_SCALE_INT        (-128)
#else
#error  Wrong Gyro range
#endif

#if (ACCEL_RANGE == ACCEL_RANGE_16G)
#define ACCEL_SCALE        (-2048.0f)
#elif (ACCEL_RANGE == ACCEL_RANGE_8G)
#define ACCEL_SCALE        (-4096.0f)
#elif (ACCEL_RANGE == ACCEL_RANGE_4G)
#define ACCEL_SCALE        (-8192.0f)
#elif (ACCEL_RANGE == ACCEL_RANGE_2G)
#define ACCEL_SCALE        (-16384.0f)
#else
#error  Wrong Accel range
#endif

#if (LOOP_RATE == 2000)
#define CYCLE_TIME            (0.00005f)
#define MICROS_PER_LOOP       500
#elif (LOOP_RATE == 1000)
#define CYCLE_TIME            (0.0001f)
#define MICROS_PER_LOOP       1000
#elif (LOOP_RATE == 500)
#define CYCLE_TIME            (0.0002f)
#define MICROS_PER_LOOP       2000
#elif (LOOP_RATE == 250)
#define CYCLE_TIME            (0.0004f)
#define MICROS_PER_LOOP       4000
#else
#error  Wrong Gyro sample rate
#endif

/*
 * According to the data sheet the device should return
 * 0x71 as reply to Who am I.
 */
#define MPU9250_WHO_AM_I_REPLY 0x71

#define CALIB_CYCLES   (2048*8)

#define X_AXIS         0
#define Y_AXIS         1
#define Z_AXIS         2
#define ROLL           0
#define PITCH          1
#define YAW            2

struct SensorData {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  float fRoll;
  float fPitch;
  float fYaw;
};


enum {
  SPI_DUMMY_BYTE = 0,
  ACCEL_XOUT_H,
  ACCEL_XOUT_L,
  ACCEL_YOUT_H,
  ACCEL_YOUT_L,
  ACCEL_ZOUT_H,
  ACCEL_ZOUT_L,
  TEMP_XOUT_H,
  TEMP_XOUT_L,
  GYRO_XOUT_H,
  GYRO_XOUT_L,
  GYRO_YOUT_H,
  GYRO_YOUT_L,
  GYRO_ZOUT_H,
  GYRO_ZOUT_L,
  IMU_DATA_SIZE,
};


int MPUInit(void);
bool GetMPUData(void);
void StartCalibration(void);
//void ResetPID(void);
void ClearFilters(void);
void DisableMPUInt(void);
#endif /* MPU9250_H_ */
