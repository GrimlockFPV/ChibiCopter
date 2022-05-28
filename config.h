/*
 * config.h
 *
 *  Created on: May 21, 2015
 *      Author: Pablo
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/* GYRO */
#define GYRO_LPF              GYRO_LPF_3600
// Gyro sampling rate.
#define GYRO_RATE             8000               // in herz
// This is the update rate to the motors
// Equivalent to loop time in Multiwii or cleanfligh
#define LOOP_RATE             2000
#define GYRO_RANGE            GYRO_RANGE_500
#define ACCEL_RANGE           ACCEL_RANGE_4G

#define SAMPLES_PER_LOOP    (GYRO_RATE/LOOP_RATE)

// Rates in degrees/second
#define ROLL_RATE             500
#define PITCH_RATE            500
#define YAW_RATE              400
// AUX RATE has no unit
#define AUX_RATE              1000

/* MACROS */

#endif /* CONFIG_H_ */
