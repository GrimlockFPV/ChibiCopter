/*
 ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#include <math.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "config.h"
#include "chprintf.h"
#include "drivers/mpu9250.h"
#include "drivers/kalman.h"

#define LINE_MPU_INT       PAL_LINE(GPIOB, 12U)
#define LINE_LED_BLUE      PAL_LINE(GPIOB, 7U)
#define LINE_LED_RED       PAL_LINE(GPIOB, 14U)
#define CHP (BaseSequentialStream*) &LPSD1
#define RC_CHANNEL_NUM     6
#define RAD_TO_DEG         (180 / 3.14f)

void InitHardware(void);
void Kalman_config(void);
void Kalman_update(void);

static binary_semaphore_t MPUDataReady; /* Semaphore fro the MPU thread */
extern int16_t GyroData[3];
extern int16_t AccelData[3];
uint16_t mpuCycles = 0;
enum { CH_1, CH_2, CH_3, CH_4, CH_5, CH_6 };
icucnt_t PulseWidth, PulsePeriod;
uint32_t channel[RC_CHANNEL_NUM];
uint8_t channel_select_counter;

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

float gyroXangle, gyroYangle; // Angle calculate using the gyro only
float compAngleX, compAngleY; // Calculated angle using a complementary filter
float kalAngleX, kalAngleY; // Calculated angle using a Kalman filter


/*
 * This is a periodic thread that does absolutely nothing except flashing a LED.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("Blinky");
  while (true) {
    palSetLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(200);
    palClearLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(200);
  }
}

/* MPU ISR:
 * Makes sure we are reading new data
 * MPU-INT is set to active high any read to clear
*/
static void mpu_isr_cb(void *arg) {

  (void)arg;
  chSysLockFromISR();
  /* Invocation of some I-Class system APIs, never preemptable.*/
  chBSemSignalI(&MPUDataReady);
  if (mpuCycles % 1000 == 0) {
    palToggleLine(LINE_LED_BLUE);
    mpuCycles = 0;
  }
  mpuCycles++;
  chSysUnlockFromISR();

}

/* MPUThread:
 * Main thread that runs the control loop
 * Waits for new MPU Values, gets the current RC Command
 * calls PID, Mixer an sets the values to the motors/ESCs
 *
 */

static THD_WORKING_AREA(waThread2, 1024);
static THD_FUNCTION(Thread2, arg) {

  (void)arg;
  chRegSetThreadName("MPU_Thread");
  chBSemObjectInit(&MPUDataReady, TRUE); /* Semaphore initialization*/
  StartCalibration(); /* Gyro calibration */

  if (MPUInit() != 0) {  /* Gyro initialization */
    while (TRUE) {
      // Init failed so we stay here blinking red
      palSetLine(LINE_LED_RED);
      chThdSleepMilliseconds(200);
      palClearLine(LINE_LED_RED);
      chThdSleepMilliseconds(200);
    }
  }

  /* Enabling events on rising edge of the interrupt line.*/
  palSetLineCallback(LINE_MPU_INT, mpu_isr_cb, NULL);
  palEnableLineEvent(LINE_MPU_INT, PAL_EVENT_MODE_RISING_EDGE);

  while (TRUE) {
    do {
      chBSemWait(&MPUDataReady);    // Wait until new Gyro Data is available
    } while (GetMPUData() == FALSE);  // Read the data
    if (mpuCycles % 500 == 0) {
      palToggleLine(LINE_LED_BLUE);
    }
    if (mpuCycles > 1000) mpuCycles = 0;
    mpuCycles++;
  }
}

/*  "Period" callback and icuGetPeriod() f'n
 *   is really measuring the active high pulse width
 *   (what we need to decode PPM receiver signals)
 */
static void icu_period_cb(ICUDriver *icup) {

  PulseWidth = icuGetPeriodX(icup);
  if (PulseWidth > 3000) channel_select_counter = 0;
  else channel_select_counter++;

  if (channel_select_counter == 1)channel[CH_1] = PulseWidth;
  if (channel_select_counter == 2)channel[CH_2] = PulseWidth;
  if (channel_select_counter == 3)channel[CH_3] = PulseWidth;
  if (channel_select_counter == 4)channel[CH_4] = PulseWidth;
  if (channel_select_counter == 5)channel[CH_5] = PulseWidth;
  if (channel_select_counter == 6)channel[CH_6] = PulseWidth;
}

/*  Dont expect to ever hit the overflow
 *  but just in case .....
 */
static void icu_overflow_cb(ICUDriver *icup) {

  (void)icup;
  channel_select_counter = 0;
  for (int i = 0; i < 10; i++) {
    chprintf(CHP, "Rollover...rollover...let Benji come over!!!");
    chThdSleepMilliseconds(100);
  }
}

static ICUConfig icucfg = {
  ICU_INPUT_ACTIVE_HIGH,
  1000000,                 /* 1MHz ICU clock frequency = 1us per counter tick   */
  NULL,                    /* pulse_width_cb ... strangely not used             */
  icu_period_cb,
  icu_overflow_cb,
  ICU_CHANNEL_1,
  0U,                     /* DMA settings in DIER register ... not used         */
  0xFFFFU                 /* Max ARR (overflow) value possible on 16 bit timer  */
};

static void pwmpcb(PWMDriver *pwmp) {

  (void)pwmp;
  //palClearPad(GPIOD, GPIOD_LED5);
}

static void pwmc1cb(PWMDriver *pwmp) {

  (void)pwmp;
  //palSetPad(GPIOD, GPIOD_LED5);
}

static PWMConfig pwmcfg = {
  10000,                                    /* 10kHz PWM clock frequency.   */
  10000,                                    /* Initial PWM period 1S.       */
  pwmpcb,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, pwmc1cb},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0,
  0
};

/*   ICU_PWM Thread:
 *   Input capture of receiver signals and
 *   PWM generation of ESC driver signals
 */
static THD_WORKING_AREA(waThread3, 1024);
static THD_FUNCTION(Thread3, arg) {

  (void)arg;
  chRegSetThreadName("PWM-ICU_Thread");

  icuObjectInit(&ICUD4);
  icuStart(&ICUD4, &icucfg);
  icuStartCapture(&ICUD4);
  icuEnableNotifications(&ICUD4);
  pwmStart(&PWMD3, &pwmcfg);
  pwmEnablePeriodicNotification(&PWMD3);

    /*
     * Starts the PWM channel 0 using 75% duty cycle.
     */
  //pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 7500));

  while (TRUE) {
    chThdSleepMilliseconds(500);
    palToggleLine(LINE_LED_RED);
  }
}

/*
 *   EKF Thread:
 */
static THD_WORKING_AREA(waThread5, 1024);
static THD_FUNCTION(Thread5, arg) {

  (void)arg;
  chRegSetThreadName("EKF_Thread");

  chBSemObjectInit(&MPUDataReady, TRUE); /* Semaphore initialization*/
  StartCalibration(); /* Gyro calibration */

  if (MPUInit() != 0) {  /* Gyro initialization */
    while (TRUE) {
      // Init failed so we stay here blinking red
      palSetLine(LINE_LED_RED);
      chThdSleepMilliseconds(200);
      palClearLine(LINE_LED_RED);
      chThdSleepMilliseconds(200);
    }
  }

  /* Enabling events on rising edge of the interrupt line.*/
  palSetLineCallback(LINE_MPU_INT, mpu_isr_cb, NULL);
  palEnableLineEvent(LINE_MPU_INT, PAL_EVENT_MODE_RISING_EDGE);

  Kalman_init(&kalmanX);
  Kalman_init(&kalmanY);
  Kalman_config();

  while (TRUE) {
    chThdSleepMilliseconds(2);
    Kalman_update();
  }
}
/*   Stats/Log Thread:
 *   Debug / JLink output
 */
static THD_WORKING_AREA(waThread4, 1024);
static THD_FUNCTION(Thread4, arg) {

  (void)arg;
  chRegSetThreadName("Statistics");
  uint8_t start = 0;

  while (TRUE) {
    chprintf(CHP, "Roll: %.3d  Pitch: %.3d  Yaw: %.3d\tAccX: %.3d  AccY: %.3d  AccZ: %.3d\r\n",
                               GyroData[ROLL], GyroData[PITCH], GyroData[YAW], AccelData[ROLL], AccelData[PITCH], AccelData[YAW]);
    chThdSleepMilliseconds(250);
    /*
    //For starting the motors: throttle low and yaw left (step 1).
    if (channel[CH_3] < 1100 && channel[CH_4] < 1100) start = 1;
    //When yaw stick is back in the center position start the motors (step 2).
    if (start == 1 && channel[CH_3] < 1100 && channel[CH_4] > 1450) start = 2;
    //Stopping the motors: throttle low and yaw right.
    if (start == 2 && channel[CH_3] < 1100 && channel[CH_4] > 1900) start = 0;

    chprintf(CHP, "Start:%d", start);

    chprintf(CHP, "  Roll:");
    if ((int32_t)channel[CH_1] - 1480 < 0) chprintf(CHP, "<<<");
    else if (channel[CH_1] - 1520 > 0) chprintf(CHP, ">>>");
    else chprintf(CHP, "-+-");
    chprintf(CHP, "%.4d", channel[CH_1]);

    chprintf(CHP, "  Pitch:");
    if ((int32_t)channel[CH_2] - 1480 < 0) chprintf(CHP, "^^^");
    else if (channel[CH_2] - 1520 > 0) chprintf(CHP, "vvv");
    else chprintf(CHP, "-+-");
    chprintf(CHP, "%.4d", channel[CH_2]);

    chprintf(CHP, "  Throttle:");
    if ((int32_t)channel[CH_3] - 1480 < 0) chprintf(CHP, "vvv");
    else if (channel[CH_3] - 1520 > 0) chprintf(CHP, "^^^");
    else chprintf(CHP, "-+-");
    chprintf(CHP, "%.4d", channel[CH_3]);

    chprintf(CHP, "  Yaw:");
    if ((int32_t)channel[CH_4] - 1480 < 0) chprintf(CHP, "<<<");
    else if (channel[CH_4] - 1520 > 0) chprintf(CHP, ">>>");
    else chprintf(CHP, "-+-");
    chprintf(CHP, "%.4d", channel[CH_4]);

    chprintf(CHP, "  CH5:");
    chprintf(CHP, "%.4d", channel[CH_5]);

    chprintf(CHP, "  CH6:");
    chprintf(CHP, "%.4d\r\n", channel[CH_6]);
    */
    }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  InitHardware();

  // Creates the blinker thread: Lowest prio
  chThdCreateStatic(waThread1, sizeof(waThread1), LOWPRIO, Thread1, NULL);

  // Start MPU Thread: Middle prio
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO+1, Thread2, NULL);

  // Start PWM_ICU Thread: Highest prio
  //chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO+2, Thread3, NULL);

  // Start Stats Thread: Low prio
  chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO, Thread4, NULL);

  // Start EKF Thread: Low prio
  //chThdCreateStatic(waThread5, sizeof(waThread5), NORMALPRIO, Thread5, NULL);

  // Normal main() thread activity, it does nothing.
  while (TRUE)
    chThdSleep(TIME_INFINITE);
  return 0;

}

void InitHardware() {

  sdStart(&LPSD1, NULL);   // Activates the serial driver (LPUART1) using the driver default configuration.
  /*
   * SPI1, TIM4, TIM3 pins setup.
   */
  palSetPadMode(GPIOB, 3, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);    /* SPI1 SCK.     */
  palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);    /* SPI1 MISO.    */
  palSetPadMode(GPIOB, 5, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);    /* SPI1 MOSI.    */
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(2));                               /* TIM4 CH1 ICU  */
  palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(2));                               /* TIM3 CH1 PWM  */
  palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(2));                               /* TIM3 CH2 PWM  */
  palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(2));                               /* TIM3 CH3 PWM  */
  palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(2));                               /* TIM3 CH4 PWM  */
  palSetPadMode(GPIOA, 4, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST); /* SPI1/MPU CS   */
  palSetPad(GPIOA, 4);
  palSetLineMode(LINE_MPU_INT, PAL_MODE_INPUT);                                 /* MPU INTERUPT  */

  palClearLine(LINE_LED_RED);
  palClearLine(LINE_LED_GREEN);
  palClearLine(LINE_LED_BLUE);
}

RTCDateTime timespec;
float timer;

void Kalman_config(void) {

  do {
    chBSemWait(&MPUDataReady);    // Wait until new Gyro Data is available
  } while (GetMPUData() == FALSE);  // Read the data
      // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  float roll  = atan2(AccelData[PITCH], AccelData[YAW]) * RAD_TO_DEG;
  float pitch = atan(-AccelData[ROLL] / sqrt(AccelData[PITCH] * AccelData[PITCH] + AccelData[YAW] * AccelData[YAW])) * RAD_TO_DEG;

  kalmanX.angle = roll; // Set starting angle
  kalmanY.angle = pitch;
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  rtcGetTime(&RTCD1, &timespec);
  timer = timespec.millisecond;
}

void Kalman_update(void) {

  /* Update all the values */
  chBSemWait(&MPUDataReady);
  GetMPUData();
  rtcGetTime(&RTCD1, &timespec);
  float dt = (timespec.millisecond - timer) / 1000;
  timer = timespec.millisecond;
  chprintf(CHP, "dt = %f", dt);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  uint16_t roll  = atan2(AccelData[PITCH], AccelData[YAW]) * RAD_TO_DEG;
  uint16_t pitch = atan(-AccelData[ROLL] / sqrt(AccelData[PITCH] * AccelData[PITCH] + AccelData[YAW] * AccelData[YAW])) * RAD_TO_DEG;


  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.angle = roll;
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = Kalman_getAngle(&kalmanX, roll, GyroData[ROLL], dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    GyroData[PITCH] = -GyroData[PITCH]; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = Kalman_getAngle(&kalmanY, pitch, GyroData[PITCH], dt);

  gyroXangle += GyroData[ROLL] * dt; // Calculate gyro angle without any filter
  gyroYangle += GyroData[PITCH] * dt;

  compAngleX = 0.93 * (compAngleX + GyroData[ROLL] * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + GyroData[PITCH] * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
  chprintf(CHP, "AccX: %d\t", AccelData[ROLL]);
  chprintf(CHP, "AccY: %d\t", AccelData[PITCH]);
  chprintf(CHP, "AccZ: %d\t\t", AccelData[YAW]);

  chprintf(CHP, "GyroX: %d\t", GyroData[ROLL]);
  chprintf(CHP, "GyroY: %d\t", GyroData[PITCH]);
  chprintf(CHP, "GyroZ: %d\r\n", GyroData[YAW]);

  chprintf(CHP, "K Roll: %f\t", roll);
  chprintf(CHP, "gyrXdeg: %f\t", gyroXangle);
  chprintf(CHP, "cmpXdeg: %f\t", compAngleX);
  chprintf(CHP, "kalXdeg: %f\r\n", kalAngleX);

  chprintf(CHP, "K Pitch: %f\t", pitch);
  chprintf(CHP, "gyrYdeg: %f\t", gyroYangle);
  chprintf(CHP, "cmpYdeg: %f\t", compAngleY);
  chprintf(CHP, "kalYdeg: %f\r\n", kalAngleY);

}

