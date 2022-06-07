/*
 * config.h
 *
 *  Created on: May 21, 2015
 *      Author: Pablo
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "lcd.h"


/* Display */
#define LINE_RS                     PAL_LINE(GPIOF, 12U)
#define LINE_E                      PAL_LINE(GPIOD, 15U)
#define LINE_A                      PAL_LINE(GPIOD, 14U)

#define LINE_D4                     PAL_LINE(GPIOF, 14U)
#define LINE_D5                     PAL_LINE(GPIOE, 11U)
#define LINE_D6                     PAL_LINE(GPIOE, 9U)
#define LINE_D7                     PAL_LINE(GPIOF, 13U)

#define LINE_MPU_INT       PAL_LINE(GPIOB, 12U)
#define LINE_LED_BLUE      PAL_LINE(GPIOB, 7U)
#define LINE_LED_RED       PAL_LINE(GPIOB, 14U)
#define CHP (BaseSequentialStream*) &LPSD1
#define RC_CHANNEL_NUM     6
#define RAD_TO_DEG         (180 / 3.14f)
enum { CH_1, CH_2, CH_3, CH_4, CH_5, CH_6 };

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

/*===========================================================================*/
/* LCD configuration                                                         */
/*===========================================================================*/


static const lcd_pins_t lcdpins = {
  LINE_RS,
  //LINE_RW,
  LINE_E,
  LINE_A,
  {
    LINE_D4,
    LINE_D5,
    LINE_D6,
    LINE_D7
  }
};

static const LCDConfig lcdcfg = {
  LCD_CURSOR_OFF,                           /* Cursor disabled               */
  LCD_BLINKING_OFF,                         /* Blinking disabled             */
  LCD_SET_FONT_5X10,                        /* Font 5x10                     */
  LCD_SET_2LINES,                           /* 2 lines                       */
  &lcdpins,                                 /* Pin map                       */
  100,                                      /* Back-light                    */
};



#endif /* CONFIG_H_ */
