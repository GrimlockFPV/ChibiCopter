
#include "ch.h"
#include "kalman.h"
#include "mpu9250.h"

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

time_measurement_t *tmp;



void Kalman_config(void) {

  GetMPUData();
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  float roll  = atan2(AccelData.pitch, AccelData.yaw) * RAD_TO_DEG;
  float pitch = atan(-AccelData.roll / sqrt(AccelData.pitch * AccelData.pitch + AccelData.yaw * AccelData.yaw)) * RAD_TO_DEG;

  kalmanX.angle(roll); // Set starting angle
  kalmanY.angle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  _tm_init();
  chTMObjectInit(&tmp);
  chTMStartMeasurementX(&tmp);
}

void Kalman_update(void) {

  /* Update all the values */
  GetMPUData();
  chTMStopMeasurementX(&tmp);

  float dt = &tmp.last; // Calculate delta time
  chTMStartMeasurementX(&tmp);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  float roll  = atan2(AccelData.pitch, AccelData.yaw) * RAD_TO_DEG;
  float pitch = atan(-AccelData.roll / sqrt(AccelData.pitch * AccelData.pitch + AccelData.yaw * AccelData.yaw)) * RAD_TO_DEG;


  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.angle = roll;
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, GyroData.roll, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    GyroData.pitch = -GyroData.pitch; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, GyroData.pitch, dt);

  gyroXangle += GyroData.roll * dt; // Calculate gyro angle without any filter
  gyroYangle += GyroData.pitch * dt;

  compAngleX = 0.93 * (compAngleX + GyroData.roll * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + GyroData.pitch * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
  chprintf(CHP, "AccX: %f\t", AccelData.roll);
  chprintf(CHP, "AccY: %f\t", AccelData.pitch);
  chprintf(CHP, "AccZ: %f\t\t", AccelData.yaw);

  chprintf(CHP, "GyroX: %f\t", GyroData.roll);
  chprintf(CHP, "GyroY: %f\t", GyroData.pitch);
  chprintf(CHP, "GyroZ: %f\r\n", GyroData.yaw);

  chprintf(CHP, "EKF Roll: %f\t", roll);
  chprintf(CHP, "gyroXangle: %f\t", gyroXangle);
  chprintf(CHP, "compXangle: %f\t", compXangle);
  chprintf(CHP, "kalXangle: %f\t\t", kalXangle);

  chprintf(CHP, "EKF Pitch: %f\t", pitch);
  chprintf(CHP, "gyroYangle: %f\t", gyroYangle);
  chprintf(CHP, "compYangle: %f\t", compYangle);
  chprintf(CHP, "kalYangle: %f\r\n", kalYangle);

}
