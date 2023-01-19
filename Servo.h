#include <Servo.h>

#define AZIMUTH_SERVO 9
#define ELEVATION_SERVO 10
Servo azimuthServo;
Servo elevationServo;

/// Angle from range: 0* - 80*
void setServoElevation(int angle)
{
	int safeAngle = 90 + min(max(angle, 0), 80);
  elevationServo.write(safeAngle);
}

bool inRange(int val, int minimum, int maximum)
{
  return ((minimum <= val) && (val <= maximum));
}

/// Direction: -1 - rotate left, 1 - rotate right, 0 - stop
void setServoAzimuth(int direction)
{
  azimuthServo.write(80);
  static uint32_t servo_prev_ms = millis();
  uint32_t current = millis();
  if (inRange(current, servo_prev_ms + 50, servo_prev_ms + 100)) {
    int safeAngle = 80 + min(max(direction, -1), 1) * 40;
    azimuthServo.write(safeAngle);
  } else if (current > servo_prev_ms + 100) {
    azimuthServo.write(80);
    servo_prev_ms = millis();   
  }
}

void initServos() {  
  azimuthServo.attach(AZIMUTH_SERVO, 600, 2400);
  setServoAzimuth(0);
  elevationServo.attach(ELEVATION_SERVO, 600, 2400);
  setServoElevation(0);
}