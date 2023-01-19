#include "GPS.h"
#include "Servo.h"
#include "Magnetometer.h"

float azimuthAngle;
float elevationAngle;
float yaw;

void initVariables() {
  azimuthAngle = 80;
  elevationAngle = 0;
}

void updateGps() {
  readGps();
  float partOfDay = getPartOfTheDay(gps.time.hour(), gps.time.minute(), gps.time.second());
  float daysSince1900 = getDaysSince1900(gps.date.year(), gps.date.month(), gps.date.day());
  elevationAngle = solarElevationAngle(1, gps.location.lat(), gps.location.lng(), partOfDay, daysSince1900);
  azimuthAngle = solarAzimuthAngle(1, gps.location.lat(), gps.location.lng(), partOfDay, daysSince1900);
}

void rotateServos() {
  setServoElevation(elevationAngle);

  yaw = mpu.getYaw() + 180;
  // Serial.print("YAW: ");    
  //Serial.println(yaw);    

  if (abs(azimuthAngle - yaw) > 25) {   
    setServoAzimuth(1);
  } else {
    // Serial.println("SERVO: Azimuth angle reached!");    
    setServoAzimuth(0);
  }  
}

void helicopterMode() {
  static uint32_t helicopter_prev_ms = millis();
  uint32_t current = millis();
  if (inRange(current, 0, 5000)) {
    setServoElevation(80);
    azimuthServo.write(120);
  } else if (inRange(current, helicopter_prev_ms + 5000, helicopter_prev_ms + 7000)) {
    setServoElevation(0);
    azimuthServo.write(80);
  } else if (inRange(current, helicopter_prev_ms + 7000, helicopter_prev_ms + 12000)) {
    setServoElevation(80);
    azimuthServo.write(40);
  } else if (inRange(current, helicopter_prev_ms + 12000, helicopter_prev_ms + 14000)) {
    setServoElevation(0);
    azimuthServo.write(80);
  } else if (current > helicopter_prev_ms + 14000){
    helicopter_prev_ms = millis();
  }
}

void refreshMagnetometer() {
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 50) {
      rotateServos();
      prev_ms = millis();      
    }
  }
}

void printAngles() {
  Serial.print(F("CALCULATED SUN ANGLES:  elevation: "));
  Serial.print(elevationAngle);
  Serial.print(F("   azimuth: "));
  Serial.println(azimuthAngle);
}