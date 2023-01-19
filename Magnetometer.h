#include <MPU9250.h>

#define YELLOW_DIODE 6
MPU9250 mpu;

void blinkDiode(int count) {
  for (int i = 0; i < count; i++) {    
    digitalWrite(YELLOW_DIODE, HIGH);
    delay(500); 
    digitalWrite(YELLOW_DIODE, LOW);
    delay(500);
  }
}

void printCalibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}

void initMagnetometer() {
  Wire.begin();
  delay(2000);  
  mpu.setup(0x68);

  Serial.println(F("Accel Gyro calibration will start in 5sec."));
  Serial.println(F("Please leave the device still on the flat plane."));  
  mpu.verbose(true);
  
  blinkDiode(5);
  mpu.calibrateAccelGyro();
  
  Serial.println(F("Mag calibration will start in 10sec."));
  Serial.println(F("Please Wave device in a figure eight until done."));
  blinkDiode(10);
  mpu.calibrateMag();
  printCalibration();
  
  digitalWrite(YELLOW_DIODE, HIGH);
  // mpu.verbose(false);
}

void printMagnetometer() {
  Serial.print(F("MAGNETOMETER ANGLES:  yaw: "));
  Serial.print(mpu.getYaw()); 
  Serial.print(F("   pitch: "));
  Serial.print(mpu.getPitch()); 
  Serial.print(F("   roll: "));
  Serial.println(mpu.getRoll());
}

