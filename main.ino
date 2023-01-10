#include <math.h>
#include <MPU9255.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// SERVO
#define AZIMUTH_SERVO 9
#define ELEVATION_SERVO 10
Servo azimuthServo;
Servo elevationServo;

// COMPASS
#define g 9.81 // 1g ~ 9.81 m/s^2
#define magnetometer_cal 0.06 //magnetometer calibration
MPU9255 mpu;

// GPS
static const uint32_t RXPin = 3;
static const uint32_t TXPin = 4;
static const uint32_t GPSBaudRate = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// DIODES
#define YELLOW_DIODE 6
#define GREEN_DIODE 7
#define BLUE_DIODE 8

// BUTTONS
#define DARK_MODE 11
#define REFRESH_MODE 12
#define HELICOPTER_MODE 13

// GLOBAL VARIABLES
bool isCompassOk;
bool isGpsOk;

void setup() {
  Serial.begin(9600);
  initDiodes();
  initCompass();
  initServos();
}

void loop() {
  digitalWrite(BLUE_DIODE, LOW);
  
  // ss.begin(GPSBaudRate);
  if (!isGpsReady()) return;
  // ss.end();
  digitalWrite(BLUE_DIODE, HIGH);

  double partOfDay = 0.5;//getPartOfTheDay(gps.time.hour(), gps.time.minute(), gps.time.second());
  double daysSince1900 = getDaysSince1900(gps.date.year(), gps.date.month(), gps.date.day());
  double elevationAngle = solarElevationAngle(1, gps.location.lat(), gps.location.lng(), partOfDay, daysSince1900);
  double azimuthAngle = solarAzimuthAngle(1, gps.location.lat(), gps.location.lng(), partOfDay, daysSince1900);
  
  Serial.print("Elevation: ");
  Serial.println(elevationAngle);
  Serial.print("Azimuth: ");
  Serial.println(azimuthAngle);

  setServoElevation(80);
  
  delay(1000);
}

//======================================================
//============= Initialiation methods ==================
//======================================================
void initDiodes() {
  pinMode(YELLOW_DIODE, OUTPUT);
  pinMode(GREEN_DIODE, OUTPUT);
  pinMode(BLUE_DIODE, OUTPUT);
  
  digitalWrite(GREEN_DIODE, HIGH);
}

void initCompass() {
  isCompassOk = !mpu.init();  
  digitalWrite(YELLOW_DIODE, isCompassOk);
}

void initServos() {  
  azimuthServo.attach(AZIMUTH_SERVO, 600, 2400);
  azimuthServo.write(80);
  elevationServo.attach(ELEVATION_SERVO, 600, 2400);
  setServoElevation(0);
}

//======================================================
//=================== GPS methods ======================
//======================================================
bool isGpsReady() {
  if(ss.available() == 0) return false;
  gps.encode(ss.read());
  return gps.location.isUpdated();
}

double solarElevationAngle(double timeZone, double latitude, double longtitude, double partOfDay, double daysSince1900)
{
	double D2 = daysSince1900;
	double E2 = partOfDay;
	double F2 = D2 + 2415018.5 + E2 - timeZone / 24;
	double G2 = (F2 - 2451545) / 36525;
	double I2 = fmod(280.46646 + G2 * (36000.76983 + G2 * 0.0003032), 360);
	double J2 = 357.52911 + G2 * (35999.05029 - 0.0001537*G2);
	double K2 = 0.016708634 - G2 * (0.000042037 + 0.0000001267*G2);
	double L2 = sin(radians(J2))*(1.914602 - G2 * (0.004817 + 0.000014*G2)) + sin(radians(2 * J2))*(0.019993 - 0.000101*G2) + sin(radians(3 * J2))*0.000289;
	double M2 = I2 + L2;
	double P2 = M2 - 0.00569 - 0.00478*sin(radians(125.04 - 1934.136*G2));
	double Q2 = 23 + (26 + ((21.448 - G2 * (46.815 + G2 * (0.00059 - G2 * 0.001813)))) / 60) / 60;
	double R2 = Q2 + 0.00256*cos(radians(125.04 - 1934.136*G2));
	double T2 = degrees(asin(sin(radians(R2))*sin(radians(P2))));
	double U2 = tan(radians(R2 / 2))*tan(radians(R2 / 2));
	double V2 = 4 * degrees(U2*sin(2 * radians(I2)) - 2 * K2*sin(radians(J2)) + 4 * K2*U2*sin(radians(J2))*cos(2 * radians(I2)) - 0.5*U2*U2*sin(4 * radians(I2)) - 1.25*K2*K2*sin(2 * radians(J2)));
	double AB2 = fmod(E2 * 1440 + V2 + 4 * longtitude - 60 * timeZone, 1440);
	double AC2 = 0;
	if (AB2 / 4 < 0) AC2 = AB2 / 4 + 180;
	else AC2 = AB2 / 4 - 180;
	double AD2 = degrees(acos(sin(radians(latitude))*sin(radians(T2)) + cos(radians(latitude))*cos(radians(T2))*cos(radians(AC2))));

	return 90 - AD2;
}

double solarAzimuthAngle(double timeZone, double latitude, double longtitude, double partOfDay, double daysSince1900)
{
	double D2 = daysSince1900;
	double E2 = partOfDay;
	double F2 = D2 + 2415018.5 + E2 - timeZone / 24;
	double G2 = (F2 - 2451545) / 36525;
	double I2 = fmod(280.46646 + G2 * (36000.76983 + G2 * 0.0003032), 360);
	double J2 = 357.52911 + G2 * (35999.05029 - 0.0001537*G2);
	double K2 = 0.016708634 - G2 * (0.000042037 + 0.0000001267*G2);
	double L2 = sin(radians(J2))*(1.914602 - G2 * (0.004817 + 0.000014*G2)) + sin(radians(2 * J2))*(0.019993 - 0.000101*G2) + sin(radians(3 * J2))*0.000289;
	double M2 = I2 + L2;
	double P2 = M2 - 0.00569 - 0.00478*sin(radians(125.04 - 1934.136*G2));
	double Q2 = 23 + (26 + ((21.448 - G2 * (46.815 + G2 * (0.00059 - G2 * 0.001813)))) / 60) / 60;
	double R2 = Q2 + 0.00256*cos(radians(125.04 - 1934.136*G2));
	double T2 = degrees(asin(sin(radians(R2))*sin(radians(P2))));
	double U2 = tan(radians(R2 / 2))*tan(radians(R2 / 2));
	double V2 = 4 * degrees(U2*sin(2 * radians(I2)) - 2 * K2*sin(radians(J2)) + 4 * K2*U2*sin(radians(J2))*cos(2 * radians(I2)) - 0.5*U2*U2*sin(4 * radians(I2)) - 1.25*K2*K2*sin(2 * radians(J2)));

	double AB2 = fmod(E2 * 1440 + V2 + 4 * longtitude - 60 * timeZone, 1440);
	double AC2 = 0;
	if (AB2 / 4 < 0) AC2 = AB2 / 4 + 180;
	else AC2 = AB2 / 4 - 180;
	double AD2 = degrees(acos(sin(radians(latitude))*sin(radians(T2)) + cos(radians(latitude))*cos(radians(T2))*cos(radians(AC2))));
	double AH2;
	if(AC2 > 0)
		return (double)((long)(degrees(acos(((sin(radians(latitude))*cos(radians(AD2))) - sin(radians(T2))) / (cos(radians(latitude))*sin(radians(AD2))))) + 180) % 360);
	else return (double)((long)(540 - degrees(acos(((sin(radians(latitude))*cos(radians(AD2))) - sin(radians(T2))) / (cos(radians(latitude))*sin(radians(AD2)))))) % 360);
}

double getPartOfTheDay(double hour, double minute, double second)
{
	return (hour*3600+minute*60+second)/86400;
}

uint32_t getDaysSince1900(uint32_t year, uint32_t month, uint32_t day)
{
	uint32_t y = year - 1900;
	uint32_t days = y * 365 + y / 4;
  
	switch (month)
	{
    case 12: days += 30;
    case 11: days += 31;
    case 10: days += 30;
    case 9: days += 31;
    case 8: days += 31;
    case 7: days += 30;
    case 6: days += 31;
    case 5: days += 30;
    case 4: days += 31;
    case 3: days += 28;
    case 2: days += 31;    
	}
	days += day;

	return days - 1;
}

void printGPS() //TODO: remove when unnecessary
{ 
  Serial.print("Latitude= "); 
  Serial.print(gps.location.lat(), 6);
  Serial.print(" Longitude= "); 
  Serial.println(gps.location.lng(), 6);

  // Raw date in DDMMYY format (u32)
  Serial.print("Raw date DDMMYY = ");
  Serial.println(gps.date.value()); 

  // Year (2000+) (u16)
  Serial.print("Year = "); 
  Serial.println(gps.date.year()); 
  // Month (1-12) (u8)
  Serial.print("Month = "); 
  Serial.println(gps.date.month()); 
  // Day (1-31) (u8)
  Serial.print("Day = "); 
  Serial.println(gps.date.day()); 


  // Raw time in HHMMSSCC format (u32)
  Serial.print("Raw time in HHMMSSCC = "); 
  Serial.println(gps.time.value()); 

  // Hour (0-23) (u8)
  Serial.print("Hour = "); 
  Serial.println(gps.time.hour()); 
  // Minute (0-59) (u8)
  Serial.print("Minute = "); 
  Serial.println(gps.time.minute()); 
}

//======================================================
//================== Servo methods =====================
//======================================================
void setServoElevation(int angle) //0* - 85*
{
	int safeAngle;

	if(angle < 0)
	{
		safeAngle = 90;
	}
	else if(angle > 80)
	{
		safeAngle = 170;
	}
	else
	{
		safeAngle = 90 + angle;
	}
  elevationServo.write(safeAngle);
}

//======================================================
//================= Compass methods ====================
//======================================================

//process raw acceleration data
//input = raw reading from the sensor, sensor_scale = selected sensor scale
//returns : acceleration in m/s^2
double process_acceleration(int input, scales sensor_scale )
{
  /*
  To get acceleration in 'g', each reading has to be divided by :
   -> 16384 for +- 2g scale (default scale)
   -> 8192  for +- 4g scale
   -> 4096  for +- 8g scale
   -> 2048  for +- 16g scale
  */
  double output = 1;

  //for +- 2g

  if(sensor_scale == scale_2g)
  {
    output = input;
    output = output/16384;
    output = output*g;
  }

  //for +- 4g
  if(sensor_scale == scale_4g)
  {
    output = input;
    output = output/8192;
    output = output*g;
  }

  //for +- 8g
  if(sensor_scale == scale_8g)
  {
    output = input;
    output = output/4096;
    output = output*g;
  }

  //for +-16g
  if(sensor_scale == scale_16g)
  {
    output = input;
    output = output/2048;
    output = output*g;
  }

  return output;
}

//process raw gyroscope data
//input = raw reading from the sensor, sensor_scale = selected sensor scale
//returns : angular velocity in degrees per second
double process_angular_velocity(int16_t input, scales sensor_scale )
{
  /*
  To get rotation velocity in dps (degrees per second), each reading has to be divided by :
   -> 131   for +- 250  dps scale (default value)
   -> 65.5  for +- 500  dps scale
   -> 32.8  for +- 1000 dps scale
   -> 16.4  for +- 2000 dps scale
  */

  //for +- 250 dps
  if(sensor_scale == scale_250dps)
  {
    return input/131;
  }

  //for +- 500 dps
  if(sensor_scale == scale_500dps)
  {
    return input/65.5;
  }

  //for +- 1000 dps
  if(sensor_scale == scale_1000dps)
  {
    return input/32.8;
  }

  //for +- 2000 dps
  if(sensor_scale == scale_2000dps)
  {
    return input/16.4;
  }

  return 0;
}

//process raw magnetometer data
//input = raw reading from the sensor, sensitivity =
//returns : magnetic flux density in μT (in micro Teslas)
double process_magnetic_flux(int16_t input, double sensitivity)
{
  /*
  To get magnetic flux density in μT, each reading has to be multiplied by sensitivity
  (Constant value different for each axis, stored in ROM), then multiplied by some number (calibration)
  and then divided by 0.6 .
  (Faced North each axis should output around 31 µT without any metal / walls around
  Note : This manetometer has really low initial calibration tolerance : +- 500 LSB !!!
  Scale of the magnetometer is fixed -> +- 4800 μT.
  */
  return (input*magnetometer_cal*sensitivity)/0.6;
}


void compass() {
  //take readings
  mpu.read_acc();
  mpu.read_gyro();
  mpu.read_mag();

  ////process and print acceleration data////
  //X axis
  Serial.print("AX: ");
  Serial.print(process_acceleration(mpu.ax,scale_2g));

  //Y axis
  Serial.print("  AY: ");
  Serial.print(process_acceleration(mpu.ay,scale_2g));

  //Z axis
  Serial.print("  AZ: ");
  Serial.print(process_acceleration(mpu.az,scale_2g));


  ////process and print gyroscope data////
  //X axis
  Serial.print("      GX: ");
  Serial.print(process_angular_velocity(mpu.gx,scale_250dps));

  //Y axis
  Serial.print("  GY: ");
  Serial.print(process_angular_velocity(mpu.gy,scale_250dps));

  //Z axis
  Serial.print("  GZ: ");
  Serial.print(process_angular_velocity(mpu.gz,scale_250dps));


  ////process and print magnetometer data////
  //X axis
  Serial.print("      MX: ");
  Serial.print(process_magnetic_flux(mpu.mx,mpu.mx_sensitivity));

  //Y axis
  Serial.print("  MY: ");
  Serial.print(process_magnetic_flux(mpu.my,mpu.my_sensitivity));

  //Z axis
  Serial.print("  MZ: ");
  Serial.println(process_magnetic_flux(mpu.mz,mpu.mz_sensitivity));

  Serial.println(atan2(process_magnetic_flux(mpu.my,mpu.my_sensitivity), process_magnetic_flux(mpu.mx,mpu.mx_sensitivity)) * 180 / M_PI);

  // if (counter < 10) 
  // {
  //   int value = atan2(process_magnetic_flux(mpu.my,mpu.my_sensitivity), process_magnetic_flux(mpu.mx,mpu.mx_sensitivity)) * 180 / M_PI;
  //   angles[counter] = value;
  // }  
  // else
  // {
  //   int sum = 0;
    
  //   for (int i = 0; i < 10; i++)
  //   {
  //     sum += angles[i];
  //   }    
  //   Serial.print("HEADING: ");
  //   Serial.println(sum / 10);    
  //   counter = 0;
  // }

  // counter++;

  delay(1000);
}
