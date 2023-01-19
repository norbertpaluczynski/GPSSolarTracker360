#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define BLUE_DIODE 8

static const uint8_t RXPin = 4;
static const uint8_t TXPin = 5;
static const uint16_t GPSBaudRate = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);


bool readGps() {
  ss.begin(GPSBaudRate);
  Serial.println(F("Reading data from GPS..."));
  digitalWrite(BLUE_DIODE, HIGH);

  while (ss.available() > 0) {
    gps.encode(ss.read());    
    
    if (gps.location.isUpdated()) {
      Serial.println(F("GPS data updated..."));      
      digitalWrite(BLUE_DIODE, LOW);
      ss.end();

      return true;
    }
  }    
  ss.end();
}

float solarElevationAngle(float timeZone, float latitude, float longtitude, float partOfDay, float daysSince1900)
{
	float D2 = daysSince1900;
	float E2 = partOfDay;
	float F2 = D2 + 2415018.5 + E2 - timeZone / 24;
	float G2 = (F2 - 2451545) / 36525;
	float I2 = fmod(280.46646 + G2 * (36000.76983 + G2 * 0.0003032), 360);
	float J2 = 357.52911 + G2 * (35999.05029 - 0.0001537*G2);
	float K2 = 0.016708634 - G2 * (0.000042037 + 0.0000001267*G2);
	float L2 = sin(radians(J2))*(1.914602 - G2 * (0.004817 + 0.000014*G2)) + sin(radians(2 * J2))*(0.019993 - 0.000101*G2) + sin(radians(3 * J2))*0.000289;
	float M2 = I2 + L2;
	float P2 = M2 - 0.00569 - 0.00478*sin(radians(125.04 - 1934.136*G2));
	float Q2 = 23 + (26 + ((21.448 - G2 * (46.815 + G2 * (0.00059 - G2 * 0.001813)))) / 60) / 60;
	float R2 = Q2 + 0.00256*cos(radians(125.04 - 1934.136*G2));
	float T2 = degrees(asin(sin(radians(R2))*sin(radians(P2))));
	float U2 = tan(radians(R2 / 2))*tan(radians(R2 / 2));
	float V2 = 4 * degrees(U2*sin(2 * radians(I2)) - 2 * K2*sin(radians(J2)) + 4 * K2*U2*sin(radians(J2))*cos(2 * radians(I2)) - 0.5*U2*U2*sin(4 * radians(I2)) - 1.25*K2*K2*sin(2 * radians(J2)));
	float AB2 = fmod(E2 * 1440 + V2 + 4 * longtitude - 60 * timeZone, 1440);
	float AC2 = 0;
	if (AB2 / 4 < 0) AC2 = AB2 / 4 + 180;
	else AC2 = AB2 / 4 - 180;
	float AD2 = degrees(acos(sin(radians(latitude))*sin(radians(T2)) + cos(radians(latitude))*cos(radians(T2))*cos(radians(AC2))));

	return 90 - AD2;
}

float solarAzimuthAngle(float timeZone, float latitude, float longtitude, float partOfDay, float daysSince1900)
{
	float D2 = daysSince1900;
	float E2 = partOfDay;
	float F2 = D2 + 2415018.5 + E2 - timeZone / 24;
	float G2 = (F2 - 2451545) / 36525;
	float I2 = fmod(280.46646 + G2 * (36000.76983 + G2 * 0.0003032), 360);
	float J2 = 357.52911 + G2 * (35999.05029 - 0.0001537*G2);
	float K2 = 0.016708634 - G2 * (0.000042037 + 0.0000001267*G2);
	float L2 = sin(radians(J2))*(1.914602 - G2 * (0.004817 + 0.000014*G2)) + sin(radians(2 * J2))*(0.019993 - 0.000101*G2) + sin(radians(3 * J2))*0.000289;
	float M2 = I2 + L2;
	float P2 = M2 - 0.00569 - 0.00478*sin(radians(125.04 - 1934.136*G2));
	float Q2 = 23 + (26 + ((21.448 - G2 * (46.815 + G2 * (0.00059 - G2 * 0.001813)))) / 60) / 60;
	float R2 = Q2 + 0.00256*cos(radians(125.04 - 1934.136*G2));
	float T2 = degrees(asin(sin(radians(R2))*sin(radians(P2))));
	float U2 = tan(radians(R2 / 2))*tan(radians(R2 / 2));
	float V2 = 4 * degrees(U2*sin(2 * radians(I2)) - 2 * K2*sin(radians(J2)) + 4 * K2*U2*sin(radians(J2))*cos(2 * radians(I2)) - 0.5*U2*U2*sin(4 * radians(I2)) - 1.25*K2*K2*sin(2 * radians(J2)));
	float AB2 = fmod(E2 * 1440 + V2 + 4 * longtitude - 60 * timeZone, 1440);
	float AC2 = 0;
	if (AB2 / 4 < 0) AC2 = AB2 / 4 + 180;
	else AC2 = AB2 / 4 - 180;
	float AD2 = degrees(acos(sin(radians(latitude))*sin(radians(T2)) + cos(radians(latitude))*cos(radians(T2))*cos(radians(AC2))));
	float AH2;
	if(AC2 > 0)
		return (float)((long)(degrees(acos(((sin(radians(latitude))*cos(radians(AD2))) - sin(radians(T2))) / (cos(radians(latitude))*sin(radians(AD2))))) + 180) % 360);
	else return (float)((long)(540 - degrees(acos(((sin(radians(latitude))*cos(radians(AD2))) - sin(radians(T2))) / (cos(radians(latitude))*sin(radians(AD2)))))) % 360);
}

float getPartOfTheDay(float hour, float minute, float second)
{
	return (hour*3600+minute*60+second)/86400;
}

uint16_t getDaysSince1900(uint16_t year, uint16_t month, uint16_t day)
{
	uint16_t y = year - 1900;
	uint16_t days = y * 365 + y / 4;
  
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
