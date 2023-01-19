#include <math.h>
#include "Timers.h"
#include "TimerHandlers.h"

#define GREEN_DIODE 7
#define DARK_MODE 11
#define STATIC_MODE 12
#define HELICOPTER_MODE 13
int mode = 11;

Timers<2> timers;

void setup() {
  Serial.begin(9600);

  initVariables();
  initDiodes();
  initButtons();
  initServos();
  initTimers();
  initMagnetometer();
  Serial.println(F("Initialization completed!"));
  updateGps();
}

void loop() {
  timers.process();
  
  switch(mode) {
    case DARK_MODE:
      refreshMagnetometer();
      timers.updateInterval(0, 10000);   
    break;
    case STATIC_MODE:
      azimuthAngle = 180;
      elevationAngle = 70;
      refreshMagnetometer();
      timers.updateInterval(0, 0);    
    break;
    case HELICOPTER_MODE:
      helicopterMode();
    break;
  }

  if (!digitalRead(DARK_MODE)) {
    mode = DARK_MODE;
  } else if (!digitalRead(STATIC_MODE)) {
    mode = STATIC_MODE;
  } else if (!digitalRead(HELICOPTER_MODE)) {
    mode = HELICOPTER_MODE;
  }  
}

void initTimers() {
  timers.attach(0, 10000, updateGps);
  timers.attach(1, 5000, printAngles);
}

void initDiodes() {
  pinMode(YELLOW_DIODE, OUTPUT);
  pinMode(GREEN_DIODE, OUTPUT);
  pinMode(BLUE_DIODE, OUTPUT);
  
  digitalWrite(GREEN_DIODE, HIGH);
}

void initButtons() {
  pinMode(DARK_MODE, INPUT_PULLUP);
  pinMode(STATIC_MODE, INPUT_PULLUP);
  pinMode(HELICOPTER_MODE, INPUT_PULLUP);
}

