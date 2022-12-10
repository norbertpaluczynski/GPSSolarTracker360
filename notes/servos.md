# Voltage
**4,8V ~ 6V**
### 4,8V
- Moment: 13 kg*cm (1,27 Nm)
- Prędkość: 0,17 s/60°

### 6V
- Moment: 15 kg*cm (1,47 Nm)
- Prędkość: 0,13 s/60°

# Cables
- red - power
- brown - ground
- orange - signal

# Library
https://www.arduino.cc/reference/en/libraries/servo/

```
write()
On a continuous rotation servo, this will set the speed of the servo (with 0 being full-speed in one direction, 180 being full speed in the other, and a value near 90 being no movement).
```

# Example
```
#include <Servo.h> 

Servo myservo;

void setup() 
{ 
  myservo.attach(9);
  myservo.write(90);  // set servo to mid-point
} 

void loop() {} 
```