#include <Servo.h>
Servo Servo1;
Servo Servo2;

int servoPin1 = 9;
int servoPin2 = 7;
int porPin = A0;

void setup(){
  Servo1.attach(servoPin1);
  Servo2.attach(servoPin2);
}

void loop(){
  int reading1 = analogRead(porPin);
  int angle1 = map(reading1 , 0 , 511, 0 ,180 );
    Servo1.write(angle1);
  int reading2 = analogRead(porPin);
  int angle2 = map(reading2 , 551 , 1023, 0 ,180 );
  Servo2.write(angle2);
  
}
