//SBRServo movement library
//by James "Shridatt" Sugrim

#include <Servo.h>
#include "SBRservo.h"

#define SERVO1_PIN 17
#define SERVO2_PIN 18


int spd  = -300;


void setup() 
{ 
  Serial.begin(9600);
  SBRServoBegin(SERVO1_PIN, SERVO2_PIN);
} 


void loop() 
{ 
  if (spd > 300)
  {
    spd = -300;
  }
  Serial.print("SPD Value:");
  Serial.print(spd);
  spd += 10;
  pos = setSPD(spd);
  Serial.print(" Pos Value:");
  Serial.println(pos);
  delay(1000);  
} 



