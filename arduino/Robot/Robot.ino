/*
The actual robot code
 */

#include <Wire.h>
#include <math.h>
#include <ITG3200.h>
#include <Servo.h>
#include "ADXL345.h"
#include "SBRservo.h"
#include <Cfilter.h>

#define SERVO1_PIN 10
#define SERVO2_PIN 17
#define POT_PIN 18
#define LED_PIN 11

unsigned long curMicro = 0, lastMicro = 0, prevMicro = 0, rate = 0;

int pot = 0, spd = 0, gyroRate = 0, resPos = 0;
float kg = 0.0, ka = 0.0;

void setup()
{
  Serial.begin(115200); // Fastest Transfer rate?
  Wire.begin();         

  Cfilterbegin();
  SBRServoBegin(SERVO1_PIN, SERVO2_PIN, lastMicro);
}

void loop() 
{
  curMicro = micros();

  if ((curMicro - lastMicro) > SAMPLE_RATE){
    getAccAngle(); // Internal updates
    getGyroValues();

    rate = (curMicro - prevMicro) / 100000; // Calculate the actual rate in seconds (Gryo read outs are in seconds).
    gyroRate = max(gy,max(gx,gz));
    angle = compositeFilter( max(theta,psi), gyroRate, rate, angle); //compute the worst angle tilt
    lastMicro = curMicro; // move up the clock refrence 
    Serial.print(curMicro);
    Serial.print(" , ");
    Serial.print(angle);
    Serial.print(" , ");
    Serial.print(gyroRate);

    pot = analogRead(POT_PIN); // read the potentiometer

    Serial.print(" , ");
    Serial.print(pot);

    kg =  map(pot, 0, 1023, 0, 100); // calculate the scaling coefficent
    ka = 1 - kg;
    
    Serial.print(" , ");
    Serial.print(kg);
    
    spd = (kg * gyroRate) + (ka * angle); // Compute the speed to set the acceleromter
    
    Serial.print(" , ");
    Serial.print(spd);
    
    resPos = setSPD(-spd, curMicro); //set the Speed 
    Serial.print(" , ");
    Serial.println(resPos); 
  }
  else if((curMicro - lastMicro) < 0){ // time variable wraped around 
    lastMicro = curMicro;
  }

  prevMicro = curMicro; // needed for loop length calculation
}


