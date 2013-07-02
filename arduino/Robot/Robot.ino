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
#define REF_ANGLE_SAMPLES 500

unsigned long curMicro = 0, lastMicro = 0, prevMicro = 0, rate = 0;

int pot = 0, spd = 0, gyroRate = 0, resPos = 0, acc = 0;
float kg = 0.0, ka = 0.0, angle = 0, refAngle = 0;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(115200); // Fastest Transfer rate?
  Wire.begin();         

  Cfilterbegin();
  SBRServoBegin(SERVO1_PIN, SERVO2_PIN, lastMicro);
  Serial.print("Computing Refrence angle...:"); 
  for (int i = 0; i < REF_ANGLE_SAMPLES; i++){
    getAccAngle();
    refAngle += absMax(theta,psi);
    delay(10);
  }
  refAngle /= REF_ANGLE_SAMPLES;
  Serial.println(refAngle);
  digitalWrite(LED_PIN, LOW);
}

void loop() 
{
  curMicro = micros();

  if ((curMicro - lastMicro) > SAMPLE_RATE){
    getAccAngle(); // Read the devices
    getGyroValues();

    rate = (curMicro - prevMicro) / 100000; // Calculate the actual rate in seconds (Gryo read outs are in seconds).

    gyroRate = absMax(gy,absMax(gx,gz)); //Since we can react to only one direction, it is assume to be maximal
    acc = absMax(theta,psi);

    angle = compositeFilter( acc, gyroRate, rate, angle); //angle tilt and move the clock up
    lastMicro = curMicro;  

    pot = analogRead(POT_PIN); // read the potentiometer

    kg =  map(pot, 0, 1023, 0, 100); // calculate the scaling coefficent
    ka = 0.05;

    spd = (kg * gyroRate) + (ka * (angle - refAngle)); // Compute the speed to set the acceleromter

    resPos = setSPD(-spd, curMicro); //set the Speed 

    if (resPos  != -1){ // Only print when we change something
      Serial.print(curMicro);
      Serial.print(" , ");
      Serial.print(acc);
      Serial.print(" , ");
      Serial.print(gyroRate);
      Serial.print(" , ");
      Serial.print(angle - refAngle); // remove the refrence from the display
      Serial.print(" , ");
      Serial.print(pot);
      Serial.print(" , ");
      Serial.print(kg);
      Serial.print(" , ");
      Serial.print(spd);
      Serial.print(" , ");
      Serial.println(resPos); 
    }
  }
  else if((curMicro - lastMicro) < 0){ // time variable wraped around 
    lastMicro = curMicro;
  }
  prevMicro = curMicro; // needed for loop length calculation
}
