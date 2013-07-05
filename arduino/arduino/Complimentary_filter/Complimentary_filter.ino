/*
 *
 * Compute the composite filter angle and gyro rates.
 * theta, psi, phi and gx,gy,gz
 *
 */
#include <Wire.h>
#include <Arduino.h>
#include <ADXL345.h>
#include <math.h>
#include <ITG3200.h>
#include <Cfilter.h>

unsigned long curMicro = 0, lastMicro = 0, prevMicro = 0, rate = 0;




//------------- Begin Aurdino doing stuff ---------------
void setup()
{
  Serial.begin(115200); // Fastest Transfer rate?
  Wire.begin();         
 
  Cfilterbegin();
  
  lastMicro = micros();
  prevMicro = lastMicro;
}



void loop()
{
  curMicro = micros();
  getAccAngle(); // Internal updates
  getGyroValues(); 

  if ((curMicro - lastMicro) > SAMPLE_RATE){
    rate = (curMicro - prevMicro) / 100000; // Calculate the actual rate in seconds (Gryo read outs are in seconds).
    angle = compositeFilter( max(theta,psi),max(gy,max(gx,gz)), rate, angle); //compute the worst angle tilt
    lastMicro = curMicro;
    Serial.print(curMicro);
    Serial.print(" , ");
    Serial.print(angle);
    Serial.print(" , ");
    Serial.println(max(gy,max(gx,gz)));

  }
  else if((curMicro - lastMicro) < 0){
    lastMicro = curMicro;
  }
  prevMicro = curMicro;
}







