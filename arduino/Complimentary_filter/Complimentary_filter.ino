/*
 *
 * Compute the composite filter angle and gyro rates.
 * theta, psi, phi and gx,gy,gz
 *
 */
#include <Wire.h>
#include "Arduino.h"
#include "ADXL345.h"
#include <math.h>
#include <ITG3200.h>
#include "Cfilter.h"

unsigned long curMicro = 0, lastMicro = 0, prevMicro = 0, rate = 0;




//------------- Begin Aurdino doing stuff ---------------
void setup()
{
  Serial.begin(115200); // Fastest Transfer rate?
  Wire.begin();         
  //Turning on the accelerometer
  Accel.init(ADXL345_ADDR_ALT_LOW);
  Accel.set_bw(ADXL345_BW_12);

  gyro.reset();
  // Use ITG3200_ADDR_AD0_HIGH or ITG3200_ADDR_AD0_LOW as the ITG3200 address 
  // depending on how AD0 is connected on your breakout board, check its schematics for details
  gyro.init(ITG3200_ADDR_AD0_LOW);	
  Serial.print("zeroCalibrating...");
  gyro.zeroCalibrate(2500,2);
  Serial.println("done.");

  alpha = float(TIME_CONSTANT) / (float(TIME_CONSTANT) + float(SAMPLE_RATE)); // calculate the scaling coefficent
  Serial.print("Scaling Coefficent: ");
  Serial.println(alpha);  

  delay(100);
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







