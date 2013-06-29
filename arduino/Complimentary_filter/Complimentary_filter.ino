/*
 *
 * Compute the composite filter angle and gyro rates.
 * theta, psi, phi and gx,gy,gz
 *
 */
#include <Wire.h>
#include <math.h>
#include "ADXL345.h"
#include <ITG3200.h>

#define TIME_CONSTANT 130000 // time constant in Micrseconds
#define SAMPLE_RATE   4000 // Assumed Sampled rate in microseconds
#define REPORT_RATE   2000 // Assumed Sampled rate in microseconds

ITG3200 gyro = ITG3200();
ADXL345 Accel;


int  gx = 0, gy = 0, gz = 0; //Compute Variables
float theta = 0, psi = 0, phi = 0, normAngle = 0, normACC = 0, alpha = 0, angle = 0; //Compute Variables
unsigned long curMicro = 0, lastMicro = 0, prevMicro = 0, rate = 0;

// ------------------ read Accelerometer angles ---------------------

void getAccAngle() {

  float acc_data[3];
  Accel.get_Gxyz(acc_data);
  /* The correct calculation
   theta = atan(acc_data[0]/sqrt(acc_data[1]*acc_data[1] + acc_data[2]*acc_data[2]))*180/PI; 
   psi = atan(acc_data[1]/sqrt(acc_data[0]*acc_data[0] + acc_data[2]*acc_data[2]))*180/PI;
   phi=  atan(sqrt(acc_data[0]*acc_data[0] + acc_data[1]*acc_data[1])/acc_data[2])*180/PI; // Not needed since we only care bout x and y
   */
  theta = (acc_data[0]/sqrt(acc_data[1]*acc_data[1] + acc_data[2]*acc_data[2]))*180/PI; //drop atan for computation speed, small angle aproximation
  psi = (acc_data[1]/sqrt(acc_data[0]*acc_data[0] + acc_data[2]*acc_data[2]))*180/PI;
}
// ------------------ read gyroscope angles ---------------------

void getGyroValues(){
  float xyz[3];
  gyro.readGyro(xyz);
  gx = xyz[0];
  gy = xyz[1];
  gz = xyz[2];
}

//------------------ Composite Filter ----------------

float compositeFilter(float acc, int gyro, float angle){
  rate = (curMicro - prevMicro) / 100000; // Calculate the actual rate in seconds (Gryo read outs are in seconds).
  return alpha * (angle + (gyro * float(rate)))  + (1 - alpha) * acc; // th eactual filter
}

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
    angle = compositeFilter( max(theta,psi),max(gy,max(gx,gz)),angle); //compute the worst angle tilt
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







