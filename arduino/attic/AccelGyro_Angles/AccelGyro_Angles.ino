/*
*
* Generate Accelerometer, and Gyroscop Angles
* theta, psi, phi and gx,gy,gz
*
*/

#include <Wire.h>
#include <math.h>
#include "ADXL345.h"
#include <ITG3200.h>

ITG3200 gyro = ITG3200();
float xyz[3], temperature;
ADXL345 Accel;


#define   GYR_Y                 0                              
#define   ACC_Z                 1                              
#define   ACC_Y                 2  


int sensorValue[3]  = { 
  0, 0, 0};
int sensorZero[3]  = { 
  0, 0, 0 }; 

int actAngle;                                                  
int ACC_angle;
int GYRO_rate;

void getGyroRate();
void getAccAngle();

void setup()
{
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

  //setupgyroscope(2000); // Configure to  - 250, 500 or 2000 deg/sec  
  delay(100);    

}

void loop()
{
  getAccAngle(); 
  getGyroValues();  

}

void getAccAngle() {
  float theta;
  float psi;
  float phi;

  float acc_data[3];
  Accel.get_Gxyz(acc_data);

  theta = atan(acc_data[0]/sqrt(acc_data[1]*acc_data[1] + acc_data[2]*acc_data[2]))*180/PI;
  psi = atan(acc_data[1]/sqrt(acc_data[0]*acc_data[0] + acc_data[2]*acc_data[2]))*180/PI;
  phi=  atan(sqrt(acc_data[0]*acc_data[0] + acc_data[1]*acc_data[1])/acc_data[2])*180/PI;

  //Serial.print("Theta: ");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(theta);
  Serial.print(",");
  Serial.print(psi);
  Serial.print(",");
  Serial.print(phi);

 // return psi;
}

// ------------------ read gyroscope angles ---------------------

void getGyroValues(){
  int gx, gy, gz;
  gyro.readGyro(xyz);
  gx = xyz[0];
  gy = xyz[1];
  gz = xyz[2];

  // Serial.print(" gx: ");
  Serial.print(",");
  Serial.print(gx);
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.print(gz);
  Serial.println("\r");
  
 // return gx;
}

