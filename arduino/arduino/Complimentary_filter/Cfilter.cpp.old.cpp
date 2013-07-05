

#include "Arduino.h"
#include "ADXL345.h"
#include <math.h>
#include <ITG3200.h>



ITG3200 gyro = ITG3200();
ADXL345 Accel;


int  gx = 0, gy = 0, gz = 0; //Compute Variables
float theta = 0, psi = 0, phi = 0, normAngle = 0, normACC = 0, alpha = 0, angle = 0; //Compute Variables



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

float compositeFilter(float acc, int gyro, unsigned long rate, float angle){
  return alpha * (angle + (gyro * float(rate)))  + (1 - alpha) * acc; // th eactual filter
}

