/*
ADXL345 example Code using FREE IMU library 
 */

#include <Wire.h>
//#include <math.h>
#include "ADXL345.h"

#define DEBUG

#ifdef DEBUG
#define DEBUG_BEGIN(x)   Serial.begin(x); // Fastest Transfer rate?
#define DEBUG_PRINT(x)    Serial.print (x)
#define DEBUG_PRINTLN(x)  Serial.println (x)
#else
#define DEBUG_BEGIN(x)
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x) 
#endif

// Library types
ADXL345 Accel;


float  theta = 0, psi = 0, phi = 0;


void getAccAngle(){ //compute The actual theta and psi values from the Accelerometer readings
  float acc_data[3];
  Accel.get_Gxyz(acc_data);
  /* The correct calculation which uses the math library
   theta = atan(acc_data[0]/sqrt(acc_data[1]*acc_data[1] + acc_data[2]*acc_data[2]))*180/PI; 
   psi = atan(acc_data[1]/sqrt(acc_data[0]*acc_data[0] + acc_data[2]*acc_data[2]))*180/PI;
   phi=  atan(sqrt(acc_data[0]*acc_data[0] + acc_data[1]*acc_data[1])/acc_data[2])*180/PI; // Not needed since we only care bout x and y
   */
  theta = (acc_data[0]/sqrt(acc_data[1]*acc_data[1] + acc_data[2]*acc_data[2]))*180/PI; //drop atan for computation speed, small angle approximation
  psi = (acc_data[1]/sqrt(acc_data[0]*acc_data[0] + acc_data[2]*acc_data[2]))*180/PI;
}

void accBegin(){
  //Turning on the accelerometer
  Accel.init(ADXL345_ADDR_ALT_LOW);
  Accel.set_bw(ADXL345_BW_12);
}

void setup()
{
  accBegin(); 
  DEBUG_BEGIN(115200);
}

void loop() 
{
  getAccAngle();
  DEBUG_PRINT(millis());
  DEBUG_PRINT(" , ");
  DEBUG_PRINT(theta);
  DEBUG_PRINT(" , ");
  DEBUG_PRINTLN(psi);
}




