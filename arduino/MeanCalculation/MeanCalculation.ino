/*
* How many samples? 10 samples? N samples.
 * Average every 10 samples.
 *
 */

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

#define NUM_READINGS 100
#define DIMENSION  3
#define   GYR_Y                 0                              
#define   ACC_Z                 1                              
#define   ACC_Y                 2  

#define thetaIndex 0
#define psiIndex 1
#define phiIndex 2

ITG3200 gyro = ITG3200();
float Gyro_xyz[3];
float Acc_tpp[3];

ADXL345 Accel;

float readings[NUM_READINGS][DIMENSION];      // the readings from the analog input
int index = 0;                  // the index of the current reading
float total[DIMENSION];                  // the running total
float average[DIMENSION];

int actAngle;                                                  
int ACC_angle;
int GYRO_rate;



void getGyroValues(float *Gyro_xyz);
void getAccAngles(float *Acc_tpp);

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

  for (int ii = 0; ii < NUM_READINGS; ii++)
  {
    for (int nn = 0; nn < DIMENSION; nn++)
    {
      readings[ii][nn] = 0;
      total[nn] = 0;
    }
  }
}

void loop()
{
  getAccAngles(Acc_tpp); 
  getGyroValues(Gyro_xyz);  

  total[thetaIndex] = total[thetaIndex] - readings[index][thetaIndex];
  total[psiIndex] = total[psiIndex] - readings[index][psiIndex];
  total[phiIndex] = total[phiIndex]  - readings[index][phiIndex];


  // read from the sensor:  
  readings[index][thetaIndex] = Acc_tpp[thetaIndex];
  readings[index][psiIndex] = Acc_tpp[psiIndex];
  readings[index][phiIndex] = Acc_tpp[phiIndex];

  // add the reading to the total:
  total[thetaIndex] = total[thetaIndex] + readings[index][thetaIndex];
  total[psiIndex] = total[psiIndex] + readings[index][psiIndex];
  total[phiIndex] = total[phiIndex]  + readings[index][phiIndex];

  // advance to the next position in the array:  
  index = index + 1;                    

  // if we're at the end of the array...
  if (index >= NUM_READINGS)              
    // ...wrap around to the beginning: 
    index = 0;                           

  // calculate the average:
  average[thetaIndex] = total[thetaIndex] / NUM_READINGS;   
  average[psiIndex] = total[psiIndex] / NUM_READINGS;         
  average[phiIndex] = total[phiIndex] / NUM_READINGS;         

  // send it to the computer as ASCII digits
  Serial.print(millis());
  Serial.print(",");
  Serial.print(average[thetaIndex]); 
  Serial.print(",");
  Serial.print(average[psiIndex]);  
  Serial.print(",");
  Serial.print( average[phiIndex]);   
  Serial.println("\r");



 // delay(1);        // delay in between reads for stability       

}

void getAccAngles(float *Acc_tpp) {
  float theta;
  float psi;
  float phi;

  float acc_data[3];
  Accel.get_Gxyz(acc_data);

  theta = atan(acc_data[0]/sqrt(acc_data[1]*acc_data[1] + acc_data[2]*acc_data[2]))*180/PI;
  psi = atan(acc_data[1]/sqrt(acc_data[0]*acc_data[0] + acc_data[2]*acc_data[2]))*180/PI;
  phi=  atan(sqrt(acc_data[0]*acc_data[0] + acc_data[1]*acc_data[1])/acc_data[2])*180/PI;
/*
  Serial.print(millis());
  Serial.print(",");
  Serial.print(theta);
  Serial.print(",");
  Serial.print(psi);
  Serial.print(",");
  Serial.print(phi);
  */
  Acc_tpp[0]=theta;
  Acc_tpp[1]=psi;
  Acc_tpp[2]=phi;

}

// ------------------ read gyroscope angles ---------------------

void getGyroValues(float *Gyro_xyz){

  gyro.readGyro(Gyro_xyz);

/*  Serial.print(",");
  Serial.print(Gyro_xyz[0]);
  Serial.print(",");
  Serial.print(Gyro_xyz[1]);
  Serial.print(",");
  Serial.print(Gyro_xyz[2]);
  Serial.println("\r");
  */

}


