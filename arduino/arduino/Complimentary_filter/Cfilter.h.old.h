/*Cfilter.h - Servio Library for Self Balancing Robot
 Provides: SBRservo
 methods
 setAccel
 rotateDeg
 */


#ifndef SBRservo_h
#define SBRservo_h

#define TIME_CONSTANT 130000 // time constant in Micrseconds
#define SAMPLE_RATE   4000 // Assumed Sampled rate in microseconds
#define REPORT_RATE   2000 // Assumed Sampled rate in microseconds

#include "Arduino.h"
#include "ADXL345.h"
#include <math.h>
#include <ITG3200.h>

extern ITG3200 gyro;
extern ADXL345 Accel;


extern int  gx, gy, gz; //Compute Variables
extern float theta, psi, phi, normAngle, normACC, alpha, angle; //Compute Variables

void getAccAngle();
void getGyroValues();
float compositeFilter(float acc, int gyro, unsigned long rate, float angle);



#endif
