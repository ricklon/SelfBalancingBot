/*
PID example
 */

#include <Wire.h>
//#include <math.h>
#include <ITG3200.h>
#include "ADXL345.h"

//setup stuff
#define DEBUG
#define LED_PIN 11

//Cfilter stuff
#define TIME_CONSTANT 250.0 // time constant in milli
#define SAMPLE_RATE   7.0 // Assumed Sampled rate in milli

//PID stuff
#define KP 0.5
#define KI 0.5
#define KD 0.5
#define REF_ANGLE_SAMPLES 1000 // Number of samples to calcualte the ref angle

//Debug stuff
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
ITG3200 gyro = ITG3200();
ADXL345 Accel;


unsigned long curMilli = 0;
unsigned long lastMilli = 0;
unsigned long spdMilli = 0;

int resPos = 0;
int gx = 0;
int gy = 0;
int gz = 0;
int pos = 0;
int rpos = 0;
int maxGyro = 0;

float spd = 0.0;
float angle = 0.0;
float refAngle = 0.0;
float prevError = 0.0;
float integral = 0.0;
float theta = 0.0;
float psi = 0.0;
float alpha = 0.0;
float derivative = 0.0;
float rate = 0.0;
float maxAngle = 0.0;

/*
---------------------- Composite Filter -------------------------------------------
 */

void Cfilterbegin(){
  //Turning on the accelerometer
  Accel.init(ADXL345_ADDR_ALT_LOW);
  Accel.set_bw(ADXL345_BW_12);

  gyro.reset();
  // Use ITG3200_ADDR_AD0_HIGH or ITG3200_ADDR_AD0_LOW as the ITG3200 address 
  // depending on how AD0 is connected on your breakout board, check its schematics for details
  gyro.init(ITG3200_ADDR_AD0_LOW);  
  DEBUG_PRINT("zeroCalibrating...");
  gyro.zeroCalibrate(2500,2);
  DEBUG_PRINTLN("done.");

  alpha = TIME_CONSTANT / (TIME_CONSTANT + SAMPLE_RATE); // calculate the scaling coefficent
  DEBUG_PRINT("Scaling Coefficent: ");
  DEBUG_PRINTLN(alpha); 

  delay(100); 
}

// ------------------ Convienence Function -------------------------

float absMax(float a, float b){ // returns the value that is further from center (zero)
  if (abs(a) > abs(b)){
    return a;
  }
  else {
    return b;
  }
}

// ------------------ read Accelerometer angles ---------------------

void getAccAngle() { //compute The actual theta and psi values from the Accelerometer readings

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

void getGyroValues(){ // Simple reading of gyro values
  float xyz[3];
  gyro.readGyro(xyz);
  gx = xyz[0];
  gy = xyz[1];
  gz = xyz[2];
}

//------------------ Composite Filter ----------------

float compositeFilter(float acc, float gyro, float rate, float angle){
  return alpha * (angle + (gyro * rate))  + (1 - alpha) * acc; // the actual filter
}

/*
-------------------------- Aurdino Code -------------------------------
 */

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  DEBUG_BEGIN(115200);
  Wire.begin();         

  Cfilterbegin();
  DEBUG_PRINT("Computing Refrence angle...:"); 
  for (int i = 0; i < REF_ANGLE_SAMPLES; i++){
    getAccAngle();
    getGyroValues();
    maxGyro = absMax(gy,absMax(gx,gz)); 
    maxAngle = absMax(theta,psi);
    angle = compositeFilter( maxAngle, float(maxGyro), SAMPLE_RATE, angle); //angle tilt and move the clock up

    refAngle += angle; // Add up the samples
    delay(SAMPLE_RATE);
  }
  refAngle /= REF_ANGLE_SAMPLES; // Divide by the number of samples to get the average
  angle = 0; // Reset the Filter

  DEBUG_PRINTLN(refAngle);
  digitalWrite(LED_PIN, LOW);
  lastMilli = millis();
}

void loop() 
{
  curMilli = millis();

  if ((curMilli - lastMilli) > SAMPLE_RATE){
    getAccAngle(); // Read the devices
    getGyroValues();

    rate = (float(curMilli) - float(lastMilli)) / 1000.0; // Calculate the actual rate in seconds (Gryo read outs are in seconds).

    maxGyro = absMax(gy,absMax(gx,gz)); //Since we can react to only one direction, it is assume to be maximal
    maxAngle = absMax(theta,psi);

    angle = compositeFilter( maxAngle, float(maxGyro), rate, angle); //angle tilt and move the clock up
    lastMilli = curMilli;  

    float error =  angle - refAngle;

    integral = integral + error * rate; // constrain the integral so we don't have to relax from really far values 
    derivative = (error - prevError) / rate;
    spd = (KP * error) + (KI * integral) + (KD * derivative); // Compute the speed to set the acceleromter
    prevError = error;    

    // Debug Output
    DEBUG_PRINT(curMilli);
    DEBUG_PRINT(" , ");
    DEBUG_PRINT(maxAngle);
    DEBUG_PRINT(" , ");
    DEBUG_PRINT(maxGyro);
    DEBUG_PRINT(" , ");
    DEBUG_PRINT(angle); // remove the refrence from the display
    DEBUG_PRINT(" , ");
    DEBUG_PRINT(error);
    DEBUG_PRINT(" , ");
    DEBUG_PRINT(rate);
    DEBUG_PRINT(" , ");
    DEBUG_PRINT(integral);
    DEBUG_PRINT(" , ");
    DEBUG_PRINT(derivative);
    DEBUG_PRINT(" , ");
    DEBUG_PRINT(spd);
    if ( spd  > 3.0){
      DEBUG_PRINT(" , ");
      DEBUG_PRINTLN("Posotive");
      digitalWrite(LED_PIN, HIGH);
    }
    else if ( spd  < -33.0){
      DEBUG_PRINT(" , ");
      DEBUG_PRINTLN("Negative");
      digitalWrite(LED_PIN, HIGH);
    }
    else{
      DEBUG_PRINT(" , ");
      DEBUG_PRINTLN("LEVEL");
      digitalWrite(LED_PIN, HIGH);

    }
  }
  else if((curMilli - lastMilli) < 0){ // time variable wraped around 
    lastMilli = curMilli;
  }
}









