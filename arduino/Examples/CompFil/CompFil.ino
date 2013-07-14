/*
The actual robot code
 */

#include <Wire.h>
//#include <math.h>
#include <ITG3200.h>
#include "ADXL345.h"


#define DEBUG

/*
Complimentary filter Constants
 */
#define LED_PIN 11
#define TIME_CONSTANT 250.0 // time constant in milli
#define SAMPLE_RATE   7.0 // Assumed Sampled rate in milli



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

int gx = 0;
int gy = 0;
int gz = 0;

float alpha = 0.0;
float angle = 0.0;
float theta = 0.0;
float psi = 0.0;
float rate = 0.0;


/*
---------------------- Composite Filter -------------------------------------------
 */

void Cfilterbegin(){
  //Turning on the accelerometer
  Accel.init(ADXL345_ADDR_ALT_LOW);
  Accel.set_bw(ADXL345_BW_12);

  // Gyro Initilization
  gyro.reset();
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
  pinMode(LED_PIN, OUTPUT); // Setup led pin
  digitalWrite(LED_PIN, HIGH); // Turn on LED

  DEBUG_BEGIN(115200); //Serial Com setup
  Wire.begin(); //IMU com setup

  Cfilterbegin(); // Turn on IMU and calibrate

  digitalWrite(LED_PIN, LOW); // Turn off LED to indicate calibration done
  lastMilli = millis(); // Initialize clock
}

void loop() 
{
  curMilli = millis();

  if ((curMilli - lastMilli) > SAMPLE_RATE){
    getAccAngle(); // Read the devices
    getGyroValues();

    rate = (float(curMilli) - float(lastMilli)) / 1000.0; // Calculate the actual rate in seconds (Gryo read outs are in seconds).

    int  gyroMax = absMax(gy,absMax(gx,gz)); //Since we can react to only one direction, it is assume to be maximal
    int angleMax = absMax(theta,psi);

    angle = compositeFilter( angleMax, gyroMax, rate, angle); //angle tilt and move the clock up
    lastMilli = curMilli;  
    DEBUG_PRINT(curMilli);
    DEBUG_PRINT(" , ");
    DEBUG_PRINT(angleMax);
    DEBUG_PRINT(" , ");
    DEBUG_PRINT(gyroMax);
    DEBUG_PRINT(" , ");
    DEBUG_PRINT(angle); 
    DEBUG_PRINT(" , ");
    DEBUG_PRINTLN(angleMax - angle); // remove the refrence from the display
  }
  else if((curMilli - lastMilli) < 0){ // time variable wraped around 
    lastMilli = curMilli;
  }
}








