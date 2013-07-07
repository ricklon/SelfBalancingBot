/*
The actual robot code
 */

#include <Wire.h>
//#include <math.h>
#include <ITG3200.h>
#include <Servo.h>
#include "ADXL345.h"

//#define DEBUG
#define SERVO1_PIN 10
#define SERVO2_PIN 12
#define LED_PIN 11
#define REF_ANGLE_SAMPLES 500
#define SPD_WRITE_WAIT 10000 // Duration limit on Chaning the Speed in microseconds
#define TIME_CONSTANT 130000 // time constant in Micrseconds
#define SAMPLE_RATE   4000 // Assumed Sampled rate in microseconds
#define REPORT_RATE   2000 // Assumed Sampled rate in microseconds
#define SERVO_CENTER 1500 // The center value of the servo (where it should be still)

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
Servo servo1 , servo2;
ITG3200 gyro = ITG3200();
ADXL345 Accel;


unsigned long curMicro = 0, lastMicro = 0, prevMicro = 0, rate = 0, spdMicro = 0; //Varios time stamps

int spd = 0, gyroRate = 0, resPos = 0, acc = 0, gx = 0, gy = 0, gz = 0, pos = 0, rpos = 0;
float kg = 0.0, ka = 0.0, angle = 0, refAngle = 0 , theta = 0, psi = 0, phi = 0, normAngle = 0, normACC = 0, alpha = 0;

/*
-------------- Servo mangment functions --------------------------
 */

void SBRServoBegin(int pin1, int pin2, unsigned long curMicro)
{

  servo1.attach(pin1);  // attaches the servo 
  servo2.attach(pin2);  
  spdMicro = curMicro;
}

/*
-------------- Alternative range mapping function --------------------------
 */

//from: http://rosettacode.org/wiki/Map_range#C.2B.2B
float mapRange(float s, float a1,float a2,float b1,float b2)  // Linear mapping function
{
  return b1 + (s-a1)*(b2-b1)/(a2-a1);
}

/*
-------------- Speed Setting function --------------------------
 */


int setSPD(int spd, unsigned long curMicro) 
{
  /*
  This function will set the speed of the servos. It takes spd argument in deg/sec (-300, 300), and uses an aproximated
   calibration curve to set the pulse width. Sine the servos require some time to adjust the speed there is 
   lag check that will immedately return if there wasn't enough time between calls to the function
   */

  unsigned long dur = curMicro - spdMicro;

  if ( dur < SPD_WRITE_WAIT){ // return -1 if we haven't passed through the time limit, HOW do we prevent this from overflowing?
    return -1;
  } 
  spdMicro = curMicro; // Setting speed, so update clock

  /*
 This is an aproximated calibration curve, was built by hand using the servocalibrate degree sketch
   */
  if ( spd >= 200){
    pos = 1700;
    digitalWrite(LED_PIN, LOW);
  }
  else if ( 100 <= spd && spd < 200){
    pos =  1650;
   digitalWrite(LED_PIN, HIGH); 
  }
  else if ( 20 <= spd && spd < 100){
    pos =  1600;
    digitalWrite(LED_PIN, HIGH);
  }
  else if ( 5 <= spd && spd < 20){
    pos =  1575;
    digitalWrite(LED_PIN, HIGH);
  }
  else if ( -1 < spd && spd < 5)
  {
     pos =  1550;
    digitalWrite(LED_PIN, HIGH);
  }
  else if ( -1 <= spd && spd <= 1) {
    pos =  SERVO_CENTER;
    digitalWrite(LED_PIN, LOW);
  }
  else if ( -5 < spd && spd < -1)
  {
    pos =  1450;
    digitalWrite(LED_PIN, HIGH);
  }
  else if ( -20 < spd && spd <= -5){
    pos =  1425;
    digitalWrite(LED_PIN, HIGH);
  }
  else if ( -100 < spd && spd <= -20){
    pos =  1400;
    digitalWrite(LED_PIN, HIGH);
  }
  else if ( -200 < spd && spd <= -100){
    pos =  1350;
    digitalWrite(LED_PIN, HIGH);
  }
  else if (spd <= -200 ) { 
    pos = 1300;
    digitalWrite(LED_PIN, LOW);
  }

  servo1.writeMicroseconds(pos);              // Actully setting the speed.
  rpos = mapRange(pos, 1300, 1700, 1700, 1300); // Reverse the direction for the other side
  servo2.writeMicroseconds(rpos);
  return abs(pos - SERVO_CENTER); //  return the pulse width value as an offset from center.
}


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

  alpha = float(TIME_CONSTANT) / (float(TIME_CONSTANT) + float(SAMPLE_RATE)); // calculate the scaling coefficent
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

float compositeFilter(float acc, float gyro, unsigned long rate, float angle){
  return alpha * (angle + (gyro * float(rate)))  + (1 - alpha) * acc; // the actual filter
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
  SBRServoBegin(SERVO1_PIN, SERVO2_PIN, lastMicro);
  DEBUG_PRINT("Computing Refrence angle...:"); 
  for (int i = 0; i < REF_ANGLE_SAMPLES; i++){
    getAccAngle();
    refAngle += absMax(theta,psi);
    delay(10);
  }
  refAngle /= REF_ANGLE_SAMPLES;
  DEBUG_PRINTLN(refAngle);
  SBRServoBegin(SERVO1_PIN, SERVO2_PIN, lastMicro);
  digitalWrite(LED_PIN, LOW);
}

void loop() 
{
  curMicro = micros();

  if ((curMicro - lastMicro) > SAMPLE_RATE){
    getAccAngle(); // Read the devices
    getGyroValues();

    rate = (curMicro - prevMicro) / 100000; // Calculate the actual rate in seconds (Gryo read outs are in seconds).

    gyroRate = absMax(gy,absMax(gx,gz)); //Since we can react to only one direction, it is assume to be maximal
    acc = absMax(theta,psi);

    angle = compositeFilter( acc, gyroRate, rate, angle); //angle tilt and move the clock up
    lastMicro = curMicro;  

    ka = 0.90;
    kg = 0.05;

    spd = (kg * gyroRate) + (ka * (angle - refAngle)); // Compute the speed to set the acceleromter

    resPos = setSPD(spd, curMicro); //set the Speed 

    // Debug Output
    if (resPos  != -1){ // Only print when we change something
      DEBUG_PRINT(curMicro);
      DEBUG_PRINT(" , ");
      DEBUG_PRINT(acc);
      DEBUG_PRINT(" , ");
      DEBUG_PRINT(gyroRate);
      DEBUG_PRINT(" , ");
      DEBUG_PRINT(angle - refAngle); // remove the refrence from the display
      DEBUG_PRINT(" , ");
      DEBUG_PRINT(spd);
      DEBUG_PRINT(" , ");
      DEBUG_PRINTLN(resPos); 
    }

  }
  else if((curMicro - lastMicro) < 0){ // time variable wraped around 
    lastMicro = curMicro;
  }
  prevMicro = curMicro; // needed for loop length calculation
}



