/*
The actual robot code
 */

#include <Wire.h>
//#include <math.h>
#include <ITG3200.h>
#include <Servo.h>
#include "ADXL345.h"

//setup stuff
#define DEBUG
#define LED_PIN 11

//Servo stuff
#define SERVO1_PIN 10
#define SERVO2_PIN 12
#define SPD_WRITE_WAIT 15 // Duration limit on Chaning the Speed in milli
#define SERVO_CENTER 1500 // The center value of the servo (where it should be still)

//LPF stuff
#define FILTER_CONSTANT 0.90
#define PASTLENGTH 50
#define SAMPLE_RATE 5

//Ref Stuff
#define REF_SAMPLES 20

#define KA .70
#define KG .45
#define KI .20
#define SCALEFACTOR 20

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
Servo servo1 , servo2;
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
int filterIndex = 0;

float spd = 0.0;
float integral = 0.0;
float angle = 0.0;
float theta = 0.0;
float psi = 0.0;
float alpha = 0.0;
float rate = 0.0;
float maxAngle = 0.0;
float refAngle = 0.0;
float total = 0.0;
float pastVal[PASTLENGTH];

/*
-------------- Servo mangment functions --------------------------
 */

void SBRServoBegin(int pin1, int pin2, unsigned long curMilli)
{

  servo1.attach(pin1);  // attaches the servo 
  servo2.attach(pin2);  
  spdMilli = curMilli;
}

/*
-------------- Speed Setting function --------------------------
 */

int setSPD(float spd, unsigned long curMilli) 
{
  /*
  This function will set the speed of the servos. It takes spd argument in deg/sec (-300, 300), and uses an aproximated
   calibration curve to set the pulse width. Sine the servos require some time to adjust the speed there is 
   lag check that will immedately return if there wasn't enough time between calls to the function
   */

  unsigned long dur = curMilli - spdMilli;

  if ( dur < SPD_WRITE_WAIT){ // return -1 if we haven't passed through the time limit, HOW do we prevent this from overflowing?
    return -1;
  } 
  spdMilli = curMilli; // Setting speed, so update clock

  /*
 This is an exact calibration curve, was built by hand using the servocalibrate degree sketch
   */
  if ( spd >= 300.0){
    pos = 1700;
    digitalWrite(LED_PIN, LOW);
  }
  else if ( 180.0 <= spd && spd < 300.0){
    pos =  1559;
    digitalWrite(LED_PIN, HIGH);
  }
  else if ( 90.0 <= spd && spd < 180.0){
    pos =  1531;
    digitalWrite(LED_PIN, HIGH);
  }
  else if ( 45.0 <= spd && spd < 90.0)
  {
    pos =  1520;
    digitalWrite(LED_PIN, HIGH);
  }
  else if ( 22.0 < spd && spd < 45.0)
  {
    pos =  1515;
    digitalWrite(LED_PIN, HIGH);
  }
  else if ( -22.0 <= spd && spd <= 22.0) {
    pos =  SERVO_CENTER;
    digitalWrite(LED_PIN, LOW);
  }
  else if ( -45.0 < spd && spd < -22.0)
  {
    pos =  1485;
    digitalWrite(LED_PIN, HIGH);
  }
  else if ( -90.0 < spd && spd <= -45.0)
  {
    pos =  1479;
    digitalWrite(LED_PIN, HIGH);
  }
  else if ( -180.0 < spd && spd <= -90.0){
    pos =  1469;
    digitalWrite(LED_PIN, HIGH);
  }
  else if ( -300.0 < spd && spd <= -180.0){
    pos =  1436;
    digitalWrite(LED_PIN, HIGH);
  }
  else if (spd <= -300.0 ) { 
    pos = 1300;
    digitalWrite(LED_PIN, LOW);
  }

  servo1.writeMicroseconds(pos);              // Actully setting the speed.
  rpos = map(pos, 1300, 1700, 1700, 1300); // Reverse the direction for the other side
  servo2.writeMicroseconds(rpos);
  return abs(pos - SERVO_CENTER); //  return the pulse width value as an offset from center.
}


/*
---------------------- Composite Filter -------------------------------------------
 */

void filterbegin(){
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
  for (filterIndex = 0; filterIndex < PASTLENGTH; filterIndex++){
    pastVal[filterIndex] = 0;   
  }
  filterIndex = 0;
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
  /* 
   The correct calculation
   
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

float LPF(float inval){
  total -=  pastVal[filterIndex];
  pastVal[filterIndex] = FILTER_CONSTANT * inval;
  total +=  pastVal[filterIndex];

  if (filterIndex == PASTLENGTH - 1){
    filterIndex = 0;
  }
  else {
    filterIndex++;
  }

  return total / float(PASTLENGTH);
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
  filterbegin();

  for (int ii = 0; ii < REF_SAMPLES; ii++){
    getAccAngle(); 
    maxAngle = absMax(theta,psi);
    angle = LPF(maxAngle);
    refAngle += angle;
    delay(SAMPLE_RATE);
  }
  refAngle /= REF_SAMPLES;
  DEBUG_PRINT("Refrence Angle:");
  DEBUG_PRINTLN(refAngle);

  digitalWrite(LED_PIN, LOW);
  lastMilli = millis();
  SBRServoBegin(SERVO1_PIN, SERVO2_PIN, lastMilli);
}

void loop() 
{
  curMilli = millis();


  if ((curMilli - lastMilli) > SAMPLE_RATE){
    lastMilli = curMilli;
    getAccAngle(); // Read the devices
    getGyroValues();

    maxGyro = absMax(gy,absMax(gx,gz)); //Since we can react to only one direction, it is assume to be maximal
    maxAngle = absMax(theta,psi);

    angle = LPF(maxAngle);
    integral += (refAngle - angle) * SAMPLE_RATE;
    integral = constrain(integral, -100, 100);

    if (abs(refAngle - angle) < 5){
      spd = SCALEFACTOR * (KA * (refAngle - angle)) + (KG * maxGyro) + (KI * integral); 
    }
    else if (abs(refAngle - angle) > 5 && abs(refAngle - angle) < 10 ){
      spd = 2 * SCALEFACTOR * (KA * (refAngle - angle)) + (KG * maxGyro) + (KI * integral); 
    }
    else {
      spd = 3 * SCALEFACTOR * (KA * (refAngle - angle)) + (KG * maxGyro) + (KI * integral); 
    }
    resPos = setSPD(-spd, curMilli); //set the Speed 


    // Debug Output
    if (resPos  != -1){ // Only print when we change something
      DEBUG_PRINT(curMilli);
      DEBUG_PRINT(" , ");
      DEBUG_PRINT(maxAngle);
      DEBUG_PRINT(" , ");
      DEBUG_PRINT(maxGyro);
      DEBUG_PRINT(" , ");
      DEBUG_PRINT(angle); 
      DEBUG_PRINT(" , ");
      DEBUG_PRINT(refAngle - angle); 
      DEBUG_PRINT(" , ");
      DEBUG_PRINT(integral); 
      DEBUG_PRINT(" , ");
      DEBUG_PRINT(spd);
      DEBUG_PRINT(" , ");
      DEBUG_PRINTLN(resPos); 
    }
    else{
      //  DEBUG_PRINTLN("Skipping"); 
    }
  }
  else if((curMilli - lastMilli) < 0){ // time variable wraped around 
    lastMilli = curMilli;
  }
}














