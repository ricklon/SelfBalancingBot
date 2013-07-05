
//Balancing Robot by Jason Dorweiler http://www.jddorweiler.appspot.com/electronics.html#robot
//  Much of the code is adapted from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1284738418/all
// and http://www.kerrywong.com/2012/03/08/a-self-balancing-robot-i
// this version uses a 6DOF board (http://www.csgshop.com/product.php?id_product=64) and servo motors (https://www.adafruit.com/products/154)

#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include "ADXL345.h"
#include <ITG3200.h>

ITG3200 gyro = ITG3200();
float xyz[3], temperature;
ADXL345 Accel;


#define   GYR_Y                 0                              
#define   ACC_Z                 1                              
#define   ACC_Y                 2  

#define TO_READ (6)      //num of bytes to read each time (two bytes for each axis)
#define   LINE_END              10                             // \n
#define   SPLIT                 58                             // :

Servo servo1,servo2;  // create servo object to control a servo 
int   STD_LOOP_TIME  =          9;             

int sensorValue[3]  = { 0, 0, 0};
int sensorZero[3]  = { 0, 0, 0 }; 
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;
int actAngle;                                                  
int ACC_angle;
int GYRO_rate;
int torque1, torque2;
int setPoint = 0;
int drive = 0;
int error = 0;
int updateRate = 5;            //Nr of loop to skip sending and receving info from PC
float motorOffsetL = 1;        //The offset for left motor
float motorOffsetR = 1;        //The offset for right motor
byte buff[TO_READ] ;    //6 bytes buffer for saving data read from the device
char str[512];          //string buffer to transform data before sending it to the serial port



//********************Change the tuning parameters here**********************
//Setpoint.  Find where the robot is balanced.  
double Setpoint=-5;
//Point where it switches from conservative to agressive 
int gapDist=15;
//Aggressive
double aggK=0.5, aggKp=5, aggKi=.5, aggKd=4; 
//Conservative
double consK=0.2, consKp=5, consKi=.2, consKd=1;
//***************************************************************************

void setup() {
  /* while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  */
  
  servo1.attach(14);
  servo2.attach(15);
  analogReference(EXTERNAL); 
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

void loop() {
  
// ********************* Sensor aquisition & filtering *******************
 
  ACC_angle = getAccAngle(); 
  GYRO_rate = getGyroRate();  
  actAngle = kalmanCalculate(ACC_angle, GYRO_rate, lastLoopTime);            // calculate Absolute Angle
  

// *********************** PID and motor drive *****************
   double gap = abs(Setpoint-actAngle); //distance away from setpoint
  if(gap<gapDist)
  {  //we're close to setpoint, use conservative tuning parameters
    drive = updatePid(Setpoint, actAngle, consK, consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
    drive =  updatePid(Setpoint, actAngle, aggK, aggKp, aggKi, aggKd);
  }
  if(actAngle>(setPoint-200) && actAngle<(setPoint+200))     
  {
  Drive_Motor(drive); 
  }
  else                                                    
  {
    Drive_Motor(0);     // stop motors if situation is hopeless
  }
  
// *********************** loop timing control **************************
  lastLoopUsefulTime = millis()-loopStartTime;
  if(lastLoopUsefulTime<STD_LOOP_TIME)         delay(STD_LOOP_TIME-lastLoopUsefulTime);
  lastLoopTime = millis() - loopStartTime;
  loopStartTime = millis();
}
