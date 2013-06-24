/*
  SBRservo.cpp - Servo Library for Self Balancing Robot
 
 */

#include "SBRservo.h"
#include "Arduino.h"
#include <Servo.h>


int pos = -300;
Servo servo1;
Servo servo2;

float nodepoints[9][2]= {
  {-300,1300}
  , {-270,1400}
  , {-180,1448}
  , {-90, 1471}
  , {0,1500}
  , {90,1526}
  , {180,1550}
  , {270,1526}
  , {300,1700}
};

void SBRServoBegin(int pin1, int pin2 )
{

  servo1.attach(pin1);  // attaches the servo on pin1 to the servo object 
  servo2.attach(pin2);  // attaches the servo on pin2 to the servo object 
}

// from: http://interface.khm.de/index.php/lab/experiments/nonlinear-mapping/
int reMap(float pts[10][2], int input) { 
  int rr;
  float bb,mm;

  for (int nn=0; nn < 4; nn++) {

    if (input >= pts[nn][0] && input <= pts[nn+1][0]) {
      mm= ( pts[nn][1] - pts[nn+1][1] ) / ( pts[nn][0] - pts[nn+1][0] );
      mm= mm * (input-pts[nn][0]);
      mm = mm +  pts[nn][1];
      rr = mm;
    }
  }
  return(rr);
}



int setSPD(int spd) // acc is the desired acceleration in degrees per second, range is -300 to 300
{
  /* 
  This is the linear approximation to the calibration curve.  Map may not necissarily be the correct function to use here,
  since it blindly truncates. 
  
  if ( spd  >= 270){ 
    pos =  map(spd, 270 , 300, 1600, 1700); 
  } 
  else if ( 180 <= spd  < 270) {
    pos =  map(spd, 180, 270, 1550, 1600); 
  } 
  else if ( 90 <= spd  < 180) {
    pos =  map(spd, 90, 180, 1526, 1550); 
  } 
  else if ( 0 <= spd  < 90) {
    pos =  map(spd, 0 , 90, 1500, 1526); 
  } 
  else if ( -90 <= spd  < 0) {
    pos =  map(spd, -90 , 0, 1471, 1500); 
  } 
  else if ( -180 <= spd  < -90) {
    pos =  map(spd, -180 , -90, 1448, 1471); 
  } 
  else if ( -270 <= spd  < -180) {
    pos =  map(spd, -270 , -180, 1400, 1448); 
  } 
  else if ( -270 < spd) {
    pos =  map(spd, -300 , -270, 1300, 1400); 
  }
  else {
    return -1;
  }
  
  This didn't work, going to try a diffrent method
  */
  pos = reMap(nodepoints,spd) + 0.5;
  servo1.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos' 
  servo2.writeMicroseconds(pos);
  return pos;
}

void rotateDeg(float deg) //Rotate through a specific angle between -180 and 180
{
  /*
  _servo1.write();
   _servo2.write();
   */
}







