/*
  SBRservo.cpp - Servo Library for Self Balancing Robot
 */

#include "SBRservo.h"
#include "Arduino.h"
#include <Servo.h>

#define SPD_WRITE_WAIT 50000 // Duration limit on Chaning the Speed in microseconds

Servo servo1;
Servo servo2;

int pos = -300;


unsigned long spdTime; //speed clock, need seprate clocks to decouple calls for speed adjustment from calls for degree rotation
unsigned long degTime; //degree clock


void SBRServoBegin(int pin1, int pin2, unsigned long curTime)
{

  servo1.attach(pin1);  // attaches the servo 
  servo2.attach(pin2);  
  spdTime = curTime;
  degTime = curTime;
}

//from: http://rosettacode.org/wiki/Map_range#C.2B.2B
float mapRange(float s, float a1,float a2,float b1,float b2)  // Linear mapping function
{
  return b1 + (s-a1)*(b2-b1)/(a2-a1);

}

int setSPD(int spd, unsigned long curTime) 
{
  /*
  This function will set the speed of the servos. It takes spd argument in deg/sec (-300, 300), and uses an aproximated
   calibration curve to set the pulse width. Sine the servos require some time to adjust the speed there is 
   lag check that will immedately return if there wasn't enough time between calls to the function
   */

  unsigned long dur = curTime - spdTime;
  
  if ( dur < SPD_WRITE_WAIT){ // return -1 if we haven't passed through the time limit, HOW do we prevent this from overflowing?
    return -1;
  } 
  spdTime = curTime; // Setting speed, so update clock

  /*
 This is an aproximated calibration curve, was built by hand using the servocalibrate degree sketch
   */
  if ( spd > 300){
    pos = 1700;
  }
  else if ( 270 < spd && spd <= 300){
    pos =  mapRange(spd, 270 , 300, 1600, 1700) + 0.5; 
  } 
  else if ( 180 < spd && spd <= 270) {
    pos =  mapRange(spd, 180, 270, 1550, 1600)+ 0.5; 
  } 
  else if ( 90 < spd && spd <= 180) {
    pos =  mapRange(spd, 90, 180, 1526, 1550)+ 0.5; 
  } 
  else if ( 1 <= spd && spd <= 90) {
    pos =  mapRange(spd, 1 , 90, 1501, 1526)+ 0.5; 
  } 
  else if ( 0 == spd) {
    pos =  1500; 
  } 
  else if ( -90 < spd && spd <= -1) {
    pos =  mapRange(spd, -90 , -1, 1471, 1494)+ 0.5; 
  } 
  else if ( -180 < spd && spd <= -90) {
    pos =  mapRange(spd, -180 , -90, 1448, 1471)+ 0.5; 
  } 
  else if ( -270 < spd && spd <= -180) {
    pos =  mapRange(spd, -270 , -180, 1400, 1448)+ 0.5; 
  } 
  else if ( -300 <= spd && spd <= -270) {
    pos =  mapRange(spd, -300 , -270, 1300, 1400)+ 0.5; 
  }
  else if (spd < -300 ) { 
    pos = 1300;
  }

  servo1.writeMicroseconds(-pos);              // Actully setting the speed.
  servo2.writeMicroseconds(pos);
  return pos; // otherwise return the pulse width value that came from the calibration curve.
}

/*
int rotateDeg(int deg, unsigned long curTime)
{
  
  Rotate through a specific angle between -180 and 180, this function will have to be called through every pass of the loop
   even if not being set so that it can stop the rotation at the right time

  unsigned long dur = curTime - degTime;
  Serial.print("deg Dur ");
  Serial.println(dur);

  if (!rotating){
    Serial.println("Starting");
    resSPD = setSPD(deg, curTime);
    rotating = true;
    degTime = curTime;
    return resSPD;
  }
  else if(1000 < dur && rotating) {
        Serial.println("Stopping");
    resSPD = setSPD(0, curTime);
    rotating = false;
    return resSPD;
  }
  else{
    return resSPD;
  }
}
*/
