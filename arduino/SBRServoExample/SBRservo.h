/*SBRservo.h - Servio Library for Self Balancing Robot
 Provides: SBRservo
 methods
 setAccel
 rotateDeg
 */

#ifndef SBRservo_h
#define SBRservo_h

#include "Arduino.h"
#include <Servo.h>

 extern int pos;
 extern  Servo servo1;
 extern  Servo servo2;
 
 void SBRServoBegin(int pin1, int pin2 );
 int setSPD(int spd);
 void rotateDeg(float deg);
#endif



