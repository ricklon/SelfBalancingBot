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
extern int curDeg;
extern unsigned long spdTime;
extern unsigned long degTime;
extern boolean needStop;

void SBRServoBegin(int pin1, int pin2, unsigned long curTime);
int setSPD(int spd, unsigned long curTime);
//int rotateDeg(int deg, unsigned long curTime);
#endif




