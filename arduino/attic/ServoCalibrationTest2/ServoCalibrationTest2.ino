// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.
#include <Servo.h>

#define SERVO1_PIN 17
#define SERVO2_PIN 18

Servo servo1;
Servo servo2;
int acc = -100;
// int pos = 0;
int pos = 1300;
int reading = 0;

void setup() 
{ 
  servo1.attach(SERVO1_PIN );  // attaches the servo on pin1 to the servo object 
  servo2.attach(SERVO2_PIN );  // attaches the servo on pin2 to the servo object 
  Serial.begin(9600);
} 
/*
int setAcc(int acc){
 
 if (acc > 0){
 pos =  map(acc, -300, 300, 1316, 1716);
 }
 servo1.writeMicroseconds(pos); // tell servo1 to go to position in variable 'pos' 
 servo2.writeMicroseconds(pos); // tell servo2 to go to position in variable 'pos' 
 return servo1.readMicroseconds() + servo2.readMicroseconds(); // Sanity Check on the value of servo1
 }
 */

void loop() 
{ 
  /*
  if (acc > 100)
   {
   acc = -100;
   }
   acc += 5;
   Serial.print("Acc: ");
   Serial.print(acc);
   reading = setAcc(acc);
   Serial.print(" Reading: ");
   Serial.println(reading / 2);
   delay(2000);
   */

  //Map out the stop region
  if (pos > 1512)
  {
    pos = 1300;
  }
  pos += 1;
  Serial.print("Pos: ");
  Serial.print(pos);
  servo1.writeMicroseconds(pos); // tell servo1 to go to position in variable 'pos' 
  servo2.writeMicroseconds(pos); // tell servo2 to go to position in variable 'pos'
  Serial.print(" Reading: ");
  Serial.println(servo1.readMicroseconds());
  delay(1000);
} 





