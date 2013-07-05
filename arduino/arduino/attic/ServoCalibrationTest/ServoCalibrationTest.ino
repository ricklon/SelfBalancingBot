// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.
#include <Servo.h>

#define SERVO1_PIN 11
#define SERVO2_PIN 10

Servo servo1;
Servo servo2;

void setup() 
{ 
  servo1.attach(SERVO1_PIN );  // attaches the servo on pin1 to the servo object 
  servo2.attach(SERVO2_PIN );  // attaches the servo on pin2 to the servo object 
} 


void loop() 
{ 

  /*
  for(pos = 1000; pos < 2000; pos += 1)  // goes from 0 degrees to 180 degrees 
   {                                  // in steps of 1 degree 
   servo1.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos' 
   servo2.writeMicroseconds(pos);   
   delay(1000);                       // waits 15ms for the servo to reach the position 
   } 
   for(pos = 2000; pos>=1000; pos-=1)     // goes from 180 degrees to 0 degrees 
   {                                
   servo1.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos' 
   servo2.writeMicroseconds(pos); 
   delay(1000);                       // waits 15ms for the servo to reach the position 
   } 
   */

  int pos = analogRead(A0);
  pos =  map(pos, 0, 1023, 1000, 2000);
  servo1.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos' 
  servo2.writeMicroseconds(pos); 

} 

