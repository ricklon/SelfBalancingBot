// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
 
Servo servo1;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
           
Servo servo2;

int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  servo1.attach(14);  // attaches the servo on pin 9 to the servo object 
  servo2.attach(15);
} 
 
 
void loop() 
{ 
  
  servo1.write(0);              // tell servo to go to position in variable 'pos' 
  servo2.writeMicroseconds(1000);
   // delay(4000);   
   //  servo1.write(70);              // tell servo to go to position in variable 'pos' 
   // servo2.write(80);   
    delay(4000);   
    servo1.write(94);              // tell servo to go to position in variable 'pos' 
    servo2.writeMicroseconds(1400);  
   // delay(4000);
   // servo1.write(100);              // tell servo to go to position in variable 'pos' 
   // servo2.writeMicroseconds(1000);
    delay(4000);
    servo1.write(180);              // tell servo to go to position in variable 'pos'  
    servo2.writeMicroseconds(2000);  
    delay(4000);   
    
    
  /*
  for(pos = 0; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    servo1.write(pos);              // tell servo to go to position in variable 'pos' 
    servo2.write(pos);   
    delay(100);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos = 180; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    servo1.write(pos);              // tell servo to go to position in variable 'pos' 
    servo2.write(pos); 
    delay(100);                       // waits 15ms for the servo to reach the position 
  } 
  */
  
} 
