// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.
#include <Servo.h>



Servo servo1;
Servo servo2;
int acc = -100;
int pos = 0;
int reading = 0;
int ledState = 0;
unsigned int lastTime = 0;
unsigned int curTime = 0;
float past = 0;

void setup() 
{ 
  servo1.attach(SERVO1_PIN );  // attaches the servo on pin1 to the servo object 
  servo2.attach(SERVO2_PIN );  // attaches the servo on pin2 to the servo object 
  lastTime = millis();
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
} 

void loop() 
{ 
  /*
   Serial.print("Acc: ");
   Serial.print(acc);
   Serial.print(" Reading: ");
   Serial.println(reading / 2);
   */
   curTime = millis();
   past =  ( curTime - lastTime) / 1000; // How many seconds has passed since the last led state change

  Serial.print("Past: ");
  Serial.print(past);
  
  if ( past > 6.0){ // if it's 6 seconds switch.
    ledState += 1;
    lastTime = curTime;
    if ((ledState % 2) == 0){
      digitalWrite(LED_PIN, HIGH);
    }
    else {
      digitalWrite(LED_PIN, LOW);
    }
  }
  Serial.print(" Last: ");
  Serial.print(lastTime);
  Serial.print(" LED: ");
  Serial.print(ledState);
  
  int pos = analogRead(POT_PIN);
  pos =  map(pos, 0, 1023, 1250, 1750);
  servo1.writeMicroseconds(pos); // tell servo1 to go to position in variable 'pos' 
  servo2.writeMicroseconds(pos); // tell servo2 to go to position in variable 'pos'
  Serial.print(" Servo1: ");
  Serial.print(servo1.readMicroseconds());
  Serial.print(" Servo2: ");
  Serial.println(servo2.readMicroseconds());
  
} 









