#include <Servo.h>
#include "SBRservo.h"

#define SERVO1_PIN 17
#define SERVO2_PIN 18
#define WAIT 3000
#define DEG_WRITE_WAIT 1000 // Duration to walk through partial rotation, this must be greater than SPD_WRITE_WAIT
#define WINDOW 5 // Millisecond 

int spd  = -310;
unsigned long lastMillis = 0;
int resSPD = 0;
boolean rotating = false;

int deg = -180;

void setup() 
{ 
  Serial.begin(9600);
  lastMillis = millis();
  SBRServoBegin(SERVO1_PIN, SERVO2_PIN, lastMillis);

} 


void loop() 
{ 
  /* 
   Speed Test - Sweeps through all the possible speeds 
   
   if (spd > 310)
   {
   spd = -310;
   }
   //   Serial.print("SPD Value:");
   unsigned long curMillis = millis();
   pos = setSPD(spd, curMillis);
   //   Serial.print(" Pos Value:");
   
   if (pos != -1){
   Serial.print(spd);
   Serial.print(",");
   Serial.println(pos);
   spd += 10;
   }
   */

  /*
   Rotate through a fixed degree by setting the speed and then litiming the rotation time to 1 second.
   */

  unsigned long curMillis = millis();
  unsigned long dur = curMillis - lastMillis;

  if (0 < dur && dur < WINDOW && !rotating){ // Starting window - Only allowed 5 milliseconds to set the start the pulse
    Serial.print("Starting: ");
    resSPD = setSPD(deg, curMillis);
    Serial.println(resSPD);
    rotating = true;
  }
  else if(DEG_WRITE_WAIT < dur && dur < (DEG_WRITE_WAIT + WINDOW) && rotating) { // Stopp window - Only allowed 5 milliseconds to set the stop the pulse
    Serial.print("Stopping: ");
    resSPD = setSPD(0, curMillis); 
    Serial.println(resSPD);
    rotating = false;
  }
  else if (dur > WAIT){ // This is just for demo purposes, it sweeps through the the angle range -180 to 180
    Serial.println("Restarting");
    lastMillis = curMillis; // Reset the local clock
    if (deg > 180){ // degree range sweep
      deg = -180;      
    }
    else{
      deg += 30;
    }
  }

} 
