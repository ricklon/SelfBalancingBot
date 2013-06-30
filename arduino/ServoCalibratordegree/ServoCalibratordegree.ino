//SBRServo Calibration library
//by James "Shridatt" Sugrim

#include <Servo.h>

#define SERVO1_PIN 17
#define SERVO2_PIN 10
#define LED_PIN 11
#define SERVO_ZERO 1500

char buff[5];
int ii = 0;
Servo servo1;
Servo servo2;
int acc = -100;
int pos = 0;
int reading = 0;
int ledState = 0;
//int lastTime = 0;
int curTime = 0;
float past = 0;

void setup() 
{ 
  servo1.attach(SERVO1_PIN );  // attaches the servo on pin1 to the servo object 
  servo2.attach(SERVO2_PIN );  // attaches the servo on pin2 to the servo object 
  //  lastTime = millis();
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
} 

void loop() 
{ 

  ii=0;

  while (Serial.available() > 0)
  {
    buff[ii] = Serial.read();
    Serial.print(buff[ii]);
    if (buff[ii] == 0x0d )
    {
      pos = atoi(buff);
      break;
    }
    ii++;
  }

  ledState += 1;
  if ((ledState % 2) == 0){
    digitalWrite(LED_PIN, HIGH);
    //   int pos = analogRead(A0);
    //    pos =  map(pos, 0, 1023, 1250, 1750);
    servo1.writeMicroseconds(pos); // tell servo1 to go to position in variable 'pos' 
    servo2.writeMicroseconds(pos); // tell servo2 to go to position in variable 'pos'
  }
  else {
    digitalWrite(LED_PIN, LOW);
    servo1.writeMicroseconds(SERVO_ZERO); // tell servo1 to go to position in variable 'pos' 
    servo2.writeMicroseconds(SERVO_ZERO); // tell servo2 to go to position in variable 'pos'
  }
  Serial.print(" Servo1: ");
  Serial.print(servo1.readMicroseconds());
  Serial.print(" Servo2: ");
  Serial.println(servo2.readMicroseconds());

  delay(2000);
} 
