/**************************************************************************
 *                                                                         *
 * ADXL345 Driver for Arduino                                              *
 *                                                                         *
 ***************************************************************************
 *                                                                         * 
 * This program is free software; you can redistribute it and/or modify    *
 * it under the terms of the MIT License.                                  *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * MIT License for more details.                                           *
 *                                                                         *
 ***************************************************************************
 * 
 * Revision History
 * 
 * Date  By What
 * 20100515 TJS Initial Creation 
 * 20100524 TJS Modified to run with Kevin Stevenard's driver
 */

#include <math.h>
#include "Wire.h"
#include "ADXL345.h"

ADXL345 Accel;

void setup(){
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  delay(1);
  Wire.begin();
  delay(1);
  Serial.println("Here");
  Accel.init(ADXL345_ADDR_ALT_LOW);
  Accel.set_bw(ADXL345_BW_12);
  Serial.print("BW_OK? ");
  Serial.println(Accel.status, DEC);
  delay(1000);
}

void loop(){
  float theta;
  float psi;
  float phi;

  float acc_data[3];
  Accel.get_Gxyz(acc_data);

  /*
  int acc_data[3];
   Accel.readAccel(acc_data);
   */

  if(Accel.status){
    //x^2 + y^2
    //small again approximation but outside of 20 degrees problem.
    //  theta = (acc_data[0]/(acc_data[1]*acc_data[1] + acc_data[2]*acc_data[2]));
    //uses the math function of atan
    //theta = atan((float)acc_data[0]/((float)acc_data[1]*(float)acc_data[1] + (float)acc_data[2]*(float)acc_data[2]));
    theta = atan(acc_data[0]/sqrt(acc_data[1]*acc_data[1] + acc_data[2]*acc_data[2]));
    psi = atan(acc_data[1]/sqrt(acc_data[0]*acc_data[0] + acc_data[2]*acc_data[2]));
    phi=  atan(sqrt(acc_data[0]*acc_data[0] + acc_data[1]*acc_data[1])/acc_data[2]);
    Serial.print("Theta: ");
    Serial.print(theta);
    Serial.print(" Psi: ");
    Serial.print(psi);
    Serial.print(" Phi: ");
    Serial.println(phi);

    delay(40);
  }
  else{
    Serial.println("ERROR: ADXL345 data read error");
  }
}

