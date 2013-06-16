// Sensors Modules

int getGyroRate() {        // ARef=3.3V, Gyro sensitivity=2mV/(deg/sec)
   sensorValue[GYR_Y] = getGyroValues(); 
   
   return int((sensorValue[GYR_Y])/(lastLoopUsefulTime));          
}

float getAccAngle() {
 float theta;
  float psi;
  float phi;
  
  float acc_data[3];
  Accel.get_Gxyz(acc_data);
  
    theta = atan(acc_data[0]/sqrt(acc_data[1]*acc_data[1] + acc_data[2]*acc_data[2]))*180/PI;
    psi = atan(acc_data[1]/sqrt(acc_data[0]*acc_data[0] + acc_data[2]*acc_data[2]))*180/PI;
    phi=  atan(sqrt(acc_data[0]*acc_data[0] + acc_data[1]*acc_data[1])/acc_data[2])*180/PI;
    
    
    //Serial.print("Theta: ");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(theta);
    Serial.print(",");
    Serial.print(psi);
    Serial.print(",");
    Serial.print(phi);
    
   return psi;
}


// ------------------ read gyroscope angles ---------------------

float getGyroValues(){
  int gx, gy, gz;
  gyro.readGyro(xyz);
  gx = xyz[0];
  gy = xyz[1];
  gz = xyz[2];

  
 // Serial.print(" gx: ");
  Serial.print(",");
  Serial.print(gx);
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.print(gz);
  Serial.println("\r");


  return gx;
}


