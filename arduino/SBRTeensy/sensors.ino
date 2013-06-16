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
  
     theta = atan(acc_data[0]/sqrt(acc_data[1]*acc_data[1] + acc_data[2]*acc_data[2]));
    psi = atan(acc_data[1]/sqrt(acc_data[0]*acc_data[0] + acc_data[2]*acc_data[2]));
    phi=  atan(sqrt(acc_data[0]*acc_data[0] + acc_data[1]*acc_data[1])/acc_data[2]);
    
    Serial.print("Theta: ");
    Serial.print(theta);
    Serial.print(" Psi: ");
    Serial.print(psi);
    Serial.print(" Phi: ");
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

  
  Serial.print(" gx: ");
  Serial.print(gx);
  Serial.print(" gx: ");
  Serial.print(gx);
  Serial.print(" gx: ");
  Serial.println(gx);

  return gx;
}


