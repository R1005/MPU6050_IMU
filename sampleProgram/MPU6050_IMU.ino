#include "MPU6050_IMU.h"

MPU6050_IMU MPU6050;

void setup(){
  Serial.begin(9600);
  MPU6050.offset(34,-20,29,1241);
  MPU6050.start();
}

void loop(){
  int val = MPU6050.getAngle();
  Serial.println(val);
}
