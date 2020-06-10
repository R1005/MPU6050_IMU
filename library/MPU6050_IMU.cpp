#include "Arduino.h"
#include "MPU6050_IMU.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;

static uint8_t mpuIntStatus;
static bool dmpReady = false;
static uint16_t packetSize;

int16_t  Gyro_Now = 0, Gyro = 0, Gyro_Offset = 0;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
float ypr[3];
int GY_x,GY_y,GY_z,GY_accel;

MPU6050_IMU::MPU6050_IMU(){
}

void MPU6050_IMU::offset(int x,int y,int z,int accel){
  GY_x = x;
  GY_y = y;
  GY_z = z;
  GY_accel = accel;
}

int  MPU6050_IMU::getAngle() {
  mpuIntStatus = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);;
    Gyro_Now = degrees(ypr[0]);
    Gyro = Gyro_Now + Gyro_Offset;
    if (Gyro < 0) Gyro += 360;
    if (Gyro > 359) Gyro -= 360;
  }
  return Gyro;
}

void MPU6050_IMU::start() {
  mpu.initialize();
  if (mpu.testConnection() != true) {
    while (true) {
    }
  }
  if (mpu.dmpInitialize() != 0) {
    while (true) {
    }
  }
  mpu.setXGyroOffset(GY_x);
  mpu.setYGyroOffset(GY_y);
  mpu.setZGyroOffset(GY_z);
  mpu.setZAccelOffset(GY_accel);
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();
}
