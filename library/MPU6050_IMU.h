#ifndef _MPU6050_IMU_h
#define _MPU6050_IMU_h

class MPU6050_IMU{
  public:
    MPU6050_IMU();

  public:
    void start();
    void offset(int x,int y,int z,int accel);
    int getAngle();

  private:
};

#endif