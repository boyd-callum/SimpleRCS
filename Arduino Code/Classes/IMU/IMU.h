#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <Kalman.h>

class IMU
{
public:
    IMU();

    // doers
    void writeToRegister(uint8_t address, uint8_t reg, uint8_t value);
    void initalise(); // initialises the IMU with the specified settings
    void readMagData(uint8_t addres);
    void readAccelData(uint8_t address);
    void readGyroData(uint8_t address);
    void gyroCalibrate();                    // runs the gyro calibration process
    void measure(unsigned long currentTime); // takes measurements from sensors and updates all values
    void gyroDeadReckon();                   // sums the gyro measurements together to approximate orientation
    void kalman();

    // getters
    float getMagYaw();
    float getGyroYaw();
    float getKalmanYaw();

    float getOmegaZ();
    void getZStateVector(float (&zStateVector)[2]);

private:
    uint8_t _accelGyroAddress;
    uint8_t _magAddress;

    float _gyroScaleFactor;
    float _accelScaleFactor;
    float _magScaleFactor;
    float _microTeslaPerGauss;

    float _gyroX, _gyroY, _gyroZ;
    float _accelX, _accelY, _accelZ;
    float _magX, _magY, _magZ;
    float _angleRoll, _anglePitch, _angleYaw;

    float _magYaw;
    float _gyroRoll, _gyroPitch, _gyroYaw;

    int _interval;
    unsigned long _prevTime;
    float _dt; // small step in time between measurements

    float _gyroCalibrationX, _gyroCalibrationY, _gyroCalibrationZ;
    int _calibrationLoops;
    int _currentCalibrationLoop;

    Kalman _kalmanFilter;
    float _kalmanYaw;
};

#endif