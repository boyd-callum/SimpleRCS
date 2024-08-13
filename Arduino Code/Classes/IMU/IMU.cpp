#include <IMU.h>

IMU::IMU()
{
    _accelGyroAddress = 0x68; // MPU6050
    _magAddress = 0x1E;       // HMC5883L

    _gyroScaleFactor = 65.6;
    _accelScaleFactor = 4096.0;
    _magScaleFactor = 1090.0;
    _microTeslaPerGauss = 100.0;

    _gyroCalibrationX = 0;
    _gyroCalibrationY = 0;
    _gyroCalibrationZ = 0;

    _interval = 10; // milliseconds, 100hz
    _prevTime = 0;
    _dt = 0;
}

void IMU::writeToRegister(uint8_t address, uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void IMU::initalise()
{
    // Serial.println("Initialising Accel/Gyro");

    writeToRegister(_accelGyroAddress, 0x6B, 0x00); // setting IMU to power mode
    writeToRegister(_accelGyroAddress, 0x1A, 0x05); // low pass filter
    writeToRegister(_accelGyroAddress, 0x1B, 0x08); // gyro gain (65.6 LSB/degree/second)
    writeToRegister(_accelGyroAddress, 0x1C, 0x10); // accel gain (4096 LSB/g)

    // Serial.println("Initialising Mag");

    writeToRegister(_magAddress, 0x00, 0x70); // 8-average, 15Hz, and normal measurement
    writeToRegister(_magAddress, 0x01, 0x20); // gain set to 1.3 Ga (scale factor of 1090)
    writeToRegister(_magAddress, 0x02, 0x00); // continuous measurement mode

    // Serial.println("IMU Initialised");
}

void IMU::readMagData(uint8_t address)
{
    Wire.beginTransmission(address);
    Wire.write(0x03); // first data register
    Wire.endTransmission();
    Wire.requestFrom(address, 6);

    if (Wire.available() == 6)
    {
        _magX = Wire.read() << 8 | Wire.read(); // takes the first two sent packets (both uint8) and combines them into one int16
        _magZ = Wire.read() << 8 | Wire.read(); // same for Z
        _magY = Wire.read() << 8 | Wire.read(); // and Y

        _magX *= _microTeslaPerGauss / _magScaleFactor;
        _magY *= _microTeslaPerGauss / _magScaleFactor;
        _magZ *= _microTeslaPerGauss / _magScaleFactor;

        _magYaw = atan2(_magY, _magX) * (180.0 / PI);
        if (_magYaw < 0)
        {
            _magYaw += 360;
        }
        else if (_magYaw > 360)
        {
            _magYaw -= 360;
        }
    }
    else
    {
        Serial.println("Failed to read from mag sensor");
    }
}

void IMU::readAccelData(uint8_t address)
{
    Wire.beginTransmission(address);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(address, 6);

    if (Wire.available() == 6)
    {
        _accelX = Wire.read() << 8 | Wire.read();
        _accelY = Wire.read() << 8 | Wire.read();
        _accelZ = Wire.read() << 8 | Wire.read();

        _accelX /= _accelScaleFactor;
        _accelY /= _accelScaleFactor;
        _accelZ /= _accelScaleFactor;
    }
    else
    {
        Serial.println("Failed to read from accel sensor");
    }
}

void IMU::readGyroData(uint8_t address)
{
    Wire.beginTransmission(address);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(address, 6);

    if (Wire.available() == 6)
    {
        _gyroX = Wire.read() << 8 | Wire.read();
        _gyroY = Wire.read() << 8 | Wire.read();
        _gyroZ = Wire.read() << 8 | Wire.read();

        _gyroX /= _gyroScaleFactor;
        _gyroY /= _gyroScaleFactor;
        _gyroZ /= _gyroScaleFactor;

        _gyroX -= _gyroCalibrationX;
        _gyroY -= _gyroCalibrationY;
        _gyroZ -= _gyroCalibrationZ;
    }
    else
    {
        Serial.println("Failed to read from gyro sensor");
    }
}

void IMU::gyroCalibrate()
{
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    float tempGyroX = 0.0;
    float tempGyroY = 0.0;
    float tempGyroZ = 0.0;
    float tempMagX = 0.0;
    float tempMagY = 0.0;
    float tempMagZ = 0.0;
    float tempAngleYaw = 0.0;
    _calibrationLoops = 2000;

    for (_currentCalibrationLoop = 0; _currentCalibrationLoop < _calibrationLoops; _currentCalibrationLoop++)
    {
        readGyroData(_accelGyroAddress);
        tempGyroX += _gyroX;
        tempGyroY += _gyroY;
        tempGyroZ += _gyroZ;

        readMagData(_magAddress);

        tempAngleYaw += _magYaw;

        // Debugging: Print raw and scaled gyro values
        /*
        Serial.print("Raw Gyro X: "); Serial.println(_gyroX * _gyroScaleFactor);
        Serial.print("Raw Gyro Y: "); Serial.println(_gyroY * _gyroScaleFactor);
        Serial.print("Raw Gyro Z: "); Serial.println(_gyroZ * _gyroScaleFactor);
        Serial.print("Scaled Gyro X: "); Serial.println(_gyroX);
        Serial.print("Scaled Gyro Y: "); Serial.println(_gyroY);
        Serial.print("Scaled Gyro Z: "); Serial.println(_gyroZ);
        Serial.print("tempGyroX: "); Serial.println(tempGyroX);
        Serial.print("tempGyroY: "); Serial.println(tempGyroY);
        Serial.print("tempGyroZ: "); Serial.println(tempGyroZ);
        */
    }

    _gyroCalibrationX = tempGyroX / _calibrationLoops;
    _gyroCalibrationY = tempGyroY / _calibrationLoops;
    _gyroCalibrationZ = tempGyroZ / _calibrationLoops;

    _angleYaw = tempAngleYaw / _calibrationLoops;
    if (_angleYaw < 0)
    {
        _angleYaw += 360;
    }
    else if (_angleYaw > 360)
    {
        _angleYaw -= 360;
    }
    _gyroYaw = _angleYaw;
    _kalmanFilter.setAngle(_gyroYaw);
    /*
    Serial.println("Gyro Calibration Values:");
    Serial.print("X: "); Serial.println(_gyroCalibrationX);
    Serial.print("Y: "); Serial.println(_gyroCalibrationY);
    Serial.print("Z: "); Serial.println(_gyroCalibrationZ);
    Serial.print("Angle Yaw: "); Serial.println(_angleYaw);
    Serial.println("----------------");
    */

    digitalWrite(13, LOW);
}

void IMU::gyroDeadReckon()
{
    _gyroYaw -= _gyroZ * _dt;
}

void IMU::kalman()
{
    _kalmanYaw = _kalmanFilter.getAngle(_magYaw, _gyroZ, _dt);
}

void IMU::measure(unsigned long currentTime)
{
    _dt = (currentTime - _prevTime) / 1000.0;
    _prevTime = currentTime;
    // Serial.println(_dt);
    readMagData(_magAddress);
    readAccelData(_accelGyroAddress);
    readGyroData(_accelGyroAddress);
    gyroDeadReckon();
    kalman();
}

float IMU::getMagYaw() { return _magYaw; }

float IMU::getGyroYaw() { return _gyroYaw; }

float IMU::getKalmanYaw() { return _kalmanYaw; }

float IMU::getOmegaZ() { return _gyroZ; }

void IMU::getZStateVector(float (&zStateVector)[2])
{
    zStateVector[0] = _kalmanYaw;
    zStateVector[1] = _gyroZ;
}