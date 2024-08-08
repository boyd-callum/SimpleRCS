#ifndef KALMAN_H
#define KALMAN_H

#include <Arduino.h>

class Kalman {
    public:
        Kalman();
        float getAngle(float newAngle, float newRate, float dt);

        void setAngle(float angle);

    private:
        float _rate;
        float _angle; // angle calculated by the kalman filter
        float _bias; // gyro bias calculated by the kalman filter
        float _P[2][2]; // error covariance matrix - 2x2

        float _Q_angle; // process noise variance for the accelerometer
        float _Q_bias; // process noise variance for the gyro bias
        float _R_measure; // measurement noise variance
};

#endif
