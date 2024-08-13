#include <Kalman.h>

Kalman::Kalman()
{
    _rate = 0;
    _angle = 0;
    _bias = 0;

    // determined with matlab simulations
    _Q_angle = 0.002;   // original: 0.001
    _Q_bias = 0.01;     // original: 0.003
    _R_measure = 0.001; // original: 0.03

    _P[0][0] = 0;
    _P[0][1] = 0;
    _P[1][0] = 0;
    _P[1][1] = 0;
}

float Kalman::getAngle(float newAngle, float newRate, float dt)
{

    // prediction step
    _rate = newRate - _bias;
    _angle += dt * _rate;
    /*
    Serial.println("");
    Serial.print("_rate = "); Serial.println(_rate, 5);
    Serial.print("_angle = "); Serial.println(_angle, 5);
    */
    // update estimation error covariance - project the error covariance ahead
    _P[0][0] += dt * (dt * _P[1][1] - _P[0][1] - _P[1][0] + _Q_angle);
    _P[0][1] -= dt * _P[1][1];
    _P[1][0] -= dt * _P[1][1];
    _P[1][1] += _Q_bias * dt;

    // measurement update (update step)
    float y = newAngle - _angle;     // difference between mag reading and dead reckoning (innovation)
    float S = _P[0][0] + _R_measure; // innovation covariance
    float K[2];                      // kalman gain - 2x1 vector
    K[0] = _P[0][0] / S;
    K[1] = _P[1][0] / S;
    /*
    Serial.print("K0: "); Serial.println(K[0], 5);
    Serial.print("K1: "); Serial.println(K[1], 5);

    Serial.print("y "); Serial.println(y, 5);
    Serial.print("s "); Serial.println(S, 5);
    */

    // update the angle and bias - update with the mag reading
    _angle += K[0] * y;
    _bias += K[1] * y;

    /*
    Serial.print("_angle "); Serial.println(_angle, 5);
    Serial.print("_bias "); Serial.println(_bias, 5);
    */
    // update the error covariance - update the error covariance matrix
    float P00_temp = _P[0][0];
    float P01_temp = _P[0][1];

    _P[0][0] -= K[0] * _P[0][0];
    _P[0][1] -= K[0] * _P[0][1];
    _P[1][0] -= K[1] * _P[0][0];
    _P[1][1] -= K[1] * _P[0][1];

    /*
    Serial.print("P00: "); Serial.println(_P[0][0], 5);
    Serial.print("P01: "); Serial.println(_P[0][1], 5);
    Serial.print("P10: "); Serial.println(_P[1][0], 5);
    Serial.print("P11: "); Serial.println(_P[1][1], 5);
    */
    return _angle;
}

void Kalman::setAngle(float angle)
{
    _angle = angle;
}