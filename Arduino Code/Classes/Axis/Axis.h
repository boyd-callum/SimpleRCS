#ifndef AXIS_H
#define AXIS_H

#include <Arduino.h>
#include <Valve.h>
#include <PID_v1.h>

class Axis {
public:
    Axis(Valve* valveArray[2]);

    void initalisePID(float Kp, float Ki, float Kd);
    void updateStateVector(float (& _axisStateVector) [2] );

    void setThrottle(int throttle, unsigned long currentTime);

private:
    Valve* _axisValves[2];
    float _axisStateVector[2];
    double _PIDInput, _PIDOutput, _PIDSetpoint;

    PID _axisPID;

};

#endif