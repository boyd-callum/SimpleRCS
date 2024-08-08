#include "Axis.h"

Axis::Axis(Valve* valveArray[2])
  : _axisPID(&_PIDInput, &_PIDOutput, &_PIDSetpoint, 0, 0, 0, DIRECT) { // Properly initialize PID object
    _axisValves[0] = valveArray[0];
    _axisValves[1] = valveArray[1];
}

void Axis::initalisePID(float Kp, float Ki, float Kd){
    _axisPID.SetTunings(Kp, Ki, Kd); // Use SetTunings to configure PID parameters
}

void Axis::setThrottle(int throttle, unsigned long currentTime){
    // throttle value between -100 and 100
    if (throttle > 0){
        _axisValves[0]->customPWM(throttle, currentTime);
        _axisValves[1]->customPWM(0, currentTime);
    }
    else{
        _axisValves[0]->customPWM(0, currentTime);
        _axisValves[1]->customPWM(-throttle, currentTime);
    }
}





