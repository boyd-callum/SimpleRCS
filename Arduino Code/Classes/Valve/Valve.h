#ifndef VALVE_H
#define VALVE_H

#include <Arduino.h>

class Valve
{
public:
  Valve(const int valvePin, const int triggerPin, const int autoPin, const int autoFreq);
  void open();
  void close();
  void toggleOpen();
  void autoOpen(unsigned long currentTime);
  void checkAuto();
  void customPWM(int throttle, unsigned long currentTime);

  bool isAutoOn() const;
  bool isValveOpen() const;

  void setAutoOn(bool autoOn);

private:
  int _valvePin;
  int _triggerPin;
  int _autoPin;
  int _autoFreq;

  int _triggerVal;
  int _prevTriggerVal;
  int _autoVal;
  int _prevAutoVal;

  bool _autoOn;
  bool _valveOpen;
  unsigned long _prevTime;

  unsigned long pwmStartTime;
  int pwmFreq;
  int pwmPeriod;
};

#endif
