#include "Valve.h"

Valve::Valve(const int valvePin, const int triggerPin, const int autoPin, const int autoFreq) {
  _valvePin = valvePin;
  _triggerPin = triggerPin;
  _autoPin = autoPin;
  _autoFreq = autoFreq;

  _triggerVal = 0;
  _prevTriggerVal = 0;
  _autoVal = 0;
  _prevAutoVal = 0;

  _autoOn = false;
  _valveOpen = false;
  _prevTime = 0;

  pwmStartTime = millis();
  pwmFreq = 5; // in Hz
  pwmPeriod = 1000 / pwmFreq; // period in ms

  pinMode(_valvePin, OUTPUT);
  pinMode(_triggerPin, INPUT);
  pinMode(_autoPin, INPUT);
}

void Valve::open(){
  digitalWrite(_valvePin, HIGH);
  _valveOpen = true;
}

void Valve::close(){
  digitalWrite(_valvePin, LOW);
  _valveOpen = false;
}

void Valve::toggleOpen(){
  _triggerVal = digitalRead(_triggerPin);
  //Serial.println(_triggerVal);
  if (_triggerVal != _prevTriggerVal && _triggerVal == HIGH){
    if (_valveOpen){
      close();
    }
    else{
      open();
    }
  }
  _prevTriggerVal = _triggerVal;
}


void Valve::autoOpen(unsigned long currentTime){
  if (_autoOn && currentTime - _prevTime > 500/_autoFreq){
    //Serial.println(currentTime - _prevTime);
    //Serial.println(currentTime);
    if (_valveOpen){
      close();
    }
    else{
      open();
    }
    _prevTime = currentTime;
  }
}


void Valve::checkAuto(){
    _autoVal = digitalRead(_autoPin);
  
    if (_autoVal != _prevAutoVal && _autoVal == HIGH){
    _autoOn = !_autoOn;
    }
    //Serial.println(_autoOn);
    _prevAutoVal = _autoVal;
}


void Valve::customPWM( int throttle, unsigned long currentTime){
  
  // time since valve opened
  unsigned long dt = currentTime - pwmStartTime;

  // constrain throttle between 0% and 100%
  throttle = constrain(throttle, 0, 100);
  // if throttle isnt 0 or 100, constrain it to 13%-87%
  if (throttle > 0 && throttle < 100){
    throttle = constrain(throttle, 13, 87);
  }
  
  int onTime = pwmPeriod * throttle / 100; // time the valve is open for
  int offTime = pwmPeriod - onTime; // time the valve is closed for


  // if throttle is 0% then keep valve closed
  if (throttle ==0 && _valveOpen == true){
    if (_valveOpen == true){
      close();
    }
  }
  // if throttle is 100% then keep it open
  else if (throttle == 100){
    if (_valveOpen == false){
      open();
    }

  }
  else {
    // if the time since opening the valve is still less than the on time, open valve
    if (dt < onTime){
      if (_valveOpen == false){
        open();
        //Serial.print("open: ");
        //Serial.println(currentTime);
      }
      
    }
    // if it is more than the on time but less than the pwm period, close valve
    else if (dt < pwmPeriod){
      if (_valveOpen == true){
        close();
        //Serial.print("close: ");
        //Serial.println(currentTime);
      }
    }
    else {
      pwmStartTime = currentTime;
    }
  }
}
  


bool Valve::isAutoOn() const {return _autoOn;}
bool Valve::isValveOpen() const {return _valveOpen;}

void Valve::setAutoOn(bool autoOn) {_autoOn = autoOn;}

