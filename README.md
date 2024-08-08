# SimpleRCS
A simple Reaction Control System using compressed gas thrusters for attitude control, running on Arduino.


The system gets its orientation through a MPU6050 Accelerometer/Gyroscope, and a HMC5883L magnetometer. A simple Kalman filter is used to 
filter the results from the gyroscope and magnetometer to give a better estimate of orientation. 

The system uses simple PID controllers to control two solenoid valves per axis of rotation, aiming for a set rotation rate. 