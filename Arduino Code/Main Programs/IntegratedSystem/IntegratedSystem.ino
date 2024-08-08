#include <Wire.h>
#include <Valve.h>
#include <IMU.h>
#include <Axis.h>
#include <PID_v1.h>
#include <SPI.h>
#include <SD.h>




unsigned long currentTime = 0;

float zStateVector[2] = {0, 0};
float omegaZ;
float targetOmegaZ = 0.0;
float throttleZ;

// PID parameters
double Kp = 1.0, Ki = 0.5, Kd = 0.1;
double input, output, setpoint;

PID pidZ(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

IMU imu;

int autoFreq = 20;

Valve Zminus(9, 2, 4, autoFreq);
Valve Zplus(10, 3, 5, autoFreq);
Valve* Valves[2] = {&Zplus, &Zminus};

Axis zAxis(Valves);

const int chipSelect = 4;
File dataFile;
String fileName;


void setup() {
  Serial.begin(9600);
  while (!Serial){
    ; // wait for the serial port to open
  }
  Serial.print("Initialising SD Card...");

  if (!SD.begin(chipSelect)){
    Serial.println("Initialization failed");
    return;
  }
  Serial.println("Done.");

  fileName = "data1.csv";

  // opening the file
  dataFile = SD.open(fileName, FILE_WRITE);

  // if the file opened then write to it
  if (dataFile){
    Serial.print("Writing to ");
    Serial.println(fileName);
    dataFile.print("Time,omegaZ,Throttle"); // writing CSV header
    dataFile.close();
  }
  else{
    Serial.print("Error: Could not open ");
    Serial.println(fileName);
  }


  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  imu.initalise();
  delay(100);
  imu.gyroCalibrate();

  setpoint = targetOmegaZ;
  pidZ.SetMode(AUTOMATIC);
  pidZ.SetOutputLimits(-100, 100);
}

void loop() {
  currentTime = millis();
  imu.measure(currentTime);

  imu.getZStateVector(zStateVector); // theta, omegaZ
  omegaZ = zStateVector[1];

  input = omegaZ;
  pidZ.Compute();

  throttleZ = output;

  zAxis.setThrottle(throttleZ, currentTime);

  Serial.print("omegaZ: ");
  Serial.print(omegaZ);
  Serial.print(" Throttle: ");
  Serial.println(throttleZ);


  // logging data to SD card
  dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile){
    dataFile.print(currentTime);
    dataFile.print(",");
    dataFile.print(omegaZ);
    dataFile.print(",");
    dataFile.println(throttleZ);
    dataFile.flush(); // make sure data is written to SD card
    dataFile.close(); // close file after writing
  }
  else{
    Serial.println("Error: Could not write to file");
  }

  delay(500);
}
