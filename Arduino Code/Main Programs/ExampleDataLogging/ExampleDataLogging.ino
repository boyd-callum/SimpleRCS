#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <IMU.h>
#include <Axis.h>
#include <Valve.h>

const int chipSelect = 4;
char fileName[] = "example.csv";

unsigned long currentTime;

float zStateVector[2] = {0,0};

IMU imu;
int autoFreq = 0;

Valve Zminus(9, 2, 4, autoFreq);
Valve Zplus(10, 3, 5, autoFreq);
Valve* ZValves[2] = {&Zplus, &Zminus};


Axis zAxis(ZValves);

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("Initialization done.");

  // Open the file. Note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(fileName, FILE_WRITE);

  // if the file opened okay, write to it:
  if (dataFile) {
    Serial.print("Writing to example.txt...");
    dataFile.println("Time,Value");
    dataFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("Error opening example.txt");
  }


  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  imu.initalise();
  delay(100);
  imu.gyroCalibrate();

}

void loop() {
  currentTime = millis();
  imu.measure(currentTime);

  imu.getZStateVector(zStateVector);

  File dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile){
    //dataFile.println("1,1");
    dataFile.print(currentTime);
    dataFile.print(",");
    dataFile.println(zStateVector[1]);
    dataFile.close();
    Serial.println("Wrote to file");
  }else{
    Serial.println("Could not write to file");
  }
  delay(50);
}
