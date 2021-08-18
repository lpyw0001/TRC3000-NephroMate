// TRC3000 - NephroMate
// Master

#include <Wire.h>
// ---------------- //
// GLOBAL VARIABLES //
// ---------------- //

// Analogue Pins
int artPressPin = A0;
int artTempPin = A1;
int hepLevelPin = A2;
int inflowPressPin = A3;

// Analogue Values
double artPressVal = 0;
double artTempVal = 0;
double hepLevelVal = 0;
double inflowPressVal = 0;

// Digital Pins
int bloodPumpStartPin = 2;
int hepPumpStartPin = 3;
int dialPumpStartPin = 4;
int venClampActivePin = 5;
int airDetectTXPin = 6;
int airDetectRXPin = 7;
int dialClampActivePin = 8;
int bloodLeakPin = 9;
int wastePumpStartPin = 10;
int mixerStartPin = 11;
int deaeratorStartPin = 12;
int heaterStartPin = 13;

// From Slave 1
double venTempVal_S1 = 0;
double venPressVal_S1 = 0;
double wastePressVal_S1 = 0;
double wasteLevelVal_S1 = 0;

// From Slave 2
double dialConductivityVal_S2 = 0;
double pHVal_S2 = 0;
double dialTempVal_S2 = 0;
double dialPressVal_S2 = 0;

// From Slave 3
double dialLevelVal_S3 = 0;
double waterLevelVal_S3 = 0;
double bloodPumpSpeedVal_S3 = 0;
double hepPumpSpeedVal_S3 = 0;

// ----------- //
// SETUP LOOP  //
// ----------- //
void setup() {
  Wire.begin(); // join I2C bus (address optional for master device)
  Serial.begin(9600);
  pinMode(bloodPumpStartPin, OUTPUT);
  pinMode(hepPumpStartPin, OUTPUT);
  pinMode(dialPumpStartPin, OUTPUT);
  pinMode(venClampActivePin, OUTPUT);
  pinMode(airDetectTXPin, OUTPUT);
  pinMode(airDetectRXPin, INPUT);
  pinMode(dialClampActivePin, OUTPUT);
  pinMode(bloodLeakPin, INPUT);
  pinMode(wastePumpStartPin, OUTPUT);
  pinMode(mixerStartPin, OUTPUT);
  pinMode(deaeratorStartPin, OUTPUT);
  pinMode(heaterStartPin, OUTPUT);
}

// ---------- //
// MAIN LOOP  //
// ---------- //
// Wire.beginTransmission()
// Wire.write()
// Wire.endTransmission()

void loop() {
  // Request IO from slave 0x01 (4 x analogue values)
  Wire.requestFrom(0x01,16); 
  while(Wire.available()){ // change from while loop to if statement?
    // ** NEEDS TESTING **
    ((byte *)&venTempVal_S1)[0]=Wire.read(); // Wire.read reads one byte (double = 4 bytes)
    ((byte *)&venTempVal_S1)[1]=Wire.read();
    ((byte *)&venTempVal_S1)[2]=Wire.read();
    ((byte *)&venTempVal_S1)[3]=Wire.read();
    ((byte *)&venPressVal_S1)[0]=Wire.read();
    ((byte *)&venPressVal_S1)[1]=Wire.read();
    ((byte *)&venPressVal_S1)[2]=Wire.read();
    ((byte *)&venPressVal_S1)[3]=Wire.read();
    ((byte *)&wastePressVal_S1)[0]=Wire.read();
    ((byte *)&wastePressVal_S1)[1]=Wire.read();
    ((byte *)&wastePressVal_S1)[2]=Wire.read();
    ((byte *)&wastePressVal_S1)[3]=Wire.read();
    ((byte *)&wasteLevelVal_S1)[0]=Wire.read();
    ((byte *)&wasteLevelVal_S1)[1]=Wire.read();
    ((byte *)&wasteLevelVal_S1)[2]=Wire.read();
    ((byte *)&wasteLevelVal_S1)[3]=Wire.read();
    // Add actions here
  }

  // Request IO from slave 0x02
  Wire.requestFrom(0x02,** TO DO **); 
  while(Wire.available()){
    // Add actions here
  }
  
  // Request IO from slave 0x03
  Wire.requestFrom(0x03,** TO DO **); 
  while(Wire.available()){
    // Add actions here
  }
   // Request IO from slave 0x04
  Wire.requestFrom(0x04,** TO DO **);
  while(Wire.available()){
    // Add actions here
  }
}
