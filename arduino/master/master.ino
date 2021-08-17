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
  // Request IO from slave 0x02
  Wire.requestFrom(2,** TO DO **); 
  while(Wire.available()){
    // Add actions here
  }

  // Request IO from slave 0x03
  Wire.requestFrom(3,** TO DO **); 
  while(Wire.available()){
    // Add actions here
  }
  
  // Request IO from slave 0x04
  Wire.requestFrom(4,** TO DO **); 
  while(Wire.available()){
    // Add actions here
  }
   // Request IO from slave 0x05
  Wire.requestFrom(5,** TO DO **);
  while(Wire.available()){
    // Add actions here
  }
}
