// TRC3000 - NephroMate
// Slave 3

#include <Wire.h>
// ---------------- //
// GLOBAL VARIABLES //
// ---------------- //

// Analogue Pins
int dialLevelPin = A0;
int waterLevelPin = A1;
int bloodPumpSpeedPin = A2;
int hepPumpSpeedPin = A3;

// Analogue Values
double dialLevelVal = 0;
double waterLevelVal = 0;
double bloodPumpSpeedVal = 0;
double hepPumpSpeedVal = 0;

// ----------- //
// SETUP LOOP  //
// ----------- //
void setup() {
  Wire.begin(0x03); // join I2C bus as slave with address 0x03
  Wire.onRequest(IOSend);
}

// ---------- //
// MAIN LOOP  //
// ---------- //

void loop() {
  
}

// ---------- //
// FUNCTIONS  //
// ---------- //
void IOSend(){
  Wire.write(** TO DO **);
}
