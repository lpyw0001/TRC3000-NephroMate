// TRC3000 - NephroMate
// Slave 4

#include <Wire.h>
// ---------------- //
// GLOBAL VARIABLES //
// ---------------- //

// Analogue Pins
int mixerSpeedPin = A0;
int deaeratorSpeedPin = A1;
int heaterSpeedPin = A2;

// Analogue Values
double mixerSpeedVal = 0;
double deaeratorSpeedVal = 0;
double heaterSpeedVal = 0;

// ----------- //
// SETUP LOOP  //
// ----------- //
void setup() {
  Wire.begin();
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
