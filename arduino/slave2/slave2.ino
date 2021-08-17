// TRC3000 - NephroMate
// Slave 2

#include <Wire.h>
// ---------------- //
// GLOBAL VARIABLES //
// ---------------- //

// Analogue Pins
int dialConductivityPin = A0;
int pHPin = A1;
int dialTempPin = A2;
int dialPressPin = A3;

// Analogue Values
double dialConductivityVal = 0;
double pHVal = 0;
double dialTempVal = 0;
double dialPressVal = 0;

// ----------- //
// SETUP LOOP  //
// ----------- //
void setup() {
  Wire.begin(0x02); // join I2C bus as slave with address 0x02
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
