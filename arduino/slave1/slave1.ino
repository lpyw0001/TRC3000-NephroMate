// TRC3000 - NephroMate
// Slave 1

#include <Wire.h>
// ---------------- //
// GLOBAL VARIABLES //
// ---------------- //

// Analogue Pins
int venTempPin = A0;
int venPressPin = A1;
int wastePressPin = A2;
int wasteLevelPin = A3;

// Analogue Values
double venTempVal = 0;
double venPressVal = 0;
double wastePressVal = 0;
double wasteLevelVal = 0;

// ----------- //
// SETUP LOOP  //
// ----------- //
void setup() {
  Wire.begin(0x01); // join I2C bus as slave with address 0x01
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
