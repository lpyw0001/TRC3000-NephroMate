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

double IOBusDouble[4] = {0.0,0.0,0.0,0.0};

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
  IOBusDouble[0] = venTempVal;
  IOBusDouble[1] = venPressVal;
  IOBusDouble[2] = wastePressVal;
  IOBusDouble[3] = wasteLevelVal;
  delay(100); // Scan for IO changes every 100ms
}

// ---------- //
// FUNCTIONS  //
// ---------- //
// Transmit IO Data when requested by master
void IOSend(){
  byte *IOBusByte = (byte *)IOBusDouble; // Cast to byte for transmission on I2C Bus
  Wire.write(IOBusByte, 16); // Double = 4 bytes hence 4 doubles = 16 bytes
  
  // If the 16 byte write does not work alternative is to write byte by byte as below
  /*for(int i =0; i<4*sizeof(IOBusDouble); i++){
    Wire.write(IOBusByte[i]); // 
  }*/ 
}
