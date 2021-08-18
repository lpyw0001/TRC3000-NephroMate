// TRC3000 - NephroMate
// Slave 1
#include <Wire.h>
#include <LiquidCrystal.h>

// Source: https://github.com/nickgammon/I2C_Anything/blob/master/I2C_Anything.h
template <typename T> unsigned int I2C_writeAnything (const T& value)
{
  Wire.write((byte *) &value, sizeof (value));
  return sizeof (value);
}  // end of I2C_writeAnything

template <typename T> unsigned int I2C_readAnything(T& value)
{
  byte * p = (byte*) &value;
  unsigned int i;
  for (i = 0; i < sizeof value; i++)
    *p++ = Wire.read();
  return i;
}  // end of I2C_readAnything

// Liquid crystal display code adapted from: http://www.arduino.cc/en/Tutorial/LiquidCrystal
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

double IOBusDouble[4] = {0.0, 0.0, 0.0, 0.0};

// Initialise LCD
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// ----------- //
// SETUP LOOP  //
// ----------- //
void setup() {
  Wire.begin(0x10); // join I2C bus as slave with address 0x10
  Wire.onRequest(IOSend);
  lcd.begin(16, 2);
}

// ---------- //
// MAIN LOOP  //
// ---------- //

void loop() {
  venTempVal = analogRead(venTempPin);
  venPressVal = analogRead(venPressPin);
  wastePressVal = analogRead(wastePressPin);
  wasteLevelVal = analogRead(wasteLevelPin);

  IOBusDouble[0] = venTempVal;
  IOBusDouble[1] = venPressVal;
  IOBusDouble[2] = wastePressVal;
  IOBusDouble[3] = wasteLevelVal;

  displayUpdate();
  //delay(100); // Scan for IO changes every 100ms
}

// ---------- //
// FUNCTIONS  //
// ---------- //
// Transmit IO Data when requested by master
void IOSend() {
  I2C_writeAnything(venTempVal); // NOT WORKING
  /*I2C_writeAnything(venPressVal);
  I2C_writeAnything(wastePressVal);
  I2C_writeAnything(wasteLevelVal);*/
  
  //I2C_writeAnything(IOBusDouble);
  
}

// Update Screen  
void displayUpdate() {
  lcd.setCursor(0, 0);
  lcd.print("Ven Temp:  ");
  lcd.print(venTempVal);
  lcd.setCursor(0, 1);
  lcd.print("Ven Press: ");
  lcd.print(venPressVal);

  delay(100); // change to non-blocking millis?

  lcd.setCursor(0, 0);
  lcd.print("Waste Press: ");
  lcd.print(wastePressVal);
  lcd.setCursor(0, 1);
  lcd.print("Waste Level: ");
  lcd.print(wasteLevelVal);
  delay(100);
}
