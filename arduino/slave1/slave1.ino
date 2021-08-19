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
unsigned long currentTime;
unsigned long prevTime;
unsigned long cyclePeriod = 300; // time in ms to alternate the screen values
bool cycle = true;
bool firstLoop = true;

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

  //displayUpdate();
  if (firstLoop) {
    displayUpdate("Ven Temp: ", venTempVal, "Ven Press: ", venPressVal);
    firstLoop = false;
  }
  currentTime = millis();
  if (currentTime - prevTime > cyclePeriod) {
    if (cycle) {
      displayUpdate("Ven Temp: ", venTempVal, "Ven Press: ", venPressVal);
    } else {
      displayUpdate("Wst Press: ", wastePressVal, "Wst Level: ", wasteLevelVal);
    }
    prevTime = currentTime;
    cycle = !cycle;
  }
}

// ---------- //
// FUNCTIONS  //
// ---------- //
// Transmit IO Data when requested by master
void IOSend() {
  I2C_writeAnything(venTempVal);
  I2C_writeAnything(venPressVal);
  I2C_writeAnything(wastePressVal);
  I2C_writeAnything(wasteLevelVal);
}

// Update Screen

void displayUpdate(String text1, double value1, String text2, double value2) {
  lcd.setCursor(0, 0);
  lcd.print(text1);
  lcd.print(value1);
  lcd.setCursor(0, 1);
  lcd.print(text2);
  lcd.print(value2);
}
