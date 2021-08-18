// TRC3000 - NephroMate
// Master
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
int artPressPin = A0;
int artTempPin = A1;
int hepLevelPin = A2;
int inflowPressPin = A3;

// Analogue Values
double artPressVal = 0;
double artTempVal = 0;
double hepLevelVal = 0;
double inflowPressVal = 0;

double IOBusDouble[4] = {0.0, 0.0, 0.0, 0.0};

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

// Setpoints
/*const double PRESSURE_L_SCL;
const double PRESSURE_H_SCL;
const double PRESSURE_H_SP;
const double PRESSURE_L_SP;*/

// Initialise LCD
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

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
  lcd.begin(16, 2);
}

// ---------- //
// MAIN LOOP  //
// ---------- //
// Wire.beginTransmission()
// Wire.write()
// Wire.endTransmission()

void loop() {

  // Read Analogue Inputs
  artPressVal = analogRead(artPressPin);
  artTempVal = analogRead(artTempPin);
  hepLevelVal = analogRead(hepLevelPin);
  inflowPressVal = analogRead(inflowPressPin);
  displayUpdate();

  Wire.requestFrom(0x10,4);
  I2C_readAnything(venTempVal_S1); // NOT WORKING
  /*I2C_readAnything(venPressVal_S1);
  I2C_readAnything(wastePressVal_S1);
  I2C_readAnything(wasteLevelVal_S1);*/

  // Request IO from slave 0x01 (4 x analogue values)
  /*Wire.requestFrom(0x01,16);
    I2C_readAnything(venTempVal_S1);
    I2C_readAnything(venPressVal_S1);
    I2C_readAnything(wastePressVal_S1);
    I2C_readAnything(wasteLevelVal_S1);

    //Scale Values

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
    }*/
}

// -------------- //
// Update Screen  //
// ------------- //
void displayUpdate() {
  lcd.setCursor(0, 0);
  lcd.print("Art Press: ");
  lcd.print(artPressVal);
  lcd.setCursor(0, 1);
  lcd.print("Art Temp:  ");
  lcd.print(artTempVal);

  delay(100);

  lcd.setCursor(0, 0);
  lcd.print("Hep Level:  ");
  lcd.print(hepLevelVal);
  lcd.setCursor(0, 1);
  lcd.print("Infl Press: ");
  lcd.print(inflowPressVal);
  delay(100);
}
