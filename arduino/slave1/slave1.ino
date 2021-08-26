// TRC3000 - NephroMate
// Slave 1
#include <Wire.h>
#include <LiquidCrystal.h>
#include <Servo.h>

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
int dialConductivityPin = A0;
int pHPin = A1;
int dialTempPin = A2;
int dialPressPin = A3;

// Analogue Values
double dialConductivityVal = 0;
double pHVal = 0;
double dialTempVal = 0;
double dialPressVal = 0;

double dialConductivityScl = 0;
double pHScl = 0;
double dialTempScl = 0;
double dialPressScl = 0;

// Initialise LCD
LiquidCrystal lcd(12, 11, 2, 3, 4, 5); // (rs,enable,d4,d5,d6,d7)
unsigned long currentTime;
unsigned long prevTime;
unsigned long cyclePeriod = 105; // time in ms to alternate the screen values
bool cycle = true;

// Output Objects
Servo dialysateClamp;
Servo venousClamp;
const int CLAMP_ANGLE = 90;
const int CLAMP_OFF = 0;
bool startCommand = false; // From Master

// Digital Pins
int mixerIN1Pin = 0;
int bloodPumpIN2Pin = 1;
int bloodPumpIN1Pin = 6;
int dialPumpIN1Pin = 7;
int dialPumpIN2Pin = 8;
int dialClampActivePin = 9;
int venClampActivePin = 10;
int mixerIN2Pin = 13;

// ----------- //
// SETUP LOOP  //
// ----------- //
void setup() {
  Wire.begin(0x10); // join I2C bus as slave with address 0x10
  Wire.onRequest(IOSend);
  dialysateClamp.attach(9);
  venousClamp.attach(10);
  Wire.onReceive(MasterControl);
  pinMode(mixerIN1Pin, OUTPUT);
  pinMode(bloodPumpIN2Pin, OUTPUT);
  pinMode(bloodPumpIN1Pin, OUTPUT);
  pinMode(dialPumpIN1Pin, OUTPUT);
  pinMode(dialPumpIN2Pin, OUTPUT);
  pinMode(dialClampActivePin, OUTPUT);
  pinMode(venClampActivePin, OUTPUT);
  pinMode(mixerIN2Pin, OUTPUT);
  lcd.begin(16, 2);
}

// ---------- //
// MAIN LOOP  //
// ---------- //

void loop() {
  while (startCommand) {
    // Read Analogue Inputs
    dialConductivityVal = analogRead(dialConductivityPin);
    pHVal = analogRead(pHPin);
    dialTempVal = analogRead(dialTempPin);
    dialPressVal = analogRead(dialPressPin);

    // Scale Analogue Inputs for Transmission to Master
    dialConductivityScl = scaleInput(dialConductivityVal, 0, 1023, 10.0, 30.0);
    pHScl = scaleInput(pHVal, 0, 1023, 0.0, 14.0);
    dialTempScl = scaleInput(dialTempVal, 0, 1023, 5.0, 60.0);
    dialPressScl = scaleInput(dialPressVal, 0, 1023, 12.7, 127.0); // TO DO UPDATE VALUE

    //displayUpdate();
    /*if (firstLoop) {
      displayUpdate("Dial Cond: ", dialConductivityScl, "       pH: ", pHScl);
      firstLoop = false;
      }*/

    currentTime = millis();
    if (currentTime - prevTime > cyclePeriod) {
      if (cycle) {
        displayUpdate("Dial Cond: ", dialConductivityScl, "       pH: ", pHScl);
        dialysateClamp.write(CLAMP_OFF);
        venousClamp.write(CLAMP_ANGLE);
      } else {
        displayUpdate("Dial Temp: ", dialTempScl, "Dial Press: ", dialPressScl);
        dialysateClamp.write(CLAMP_ANGLE);
        venousClamp.write(CLAMP_OFF);
      }
      prevTime = currentTime;
      cycle = !cycle;
    }
    digitalWrite(bloodPumpIN1Pin, HIGH);
    digitalWrite(bloodPumpIN2Pin, LOW);
  }
  lcd.clear();
  // Stop all motors...
  // TO DO
  // Activate Clamp when requested by Master
  // dialysateClamp.write(CLAMP_ANGLE);
  // TO DO
}

// ---------- //
// FUNCTIONS  //
// ---------- //
// Transmit IO Data when requested by master
void IOSend() {
  I2C_writeAnything(dialConductivityScl);
  I2C_writeAnything(pHScl);
  I2C_writeAnything(dialTempScl);
  I2C_writeAnything(dialPressScl);
}

void MasterControl(int dataSize) {
  I2C_readAnything(startCommand);
}

// Update Screen
void displayUpdate(String text1, double value1, String text2, double value2) {
  lcd.clear(); // clear screen and set cursor to (0,0)
  lcd.print(text1);
  lcd.print(value1);
  lcd.setCursor(0, 1);
  lcd.print(text2);
  lcd.print(value2);
}

// Scale Analogue Input  //
// floating point version of the map() function
// y = ((y2-y1/(x2-x1))*(x-x1)+y1
// x1,x2 are the input min/max
// y1,y2 are the output min/max
// x is the value to scale
// y is the scaled value
double scaleInput(int rawValue, int rawMin, int rawMax, double scaledMin, double scaledMax) {
  return ((scaledMax - scaledMin) / (rawMax - rawMin)) * (rawValue - rawMin) + scaledMin;
}
