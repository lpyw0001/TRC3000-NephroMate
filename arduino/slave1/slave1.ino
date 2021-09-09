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
int bloodFlowPin = A3;

// Analogue Values
double dialConductivityVal = 0;
double pHVal = 0;
double dialTempVal = 0;
double bloodFlowVal = 0;

double dialConductivityScl = 0;
double pHScl = 0;
double dialTempScl = 0;
double bloodFlowScl = 0;


// Fault Conditions
bool bloodPumpFault = false;
bool clampLines = false;

// Initialise LCD
LiquidCrystal lcd(12, 13, 2, 3, 4, 5); // (rs,enable,d4,d5,d6,d7)
unsigned long currentTime;
unsigned long prevTime;
unsigned long cyclePeriod = 100; // time in ms to alternate the screen values
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
int mixerIN2Pin = 11;

// ----------- //
// SETUP LOOP  //
// ----------- //
void setup() {
  Wire.begin(0x10); // join I2C bus as slave with address 0x10
  Wire.onRequest(IOSend);
  dialysateClamp.write(0);
  dialysateClamp.attach(9);
  venousClamp.write(0);
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
    bloodFlowVal = analogRead(bloodFlowPin);

    // Scale Analogue Inputs for Transmission to Master
    dialConductivityScl = scaleInput(dialConductivityVal, 0, 1023, 10.0, 30.0);
    pHScl = scaleInput(pHVal, 0, 1023, 0.0, 14.0);
    dialTempScl = scaleInput(dialTempVal, 0, 250, 5.0, 60.0); // Max raw value is 358?? - TBC
    bloodFlowScl = scaleInput(bloodFlowVal, 0, 1023, 0.0, 400.0); // TO DO UPDATE VALUE

    currentTime = millis();
    if (currentTime - prevTime > cyclePeriod) {
      if (cycle) {
        displayUpdate("Dial Cond", dialConductivityScl, "pH", pHScl);
      } else {
        displayUpdate(" Dial Temp", dialTempScl, "Dial Press", bloodFlowScl);
      }
      prevTime = currentTime;
      cycle = !cycle;
    }

    // Mixer runs at a fixed speed continuously
    digitalWrite(mixerIN1Pin, HIGH);
    digitalWrite(mixerIN2Pin, LOW);

    // Dialysate pump runs at a fixed speed continuously
    digitalWrite(dialPumpIN1Pin, HIGH);
    digitalWrite(dialPumpIN2Pin, LOW);

    // Blood pump PID controlled (against what?)
    // Currently just start/stop based on fault conditions
    if (bloodPumpFault) {
      digitalWrite(bloodPumpIN1Pin, LOW);
      digitalWrite(bloodPumpIN2Pin, LOW);
    } else {
      digitalWrite(bloodPumpIN1Pin, HIGH);
      digitalWrite(bloodPumpIN2Pin, LOW);
    }

    // Dialysate clamp: triggered when air is detected?
    // Venous clamp: triggered when air is detected?
    if (clampLines) {
      venousClamp.write(CLAMP_ANGLE);
      dialysateClamp.write(CLAMP_ANGLE);
    }
  }

  lcd.clear();

  // Stop all motors
  digitalWrite(mixerIN1Pin, LOW);
  digitalWrite(mixerIN2Pin, LOW);
  digitalWrite(dialPumpIN1Pin, LOW);
  digitalWrite(dialPumpIN2Pin, LOW);


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
  I2C_writeAnything(bloodFlowScl);
}

void MasterControl(int dataSize) {
  I2C_readAnything(startCommand);
  I2C_readAnything(bloodPumpFault);
  I2C_readAnything(clampLines);
}

// Update Screen
void displayUpdate(String text1, double value1, String text2, double value2) {
  lcd.clear(); // clear screen and set cursor to (0,0)

  unsigned int strLen1 = text1.length();
  unsigned int strLen2 = text2.length();
  String outputText1 = "";
  String outputText2 = "";

  if ((10 - strLen1) > 0) {
    for (int i = 0; i < (10 - strLen1); i++) {
      outputText1 += " ";
    }
  }
  outputText1 = outputText1 + text1 + ": ";

  if ((10 - strLen2) > 0) {
    for (int i = 0; i < (10 - strLen2); i++) {
      outputText2 += " ";
    }
  }
  outputText2 = outputText2 + text2 + ": ";

  lcd.print(outputText1);
  lcd.print(value1);
  lcd.setCursor(0, 1);
  lcd.print(outputText2);
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
