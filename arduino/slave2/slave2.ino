// TRC3000 - NephroMate
// Slave 2
#include <Wire.h>
#include <Servo.h>
#define FLOAT_SCALE 100

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
int waterLevelPin = A0;
int venTempPin = A1;
int bloodLeakPin = A2;
int dialLevelPin = A3;

// Analogue Values
long waterLevelVal = 0;
long venTempVal = 0;
long bloodLeakVal = 0;
long dialLevelVal = 0;

// Scaled Analogue Values
long waterLevelValScl = 0;
long venTempValScl = 0;
long bloodLeakValScl = 0;
long dialLevelValScl = 0;

// Digital Pins
int heaterEncA = 2;
int heaterEncB = 3;
int wastePumpENPin = 11;
int wastePumpIN2Pin = 12;
int heparinPumpIN1Pin = 6;
int heparinPumpIN2Pin = 7;
int deaeratorIN1Pin = 8;
int heaterIN1Pin = 9;
int heaterIN2Pin = 10;
int deaeratorIN2Pin = 13;

volatile bool heaterRunningFB = false;
bool wastePumpRunningFB = false;
bool deaeratorRunningFB = false;
bool heparinPumpRunningFB = false;
bool serialInputFlag = false;

// Output Objects
// include: waste pump, heparin pump, deaerator, heater
bool startCommand = false; // From Master
int tempPWM = 0;
volatile bool wastePumpRun = false;
volatile double hepRunTimeRemaining = 0;
volatile int UFRate = 0;
int UFRateScl = 0;
volatile int motorPosition = 0;
volatile long dialTemp = 0;

// ----------- //
// SETUP LOOP  //
// ----------- //
void setup() {
  Wire.begin(0x11); // join I2C bus as slave with address 0x11
  Wire.onRequest(IOSend);
  Wire.onReceive(MasterControl);
  pinMode(heaterEncA, INPUT);
  attachInterrupt(digitalPinToInterrupt(heaterEncA), readEnc, RISING);
  pinMode(heaterEncB, INPUT);
  pinMode(wastePumpENPin, OUTPUT);
  pinMode(wastePumpIN2Pin, OUTPUT);
  pinMode(heparinPumpIN2Pin, OUTPUT);
  pinMode(heparinPumpIN1Pin, OUTPUT);
  pinMode(deaeratorIN1Pin, OUTPUT);
  pinMode(heaterIN1Pin, OUTPUT);
  pinMode(heaterIN2Pin, OUTPUT);
  pinMode(deaeratorIN2Pin, OUTPUT);
  Serial.begin(19200);
}

// ---------- //
// MAIN LOOP  //
// ---------- //

void loop() {

  if (!serialInputFlag) {
    Serial.println(F("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")); //'Clear' serial monitor output
    serialInputFlag = true;
  }

  while (startCommand) {
    
    // Read Analogue Inputs
    waterLevelVal = analogRead(waterLevelPin);
    venTempVal = analogRead(venTempPin);
    bloodLeakVal = analogRead(bloodLeakPin);
    dialLevelVal = analogRead(dialLevelPin);

    // Scale Analogue Inputs for Transmission to Master
    waterLevelValScl = map(waterLevelVal, 0, 1023, 0, 100 * FLOAT_SCALE);
    venTempValScl = scaleInput(venTempVal, 20, 358, 26, 60);
    bloodLeakValScl = bloodLeakVal * FLOAT_SCALE; // checking against arbitrary threshold found experimentally
    dialLevelValScl = scaleInput(dialLevelVal, 0, 466, 0, 100);

    // Heater PID controlled
    heaterRunningFB = setMotor(1, tempPWM, heaterIN1Pin, heaterIN2Pin);
    Serial.print(tempPWM);
    Serial.print("\t");
    Serial.println(dialTemp / FLOAT_SCALE);

    // Scale UF Rate Calculate for Pump Speed (achieve required pressure)
    UFRateScl = map(UFRate, 0, 13, 0, 255);

    // Waste Pump (UF Pump) runs at the speed to maintain the required pressure for ultrafiltration
    if (wastePumpRun) { // command from master
      analogWrite(wastePumpENPin, UFRateScl);
      digitalWrite(wastePumpIN2Pin, HIGH);
      wastePumpRunningFB = true;
    } else {
      digitalWrite(wastePumpENPin, LOW);
      digitalWrite(wastePumpIN2Pin, LOW);
      wastePumpRunningFB = false;
    }

    if (hepRunTimeRemaining > 0) {
      digitalWrite(heparinPumpIN1Pin, HIGH);
      digitalWrite(heparinPumpIN2Pin, LOW);
      heparinPumpRunningFB = true;
    } else {
      digitalWrite(heparinPumpIN1Pin, LOW);
      digitalWrite(heparinPumpIN2Pin, LOW);
    }

    // Deaerator runs at a fixed speed continuously
    digitalWrite(deaeratorIN1Pin, HIGH);
    digitalWrite(deaeratorIN2Pin, LOW);
    deaeratorRunningFB = true;
  }

  // Stop all motors
  digitalWrite(wastePumpENPin, LOW);
  digitalWrite(wastePumpIN2Pin, LOW);
  digitalWrite(heparinPumpIN1Pin, LOW);
  digitalWrite(heparinPumpIN2Pin, LOW);
  digitalWrite(deaeratorIN1Pin, LOW);
  digitalWrite(deaeratorIN2Pin, LOW);
  setMotor(0, 0, heaterIN1Pin, heaterIN2Pin);

  heaterRunningFB = false;
  wastePumpRunningFB = false;
  deaeratorRunningFB = false;
  heparinPumpRunningFB = false;
}

// ---------- //
// FUNCTIONS  //
// ---------- //
// Transmit IO Data when requested by master
void IOSend() {
  I2C_writeAnything(waterLevelValScl);
  I2C_writeAnything(venTempValScl);
  I2C_writeAnything(bloodLeakValScl);
  I2C_writeAnything(dialLevelValScl);
  I2C_writeAnything(heaterRunningFB);
  I2C_writeAnything(wastePumpRunningFB);
  I2C_writeAnything(deaeratorRunningFB);
  I2C_writeAnything(heparinPumpRunningFB);
}

void MasterControl(int dataSize) {
  I2C_readAnything(startCommand);
  I2C_readAnything(tempPWM);
  I2C_readAnything(wastePumpRun);
  wastePumpRun = !wastePumpRun; // Invert polarity
  I2C_readAnything(hepRunTimeRemaining);
  I2C_readAnything(UFRate);
  I2C_readAnything(dialTemp);
}

// Scale Analogue Input  //
// floating point version of the map() function
// y = ((y2-y1/(x2-x1))*(x-x1)+y1
// x1,x2 are the input min/max
// y1,y2 are the output min/max
// x is the value to scale
// y is the scaled value
long scaleInput(long rawValue, long rawMin, long rawMax, long scaledMin, long scaledMax) {
  return ((FLOAT_SCALE * (scaledMax - scaledMin) / (rawMax - rawMin)) * (rawValue - rawMin) + FLOAT_SCALE * scaledMin);
}


// Enable pin is always wired HIGH
bool setMotor(int dir, int pwmVal, int in1, int in2) {

  if (dir == 1) {
    analogWrite(in1, pwmVal);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    analogWrite(in2, pwmVal);
    digitalWrite(in1, LOW);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  return (pwmVal > 0);
}

// Read Motor Encoders
void readEnc() {
  int encB = digitalRead(heaterEncB);
  if (encB > 0) {
    motorPosition++;
  }
  else {
    motorPosition--;
  }
}
