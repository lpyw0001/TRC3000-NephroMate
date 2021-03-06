// TRC3000 - NephroMate
// Slave 1
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
int dialConductivityPin = A0;
int pHPin = A1;
int dialTempPin = A2;
int bloodFlowPin = A3;

// Analogue Values
long dialConductivityVal = 0;
long pHVal = 0;
long dialTempVal = 0;
long bloodFlowVal = 0;

long dialConductivityScl = 0;
long pHScl = 0;
long dialTempScl = 0;
long bloodFlowScl = 0;

// Fault Conditions
bool bloodPumpFault = true;
bool clampLines = false;

bool venousClampFB = false;
bool bypassValveFB = false;
bool bloodPumpRunningFB = false;
bool dialPumpRunningFB = false;
bool mixerRunningFB = false;
bool reverseOsmosisRunningFB = false;

// Initialise LCD
unsigned long currentTime;
unsigned long prevTime;
unsigned long cyclePeriod = 100; // time in ms to alternate the screen values
bool cycle = true;

// Output Objects
Servo bypassValve;
Servo venousClamp;
const int CLAMP_ANGLE = 0;
const int CLAMP_OFF = 90;
bool startCommand = false; // From Master
int flowPWM = 0;
volatile int motorPosition = 0;

// Digital Pins
int bloodPumpEncA = 2;
int bloodPumpEncB = 3;
int mixerIN1Pin = 4;
int bloodPumpIN1Pin = 5;
int bloodPumpIN2Pin = 6;
int dialPumpIN1Pin = 7;
int dialPumpIN2Pin = 8;
int dialClampActivePin = 9;
int venClampActivePin = 10;
int mixerIN2Pin = 11;
int reverseOsmosisIN1Pin = 12;
int reverseOsmosisIN2Pin = 13;

// From Master
volatile bool bypassValveCMD = false;
volatile bool dialPumpCMD = false;

// Serial Data Entry
bool serialInputFlag = false;
bool fluidFlag = false;
bool runTimeFlag = false;
bool hepFlag = false;
bool weightFlag = false;
bool UFFlag = false;
bool slave1SerialData = false;
int desiredFluidRemoval = 0;
long desiredFluidRemovalCalc;
int patientWeight = 0;
long UFRateCalc = 0;
int UFRate = 0;
int runTimeMin = 0;
int hepRunTime = 0;

// ----------- //
// SETUP LOOP  //
// ----------- //
void setup() {
  Wire.begin(0x10); // join I2C bus as slave with address 0x10
  Wire.onRequest(IOSend);
  bypassValve.attach(9);
  bypassValve.write(CLAMP_ANGLE);
  venousClamp.attach(10);
  venousClamp.write(CLAMP_ANGLE);
  Wire.onReceive(MasterControl);
  pinMode(bloodPumpEncA, INPUT);
  attachInterrupt(digitalPinToInterrupt(bloodPumpEncA), readEnc, RISING);
  pinMode(bloodPumpEncB, INPUT);
  pinMode(mixerIN1Pin, OUTPUT);
  pinMode(bloodPumpIN1Pin, OUTPUT);
  pinMode(bloodPumpIN2Pin, OUTPUT);
  pinMode(dialPumpIN1Pin, OUTPUT);
  pinMode(dialPumpIN2Pin, OUTPUT);
  pinMode(dialClampActivePin, OUTPUT);
  pinMode(venClampActivePin, OUTPUT);
  pinMode(mixerIN2Pin, OUTPUT);
  pinMode(reverseOsmosisIN2Pin, OUTPUT);
  pinMode(reverseOsmosisIN1Pin, OUTPUT);
  Serial.begin(9600);
}

// ---------- //
// MAIN LOOP  //
// ---------- //

void loop() {

  // Serial Data Entry
  const String sesDurStr = "Duration (min): ";
  const String desFluidStr = "Fluid Removal (L): ";
  const String weightStr = "Patient weight (kg): ";
  const String hepStr = "Hep Duration (min) 'z' for none: " ;

  if (!serialInputFlag) {
    Serial.println(F("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")); //'Clear' serial monitor output
    serialInputFlag = true;
  }

  if (!runTimeFlag) {
    runTimeMin = (int)SerialDataEntry(sesDurStr, 99);
    runTimeFlag = true;
  }

  if (!fluidFlag) {
    desiredFluidRemoval = SerialDataEntry(desFluidStr, 1);
    desiredFluidRemovalCalc = desiredFluidRemoval * 1000;
    fluidFlag = true;
  }

  if (!weightFlag) {
    patientWeight = SerialDataEntry(weightStr, 10);
    weightFlag = true;
  }

  if (!hepFlag) {
    hepRunTime = SerialDataEntry(hepStr, 0);
    hepFlag = true;
  }

  UFRateCalc = FLOAT_SCALE * ((FLOAT_SCALE * desiredFluidRemovalCalc) / (FLOAT_SCALE * runTimeMin / 60)) / (FLOAT_SCALE * patientWeight);
  UFRate = (int)abs((FLOAT_SCALE * UFRateCalc));

  if (!UFFlag && UFRate != 0) {
    Serial.print(F("UF Rate: "));
    Serial.print(UFRate / FLOAT_SCALE);
    Serial.print(F("."));
    Serial.print(UFRate % FLOAT_SCALE);
    Serial.println(F(" mL/h/kg"));
  }

  // User re-enter values if calculated UF Rate too high (unsafe)
  if ((UFRate / FLOAT_SCALE) > 13) {
    Serial.println(F("UF Rate too high."));
    hepFlag = false;
    weightFlag = false;
    fluidFlag = false;
    runTimeFlag = false;
  }
  else if (UFRate > 0) {
    UFFlag = true;
    UFRate = UFRate / 100;
  }

  slave1SerialData = (runTimeMin > 0) && (desiredFluidRemoval > 0) && (patientWeight > 0) && (UFRate > 0);

  // Commence main loop
  // Start command from master: toggled once valid serial data received and start button pressed
  while (startCommand) {
    // Read Analogue Inputs
    dialConductivityVal = analogRead(dialConductivityPin);
    pHVal = analogRead(pHPin);
    dialTempVal = analogRead(dialTempPin);
    bloodFlowVal = analogRead(bloodFlowPin);

    // Scale Analogue Inputs for Transmission to Master
    dialConductivityScl = map(dialConductivityVal, 0, 1023, 10 * FLOAT_SCALE, 30 * FLOAT_SCALE);
    pHScl = map(pHVal, 0, 1023, 0 * FLOAT_SCALE, 14 * FLOAT_SCALE);
    dialTempScl = map(dialTempVal, 20, 358, 25 * FLOAT_SCALE, 60 * FLOAT_SCALE);
    bloodFlowScl = scaleInput(bloodFlowVal, 0, 1023, 30, 600);
    //bloodFlowScl = scaleInput(bloodFlowVal, 0, 1023, 175, 275); // Temp for PID Demo

    // Mixer runs at a fixed speed continuously
    digitalWrite(mixerIN1Pin, HIGH);
    digitalWrite(mixerIN2Pin, LOW);
    mixerRunningFB = true;

    // Dialysate pump runs as long as no level alarm from Master
    if (dialPumpCMD) {
      digitalWrite(dialPumpIN1Pin, HIGH);
      digitalWrite(dialPumpIN2Pin, LOW);
      dialPumpRunningFB = true;
    }
    else {
      digitalWrite(dialPumpIN1Pin, LOW);
      digitalWrite(dialPumpIN2Pin, LOW);
      dialPumpRunningFB = false;
    }

    // Reverse Osmosis unit runs at a fixed speed continously
    digitalWrite(reverseOsmosisIN1Pin, HIGH);
    digitalWrite(reverseOsmosisIN2Pin, HIGH);
    reverseOsmosisRunningFB = true;

    // Blood pump PID controlled
    //int dir = (int) !bloodPumpFault;
    int bloodPumpDir = 0;
    if (!bloodPumpFault) {
      bloodPumpDir = 1;
    }

    Serial.print(flowPWM);
    Serial.print("\t");
    Serial.println(bloodFlowScl / FLOAT_SCALE);
    bloodPumpRunningFB = setMotor(bloodPumpDir, flowPWM, bloodPumpIN1Pin, bloodPumpIN2Pin);

    // Bypass Valve (command received from master)
    // Diverts dialysate to waste when temperature, conductivity or pH alarm triggered
    if (bypassValveCMD) {
      bypassValve.write(CLAMP_ANGLE);
      bypassValveFB = true;
    }
    else {
      bypassValve.write(CLAMP_OFF);
      bypassValveFB = false;
    }

    // Clamps activated when air detected
    // If in main loop ensure clamps not active
    venousClamp.write(CLAMP_OFF);
    venousClampFB = false;
  }

  // Stop all motors
  digitalWrite(mixerIN1Pin, LOW);
  digitalWrite(mixerIN2Pin, LOW);
  digitalWrite(dialPumpIN1Pin, LOW);
  digitalWrite(dialPumpIN2Pin, LOW);
  digitalWrite(reverseOsmosisIN1Pin, LOW);
  digitalWrite(reverseOsmosisIN2Pin, LOW);
  reverseOsmosisRunningFB = false;
  bloodPumpRunningFB = setMotor(0, 0, bloodPumpIN1Pin, bloodPumpIN2Pin);
  venousClamp.write(CLAMP_ANGLE); // clamp lines if not running as a safety precaution
  bypassValve.write(CLAMP_ANGLE); // divert to waste if machine not running
  venousClampFB = true;
  bypassValveFB = true;
  bloodPumpRunningFB = false;
  dialPumpRunningFB = false;
  mixerRunningFB = false;
  reverseOsmosisRunningFB = false;
}

// ---------- //
// FUNCTIONS  //
// ---------- //
// Transmit IO Data when requested by master
void IOSend() {
  if (slave1SerialData && !startCommand) {
    I2C_writeAnything(runTimeMin);
    I2C_writeAnything(desiredFluidRemoval);
    I2C_writeAnything(patientWeight);
    I2C_writeAnything(hepRunTime);
    I2C_writeAnything(UFRate);
  }
  else {
    I2C_writeAnything(dialConductivityScl);
    I2C_writeAnything(pHScl);
    I2C_writeAnything(dialTempScl);
    I2C_writeAnything(bloodFlowScl);
    I2C_writeAnything(venousClampFB);
    I2C_writeAnything(bypassValveFB);
    I2C_writeAnything(bloodPumpRunningFB);
    I2C_writeAnything(dialPumpRunningFB);
    I2C_writeAnything(mixerRunningFB);
    // I2C_writeAnything(reverseOsmosisRunningFB); // Not transmitted as not displayed on serial monitor due to a size limit
  }
}

void MasterControl(int dataSize) {
  I2C_readAnything(startCommand);
  I2C_readAnything(bloodPumpFault);
  I2C_readAnything(clampLines);
  I2C_readAnything(flowPWM);
  I2C_readAnything(bypassValveCMD);
  I2C_readAnything(dialPumpCMD);
  dialPumpCMD = !dialPumpCMD; // Invert polarity
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

// Read Motor Encoders
void readEnc() {
  int encB = digitalRead(bloodPumpEncB);
  if (encB > 0) {
    motorPosition++;
  }
  else {
    motorPosition--;
  }
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

long SerialDataEntry(const String &guide, int minValue) {
  char inputChar;
  String inputStr;
  long output = 0;

  Serial.print(guide);

  if (minValue == 0) {
    minValue = 10;
  }
  while (output <= minValue) {
    while (Serial.available() > 0) {
      inputChar = Serial.read();
      if (inputChar >= '0' && inputChar <= '9') {
        inputStr += inputChar;
      }
      else if (inputChar == 'z') {
        output = 100; // break loop
      }
    }
    if (inputChar != 'z') {
      output = inputStr.toInt();
    }
  }

  if (inputChar == 'z') {
    output = 0;
    Serial.println(inputChar);
  }
  else {
    Serial.println(output);
  }

  return output;
}
