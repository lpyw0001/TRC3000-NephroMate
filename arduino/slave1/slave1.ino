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
bool bloodPumpFault = false;
bool clampLines = false;

bool venousClampFB = false;
bool bypassValveFB = false;
bool bloodPumpRunningFB = false;
bool dialPumpRunningFB = false;
bool mixerRunningFB = false;

// Initialise LCD
unsigned long currentTime;
unsigned long prevTime;
unsigned long cyclePeriod = 100; // time in ms to alternate the screen values
bool cycle = true;

// Output Objects
Servo bypassValve;
Servo venousClamp;
const int CLAMP_ANGLE = 90;
const int CLAMP_OFF = 0;
bool startCommand = false; // From Master
int flow_PWM = 0;

// Digital Pins
int mixerIN1Pin = 0;
int bloodPumpIN2Pin = 1;
int bloodPumpPWMPin = 6;
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
  bypassValve.write(0);
  bypassValve.attach(9);
  venousClamp.write(0);
  venousClamp.attach(10);
  Wire.onReceive(MasterControl);
  pinMode(mixerIN1Pin, OUTPUT);
  pinMode(bloodPumpIN2Pin, OUTPUT);
  pinMode(bloodPumpPWMPin, OUTPUT);
  pinMode(dialPumpIN1Pin, OUTPUT);
  pinMode(dialPumpIN2Pin, OUTPUT);
  pinMode(dialClampActivePin, OUTPUT);
  pinMode(venClampActivePin, OUTPUT);
  pinMode(mixerIN2Pin, OUTPUT);
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
    // ** TO DO **: Verify analogue operating values
    /*dialConductivityScl = scaleInput(dialConductivityVal, 0, 1023, 10, 30);
    pHScl = scaleInput(pHVal, 0, 1023, 0, 14);
    dialTempScl = scaleInput(dialTempVal, 20, 400, 5, 60); // Should be 358 Tinkercad issue
    bloodFlowScl = scaleInput(bloodFlowVal, 0, 1023, 30, 600);*/
    
    dialConductivityScl = map(dialConductivityVal, 0, 1023, 10*FLOAT_SCALE, 30*FLOAT_SCALE);
    pHScl = map(pHVal, 0, 1023, 0*FLOAT_SCALE, 14*FLOAT_SCALE);
    dialTempScl = map(dialTempVal, 20, 358, 25*FLOAT_SCALE, 60*FLOAT_SCALE); // Should be 358 Tinkercad issue
    bloodFlowScl = scaleInput(bloodFlowVal, 0, 1023, 30, 600);
    
    /*dialTempScl = map(dialTempVal, 20, 400, 5*FLOAT_SCALE, 60*FLOAT_SCALE); // Should be 358 Tinkercad issue
    bloodFlowScl = map(bloodFlowVal, 0, 1023, 30*FLOAT_SCALE, 600*FLOAT_SCALE);*/
    
    // Mixer runs at a fixed speed continuously
    digitalWrite(mixerIN1Pin, HIGH);
    digitalWrite(mixerIN2Pin, LOW);
    mixerRunningFB = true;

    // Dialysate pump runs at a fixed speed continuously
    digitalWrite(dialPumpIN1Pin, HIGH);
    digitalWrite(dialPumpIN2Pin, LOW);
    dialPumpRunningFB = true;

    // Blood pump PID controlled
    int dir = (int) !bloodPumpFault;
    bloodPumpRunningFB = setMotor(dir, flow_PWM, bloodPumpPWMPin, bloodPumpIN2Pin); // flow_PWM_Pin to be defined
    // Currently just start/stop based on fault conditions
    /*if (bloodPumpFault) {
      digitalWrite(bloodPumpIN1Pin, LOW);
      digitalWrite(bloodPumpIN2Pin, LOW);
      bloodPumpRunningFB = false;
      } else {
      digitalWrite(bloodPumpIN1Pin, HIGH);
      digitalWrite(bloodPumpIN2Pin, LOW);
      bloodPumpRunningFB = true;
      }*/

    // Dialysate clamp: triggered when air is detected?
    // Venous clamp: triggered when air is detected?
    if (clampLines) {
      venousClamp.write(CLAMP_ANGLE);
      bypassValve.write(CLAMP_ANGLE);
      venousClampFB = true;
      bypassValveFB = true;
    }
    else {
      venousClamp.write(CLAMP_OFF);
      bypassValve.write(CLAMP_OFF);
    }
    
  }

  

  // Stop all motors
  digitalWrite(mixerIN1Pin, LOW);
  digitalWrite(mixerIN2Pin, LOW);
  digitalWrite(dialPumpIN1Pin, LOW);
  digitalWrite(dialPumpIN2Pin, LOW);
  setMotor(0, 0, bloodPumpPWMPin, bloodPumpIN2Pin);
  venousClamp.write(CLAMP_ANGLE); // clamp lines if not running as a safety precaution
  bypassValve.write(CLAMP_ANGLE);
  venousClampFB = false;
  bypassValveFB = false;
  bloodPumpRunningFB = false;
  dialPumpRunningFB = false;
  mixerRunningFB = false;
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
  I2C_writeAnything(venousClampFB);
  I2C_writeAnything(bypassValveFB);
  I2C_writeAnything(bloodPumpRunningFB);
  I2C_writeAnything(dialPumpRunningFB);
  I2C_writeAnything(mixerRunningFB);
}

void MasterControl(int dataSize) {
  I2C_readAnything(startCommand);
  I2C_readAnything(bloodPumpFault);
  I2C_readAnything(clampLines);
  I2C_readAnything(flow_PWM);
}
// Scale Analogue Input  //
// floating point version of the map() function
// y = ((y2-y1/(x2-x1))*(x-x1)+y1
// x1,x2 are the input min/max
// y1,y2 are the output min/max
// x is the value to scale
// y is the scaled value
long scaleInput(long rawValue, long rawMin, long rawMax, long scaledMin, long scaledMax) {
  return ((FLOAT_SCALE*(scaledMax - scaledMin) / (rawMax - rawMin)) * (rawValue - rawMin) + FLOAT_SCALE*scaledMin);
}

// set PWM of motor
// pin in2 permanently wired to LOW
bool setMotor(int dir, int pwmVal, int pwm, int in1) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    //digitalWrite(in2,LOW);
  }
  /*else if(dir == -1){
    digitalWrite(in1,LOW);
    //digitalWrite(in2,HIGH);
    }*/
  else {
    digitalWrite(in1, LOW);
    //digitalWrite(in2,LOW);
  }

  return (pwm > 0 && dir == 1);
}
