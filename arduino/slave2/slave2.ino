// TRC3000 - NephroMate
// Slave 2
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
int waterLevelPin = A0;
int venTempPin = A1;
int bloodLeakPin = A2;
int dialLevelPin = A3;

// Analogue Values
double waterLevelVal = 0;
double venTempVal = 0;
double bloodLeakVal = 0;
double dialLevelVal = 0;

// Scaled Analogue Values
double waterLevelValScl = 0;
double venTempValScl = 0;
double bloodLeakValScl = 0;
double dialLevelValScl = 0;

// Digital Pins
int wastePumpIN1Pin = 0;
int wastePumpIN2Pin = 1;
int heparinPumpIN1Pin = 6;
int heparinPumpIN2Pin = 7;
int deaeratorIN1Pin = 8;
int heaterIN1Pin = 9;
int heaterIN2Pin = 10;
int deaeratorIN2Pin = 13;

// Initialise LCD
LiquidCrystal lcd(12, 11, 2, 3, 4, 5); // (rs,enable,d4,d5,d6,d7)
unsigned long currentTime;
unsigned long prevTime;
unsigned long cyclePeriod = 105; // time in ms to alternate the screen values
bool cycle = true;
bool firstLoop = true;

// Output Objects
// include: waste pump, heparin pump, deaerator, heater
bool startCommand = false; // From Master
int temp_PWM = 0;

// ----------- //
// SETUP LOOP  //
// ----------- //
void setup() {
  Wire.begin(0x11); // join I2C bus as slave with address 0x11
  Wire.onRequest(IOSend);
  Wire.onReceive(MasterControl);
  lcd.begin(16, 2);
}

// ---------- //
// MAIN LOOP  //
// ---------- //

void loop() {
  while (startCommand) {
    // Read Analogue Inputs
    waterLevelVal = analogRead(waterLevelPin);
    venTempVal = analogRead(venTempPin);
    bloodLeakVal = analogRead(bloodLeakPin);
    dialLevelVal = analogRead(dialLevelPin);

    // Scale Analogue Inputs for Transmission to Master
    // ** TO DO **: Verify operating scaled values
    waterLevelValScl = scaleInput(waterLevelVal, 1013, 1023, 0, 100.0);
    venTempValScl = scaleInput(venTempVal, 20, 358, 5, 60);
    bloodLeakValScl = bloodLeakVal; // no need for scaling, since checking against arbitrary threshold found experimentally
    dialLevelValScl = scaleInput(dialLevelVal, 0, 466, 0.0, 100.0); 

    currentTime = millis();
    if (currentTime - prevTime > cyclePeriod) {
      if (cycle) {
        displayUpdate("Water Lvl", waterLevelValScl, "Ven Temp", venTempValScl);
      } else {
        displayUpdate("Bld Leak", bloodLeakValScl, "Dial Lvl", dialLevelValScl);
      }
      prevTime = currentTime;
      cycle = !cycle;
      // dialysateClamp.write(CLAMP_OFF);
    }
  }
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
  I2C_writeAnything(waterLevelValScl);
  I2C_writeAnything(venTempValScl);
  I2C_writeAnything(bloodLeakValScl);
  I2C_writeAnything(dialLevelValScl);
}

void MasterControl(int dataSize) {
  I2C_readAnything(startCommand);
  I2C_readAnything(temp_PWM);
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

// set PWM of motor
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}
