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

// Digital Pins
int alarmLEDPin = 0;
int alarmBuzzerPin = 1;
int startCommandPin = 6;
int EStopPin = 7;
int deaeratorStartPin = 13;

bool startCommand = true;
bool EStop = true;

// From Slave 1
double venTempVal_S1;
double venPressVal_S1;
double wastePressVal_S1;
double wasteLevelVal_S1;

// Setpoints
/*const double PRESSURE_L_SCL;
  const double PRESSURE_H_SCL;*/
const double PRESSURE_HIGH = 500; // update value
const double PRESSURE_LOW = 20; // update value
bool alarmState = false;

// Initialise LCD
LiquidCrystal lcd(12, 11, 2, 3, 4, 5);
unsigned long currentTime;
unsigned long prevTime;
unsigned long cyclePeriod = 100; // time in ms to alternate the screen values (note 100ms in tinkercad =/= 100ms real time)
bool cycle = true;
bool firstLoop = true;

// ----------- //
// SETUP LOOP  //
// ----------- //
void setup() {
  Wire.begin(); // join I2C bus (address optional for master device)
  //Serial.begin(9600);
pinMode(alarmLEDPin, OUTPUT);
pinMode(alarmBuzzerPin, OUTPUT);
pinMode(startCommandPin, INPUT_PULLUP);
pinMode(EStopPin, INPUT_PULLUP);
pinMode(deaeratorStartPin, OUTPUT);

  lcd.begin(16, 2);
}

// ---------- //
// MAIN LOOP  //
// ---------- //
void loop() {
  // Read Buttons
  startCommand = digitalRead(startCommandPin);
  EStop = digitalRead(EStopPin);
  
  if(startCommand==false){
    EStop=true;
  }
   // Read Analogue Inputs
  artPressVal = analogRead(artPressPin);
  artTempVal = analogRead(artTempPin);
  hepLevelVal = analogRead(hepLevelPin);
  inflowPressVal = analogRead(inflowPressPin);

  alarmState = (artPressVal > PRESSURE_HIGH or artPressVal < PRESSURE_LOW);

  // Trigger LED + Buzzer if any conditions satisfied
  if (alarmState) {
    digitalWrite(alarmLEDPin, HIGH);
    digitalWrite(alarmBuzzerPin, HIGH);
  }
  else {
    digitalWrite(alarmLEDPin, LOW);
    digitalWrite(alarmBuzzerPin, LOW);
  }

  if (firstLoop) {
    displayUpdate("Art Press: ", artPressVal, "Art Temp:  ", artTempVal);
    firstLoop = false;
  }
  
  // Update Display
  currentTime = millis();
  if (currentTime - prevTime > cyclePeriod) {
    if (cycle) {
      displayUpdate("Art Press: ", artPressVal, "Art Temp:  ", artTempVal);
    } else {
      displayUpdate("Hep Level:  ", hepLevelVal, "Infl Press: ", inflowPressVal);
    }
    prevTime = currentTime;
    cycle = !cycle;
  }

  // Request IO from slave 0x10 (4 x analogue values)
  Wire.requestFrom(0x10, 16);
  I2C_readAnything(venTempVal_S1);
  I2C_readAnything(venPressVal_S1);
  I2C_readAnything(wastePressVal_S1);
  I2C_readAnything(wasteLevelVal_S1);
  
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
void displayUpdate(String text1, double value1, String text2, double value2) {
  lcd.setCursor(0, 0);
  lcd.print(text1);
  lcd.print(value1);
  lcd.setCursor(0, 1);
  lcd.print(text2);
  lcd.print(value2);
}
