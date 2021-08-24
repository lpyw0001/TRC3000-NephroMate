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
int inflowPressPin = A1;
int venPressPin = A2;
int wastePressPin = A3;

// Analogue Values
double artPressVal = 0;
double inflowPressVal = 0;
double venPressVal = 0;
double wastePressVal = 0;

// Digital Pins
int alarmLEDPin = 0;
int alarmBuzzerPin = 1;
int startCommandPin = 6;
int EStopPin = 7;
int airDetectTXPin = 8;
int airDetectRXPin = 9;

bool startCommand = true; // Active Low
bool EStop = true; // Active Low
bool startCommandLatch = false;

// From Slave 1
double dialConductivityVal_S1 = 0;
double pHVal_S1 = 0;
double dialTempVal_S1 = 0;
double dialPressVal_S1 = 0;

// From Slave 2
double waterLevelVal_S2 = 0;
double venTempVal_S2 = 0;
double bloodLeakVal_S2 = 0;
double dialLevelVal_S2 = 0;

// Alarm Limits
const double PRESSURE_HIGH = 100; // TO DO update value
const double PRESSURE_LOW = 20; // TO DO update value
const double LEVEL_LOW = 10;
const double TEMP_LOW = 36;
const double TEMP_HIGH = 42;
const double CONDUCTIVITY_LOW = 12;
const double CONDUCTIVITY_HIGH = 16;
const double PH_LOW = 6.8;
const double PH_HIGH = 7.6;
const double AIR_DETECT = 2000; // TO DO update value

// Alarm Variables
// TO DO ADD SLAVE 2 ALARMS
bool artPressureAlarm = false;
bool inflowPressAlarm = false; // always false monitor only
bool venPressAlarm = false;
bool wastePressAlarm = false; // always false monitor only
bool airDetectAlarm = false;
int airDetectValue = 0;
bool dialConductivityAlarm = false;
bool pHAlarm = false;
bool dialTempAlarm = false;
bool anyAlarmTriggered = false;

// Initialise LCD
LiquidCrystal lcd(12, 11, 2, 3, 4, 5); // (rs,enable,d4,d5,d6,d7)
unsigned long currentTime = 0;
unsigned long prevTime = 0;
unsigned long cyclePeriod = 100; // time in ms to alternate the screen values (note 100ms in tinkercad =/= 100ms real time)
unsigned long runTime = 5000; // time in ms to perform hemodialysis (refer comment above)
bool cycle = true; // alternate values displayed on screen
bool firstLoop = true;

// ----------- //
// SETUP LOOP  //
// ----------- //
void setup() {
  Wire.begin(); // join I2C bus (address optional for master device)
  pinMode(alarmLEDPin, OUTPUT);
  pinMode(alarmBuzzerPin, OUTPUT);
  pinMode(startCommandPin, INPUT_PULLUP);
  pinMode(EStopPin, INPUT_PULLUP);
  pinMode(airDetectTXPin, OUTPUT);
  pinMode(airDetectRXPin, INPUT);
  lcd.begin(16, 2);
}

// ---------- //
// MAIN LOOP  //
// ---------- //
void loop() {

  // Read Buttons
  startCommand = digitalRead(startCommandPin);
  EStop = digitalRead(EStopPin);
  displayUpdateString("Machine off.", "Press start.");
  if (startCommand == false) { // Push button is active low (pullup resistor)
    startCommandLatch = true;
  }

  while ((startCommandLatch) & (currentTime < runTime)) {
   
    if (firstLoop) {
      displayUpdateString("Priming Complete", "Alarm tests pass");
      delay(50); // hold message on screen for first loop
      Wire.beginTransmission(0x10);
      I2C_writeAnything(startCommandLatch);
      Wire.endTransmission();
    }
    
    // Check for Air in the line
    digitalWrite(airDetectTXPin, LOW);
    delayMicroseconds(5);
    digitalWrite(airDetectTXPin, HIGH);
    delayMicroseconds(10); // Send + Receive
    digitalWrite(airDetectTXPin, LOW);
    airDetectValue = pulseIn(airDetectRXPin, HIGH);

    // Read Analogue Inputs
    artPressVal = analogRead(artPressPin);
    inflowPressVal = analogRead(inflowPressPin);
    venPressVal = analogRead(venPressPin);
    wastePressVal = analogRead(wastePressPin);

    // Scale Analogue Inputs
    artPressVal = scaleInput(artPressVal, 0, 1023, 12.7, 127.0); // TO DO UPDATE VALUES
    inflowPressVal = scaleInput(inflowPressVal, 0, 1023, 12.7, 127.0); // TO DO UPDATE VALUES
    venPressVal = scaleInput(venPressVal, 0, 1023, 12.7, 127.0); // TO DO UPDATE VALUES
    wastePressVal = scaleInput(wastePressVal, 0, 1023, 12.7, 127.0); // TO DO UPDATE VALUES

    // Request IO from slave 0x10 (4 x analogue values)
    Wire.requestFrom(0x10, 16);
    I2C_readAnything(dialConductivityVal_S1);
    I2C_readAnything(pHVal_S1);
    I2C_readAnything(dialTempVal_S1);
    I2C_readAnything(dialPressVal_S1);

    // Request IO from slave 0x11 (4 x analogue inputs)
    Wire.requestFrom(0x11, 16);
    I2C_readAnything(waterLevelVal_S2);
    I2C_readAnything(venTempVal_S2);
    I2C_readAnything(bloodLeakVal_S2);
    I2C_readAnything(dialLevelVal_S2);

    // Check Alarm Conditions
    // TO DO ADD SLAVE ALARMS
    artPressureAlarm = (artPressVal > PRESSURE_HIGH || artPressVal < PRESSURE_LOW);
    venPressAlarm = (venPressVal > PRESSURE_HIGH || venPressVal < PRESSURE_LOW);
    airDetectAlarm = airDetectValue > AIR_DETECT;
    dialConductivityAlarm = dialConductivityVal_S1 > CONDUCTIVITY_HIGH || dialConductivityVal_S1 < CONDUCTIVITY_LOW;
    pHAlarm = pHVal_S1 > PH_HIGH || pHVal_S1 < PH_LOW;
    dialTempAlarm = dialTempVal_S1 > TEMP_HIGH || dialTempVal_S1 < TEMP_LOW;

    anyAlarmTriggered = artPressureAlarm || venPressAlarm || airDetectAlarm || dialConductivityAlarm || pHAlarm || dialTempAlarm;
    
    if(firstLoop){
      anyAlarmTriggered = false; // Alarms not triggered first loop
      firstLoop = false;
    }

    // Trigger LED + Buzzer if any alarm conditions satisfied
    if (anyAlarmTriggered) {
      digitalWrite(alarmLEDPin, HIGH);
      //digitalWrite(alarmBuzzerPin, HIGH);
    }
    else {
      digitalWrite(alarmLEDPin, LOW);
      digitalWrite(alarmBuzzerPin, LOW);
    }

    // Update Display
    currentTime = millis();
    if (currentTime - prevTime > cyclePeriod) {
      if (cycle) {
        displayUpdateValue("Art Press: ", artPressVal, "Inf Press:  ", inflowPressVal);
      } else {
        displayUpdateValue("Ven Press:  ", venPressVal, "Wst Press: ", wastePressVal);
      }
      prevTime = currentTime;
      cycle = !cycle;
    }

    // Exit loop if EStop Pressed
    if (digitalRead(EStopPin) == false) {
      startCommandLatch = false;
    }
  }
}

// -------------- //
// Update Screen  //
// ------------- //
void displayUpdateValue(String text1, double value1, String text2, double value2) {
  lcd.clear();
  lcd.print(text1);
  lcd.print(value1);
  lcd.setCursor(0, 1);
  lcd.print(text2);
  lcd.print(value2);
}

void displayUpdateString(String text1, String text2) {
  lcd.clear();
  lcd.print(text1);
  lcd.setCursor(0, 1);
  lcd.print(text2);
}
// --------------------- //
// Scale Analogue Input  //
// -------------------- //
// floating point version of the map() function
// y = ((y2-y1/(x2-x1))*(x-x1)+y1
// x1,x2 are the input min/max
// y1,y2 are the output min/max
// x is the value to scale
// y is the scaled value
double scaleInput(int rawValue, int rawMin, int rawMax, double scaledMin, double scaledMax) {
  return ((scaledMax - scaledMin) / (rawMax - rawMin)) * (rawValue - rawMin) + scaledMin;
}
