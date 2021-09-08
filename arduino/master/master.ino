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
int bloodFlowRatePin = A1;
int venPressPin = A2;
int wastePressPin = A3;

// Analogue Values
double artPressVal = 0;
double bloodFlowRateVal = 0;
double venPressVal = 0;
double wastePressVal = 0;

// Digital Pins
int alarmLEDPin = 13;
int alarmBuzzerPin = 10;
int startCommandPin = 2;
int EStopPin = 3;
int airDetectTXPin = 8;
int airDetectRXPin = 9;

bool startCommand = true; // Active Low
bool EStop = true; // Active Low
volatile bool startCommandLatch = false; // value can change in ISR
bool startCommandPrev = false;
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
const double PRESSURE_HIGH = 150; // TO DO update value
const double PRESSURE_LOW = -250; // TO DO update value
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
bool bloodFlowRateAlarm = false; // always false monitor only
bool venPressAlarm = false;
bool wastePressAlarm = false; // always false monitor only
bool airDetectAlarm = false;
int airDetectValue = 0;
bool dialConductivityAlarm = false;
bool pHAlarm = false;
bool dialTempAlarm = false;
bool anyAlarmTriggered = false;
bool bloodPumpFault = false;

// Initialise LCD
LiquidCrystal lcd(12, 11, 6, 7, 4, 5); // (rs,enable,d4,d5,d6,d7)
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
  attachInterrupt(digitalPinToInterrupt(startCommandPin), startCommandISR, FALLING);
  pinMode(EStopPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EStopPin), stopCommandISR, FALLING);
  pinMode(airDetectTXPin, OUTPUT);
  pinMode(airDetectRXPin, INPUT);
  lcd.begin(16, 2);
  Serial.begin(9600);
}

// ---------- //
// MAIN LOOP  //
// ---------- //
void loop() {
  
  digitalWrite(alarmBuzzerPin, LOW); // JUST TO REMOVE SOUND DURING TESTING - DELETE THIS LINE
  
  // Read Buttons
  //startCommand = digitalRead(startCommandPin);
  //EStop = digitalRead(EStopPin);
  displayUpdateString("Machine off.", "Press start.", 0);
  
  // Clear Alarms
  digitalWrite(alarmLEDPin, LOW);
  digitalWrite(alarmBuzzerPin, LOW);
  firstLoop = true;
  /*if (startCommand == false) { // Push button is active low (pullup resistor)
    startCommandLatch = true;
    }*/
    
  // Make sure slave devices have stopped (e.g. if reset button on master pressed)
  Wire.beginTransmission(0x10);
  I2C_writeAnything(false);
  Wire.endTransmission();
  Wire.beginTransmission(0x11);
  I2C_writeAnything(false);
  Wire.endTransmission();

  
  while ((startCommandLatch) & (currentTime < runTime)) {

    if (firstLoop) {
      displayUpdateString("Priming Complete", "Alarm tests pass", 1);
      delay(50); // hold message on screen for first loop
      Wire.beginTransmission(0x10);
      I2C_writeAnything(startCommandLatch);
      Wire.endTransmission();
      Wire.beginTransmission(0x11);
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
    bloodFlowRateVal = analogRead(bloodFlowRatePin);
    venPressVal = analogRead(venPressPin);
    wastePressVal = analogRead(wastePressPin);

    // Scale Analogue Inputs
    artPressVal = scaleInput(artPressVal, 0, 466, -300.0, -30.0); // TO DO UPDATE VALUES - 466 max value read from sensor?? - TBC
    bloodFlowRateVal = scaleInput(bloodFlowRateVal, 0, 466, 50.0, 250.0); // TO DO UPDATE VALUES (1023?)
    venPressVal = scaleInput(venPressVal, 0, 466, 50.0, 250.0); // TO DO UPDATE VALUES
    wastePressVal = scaleInput(wastePressVal, 0, 466, 0, 400.0); // TO DO UPDATE VALUES

    // Check if Emergency Stop Button has been pressed and relay stop to slave devices
    if (!startCommandLatch) {
      Wire.beginTransmission(0x10);
      I2C_writeAnything(startCommandLatch);
      Wire.endTransmission();
      Wire.beginTransmission(0x11);
      I2C_writeAnything(startCommandLatch);
      Wire.endTransmission();
      break;
    }
    
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

    // Check Device Fault Conditions
    bloodPumpFault = airDetectAlarm || venPressAlarm || artPressureAlarm; // ADD INFLOW + WATER LEVEL + VEN TEMP
    
    if (firstLoop) {
      anyAlarmTriggered = false; // Alarms not triggered first loop
      firstLoop = false;
    }

    // Trigger LED + Buzzer if any alarm conditions satisfied
    if (anyAlarmTriggered) {
      digitalWrite(alarmLEDPin, HIGH);
     // digitalWrite(alarmBuzzerPin, HIGH);
    }
    else {
      digitalWrite(alarmLEDPin, LOW);
      digitalWrite(alarmBuzzerPin, LOW);
    }

    // Update Display
    currentTime = millis();
    if (currentTime - prevTime > cyclePeriod) {
      if (cycle) {
        displayUpdateValue("Art Press: ", artPressVal, "Inf Press:  ", bloodFlowRateVal);
        serialPrint();
      } else {
        displayUpdateValue("Ven Press:  ", venPressVal, "Wst Press: ", wastePressVal);
      }
      prevTime = currentTime;
      cycle = !cycle;
    }

    // Second Check if Emergency Stop Button has been pressed and relay stop to slave devices
    if (!startCommandLatch) {
      Wire.beginTransmission(0x10);
      I2C_writeAnything(startCommandLatch);
      Wire.endTransmission();
      
      Wire.beginTransmission(0x11);
      I2C_writeAnything(startCommandLatch);
      Wire.endTransmission();
    }

    startCommandPrev = startCommandLatch;
  
    // Send fault conditions to slave devices
    // Update to only send if value changes?
    // Only one receive function so need to send all values
     Wire.beginTransmission(0x10);
      I2C_writeAnything(startCommandLatch);
      I2C_writeAnything(bloodPumpFault);
      I2C_writeAnything(airDetectAlarm); // Activate dialysate and venous clamps
      Wire.endTransmission();
    
    // Exit loop if EStop Pressed
    /*if (digitalRead(EStopPin) == false) {
      startCommandLatch = false;
      Wire.beginTransmission(0x10);
      I2C_writeAnything(startCommandLatch);
      Wire.endTransmission();
      displayUpdateString("Machine off.", "Press start.", 1);
      }*/
  }
}

// -------------- //
// Update Screen  //
// ------------- //
void startCommandISR() {
  startCommandLatch = true;
  /*Wire.beginTransmission(0x10);
    I2C_writeAnything(startCommandLatch);
    Wire.endTransmission();*/
}

void stopCommandISR() {
  startCommandLatch = false;
  displayUpdateString("Machine off.", "Press start.", 1);
}
void displayUpdateValue(String text1, double value1, String text2, double value2) {
  lcd.clear(); // clear screen and set cursor to (0,0)
  lcd.print(text1);
  lcd.print(value1);
  lcd.setCursor(0, 1);
  lcd.print(text2);
  lcd.print(value2);
}

void displayUpdateString(String text1, String text2, bool clear) {
  if (clear) {
    lcd.clear(); // clear screen and set cursor to (0,0)
  }
  else {
    lcd.setCursor(0, 0);
  }
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

void serialPrint(){
  // Device run state, speed, alarms, runtime remaing
  Serial.println(" ** RUNTIME **");
  Serial.println("Runtime remaining: ");
  Serial.println(" ** DEVICES **");
  Serial.println("Blood pump: ");
  Serial.println(" ** ALARMS **");
  Serial.println("Arterial Pressure Alarm: ");
  Serial.println("");
}
