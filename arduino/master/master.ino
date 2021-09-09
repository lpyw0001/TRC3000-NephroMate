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
int dialPressPin = A1;
int venPressPin = A2;
int wastePressPin = A3;

// Analogue Values
double artPressVal = 0;
double dialPressVal = 0;
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
bool startCommandLatchPrev = false;
bool bloodPumpFaultPrev = false;
bool airDetectAlarmPrev = false;

// From Slave 1
double dialConductivityVal_S1 = 0;
double pHVal_S1 = 0;
double dialTempVal_S1 = 0;
double bloodFlowVal_S1 = 0;

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
bool artPressureAlarm = false;
bool dialPressAlarm = false; // always false monitor only
bool venPressAlarm = false;
bool wastePressAlarm = false; // always false monitor only
bool airDetectAlarm = false;
int airDetectValue = 0;
bool dialConductivityAlarm = false;
bool pHAlarm = false;
bool dialTempAlarm = false;
bool waterLevelAlarm = false;
bool venTempAlarm = false;
bool bloodLeakAlarm = false;
bool dialLevelAlarm = false;
bool bloodFlowAlarm = false;
bool anyAlarmTriggered = false;
bool bloodPumpFault = false;

// Initialise LCD
LiquidCrystal lcd(12, 11, 6, 7, 4, 5); // (rs,enable,d4,d5,d6,d7)
String tempPrinting = "";

// Timing and Sequencing Variables
unsigned long currentTime = 0;
unsigned long prevTime = 0;
float runTimeRemaining = 0;
unsigned long cyclePeriod = 100; // time in ms to alternate the screen values (note 100ms in tinkercad =/= 100ms real time)
unsigned long runTime = 4000; // time in ms to perform hemodialysis (refer comment above)
unsigned long hepRunTime = 200; // Duration to run Heparin infusion
bool cycle = true; // alternate values displayed on LCD screen and in serial monitor
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

  displayUpdateString("Machine off.", "Press start.", 0);

  // Clear Alarms
  digitalWrite(alarmLEDPin, LOW);
  digitalWrite(alarmBuzzerPin, LOW);
  firstLoop = true;

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
      Serial.println("Haemodialysis started\n");
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
    dialPressVal = analogRead(dialPressPin);
    venPressVal = analogRead(venPressPin);
    wastePressVal = analogRead(wastePressPin);

    // Scale Analogue Inputs
    artPressVal = scaleInput(artPressVal, 0, 466, -300.0, -30.0); // TO DO UPDATE VALUES - 466 max value read from sensor?? - TBC
    dialPressVal = scaleInput(dialPressVal, 0, 466, 50.0, 250.0); // TO DO UPDATE VALUES (1023?)
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
    I2C_readAnything(bloodFlowVal_S1);

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

    // TO DO CHECK SLAVE 2 ALARM VALUES

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
    runTimeRemaining = (runTime - currentTime) / 18; // arbitrary scaling to return a reasonable value
    if (currentTime - prevTime > cyclePeriod) {
      if (cycle) {
        displayUpdateValue("Art Press: ", artPressVal, "Inf Press:  ", dialPressVal);
        serialPrintStatus();
      } else {
        displayUpdateValue("Ven Press:  ", venPressVal, "Wst Press: ", wastePressVal);
        serialPrintValues();
      }
      prevTime = currentTime;
      cycle = !cycle;
    }

    // Second Check if Emergency Stop Button has been pressed and relay stop to slave devices
    if (!startCommandLatch) {
      Wire.beginTransmission(0x11);
      I2C_writeAnything(startCommandLatch);
      Wire.endTransmission();
    }

    // Send fault conditions to slave devices if any values have changed
    // Only one receive function so need to send all values
    if (startCommandLatch != startCommandLatchPrev || bloodPumpFault != bloodPumpFaultPrev || airDetectAlarm != airDetectAlarmPrev) {
      Wire.beginTransmission(0x10);
      I2C_writeAnything(startCommandLatch);
      I2C_writeAnything(bloodPumpFault);
      I2C_writeAnything(airDetectAlarm); // Activate dialysate and venous clamps
      Wire.endTransmission();
    }

    // Update Previous Values
    startCommandLatchPrev = startCommandLatch;
    bloodPumpFaultPrev = bloodPumpFault;
    airDetectAlarmPrev = airDetectAlarm;
  }
}

// ------------- //
// INTERRUPTS    //
// ------------- //
void startCommandISR() {
  startCommandLatch = true;
  // Note interrupt driven so cannot send stop command to slave devices over
  // I2C immediately as the "Wire" library is interrupt driven as well.
}

void stopCommandISR() {
  startCommandLatch = false;
  displayUpdateString("Machine off.", "Press start.", 1);
  Serial.println("\n\n======================\n EMERGENCY STOP PRESSED \n======================\n\n");
}

// ---------- //
// FUNCTIONS  //
// ---------- //
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

void serialPrintStatus() {
  // Device run state, speed, alarms, runtime remaing
  Serial.println("\n **** RUNTIME ****");
  printAnalogue("Runtime remaining", runTimeRemaining, "min");

  Serial.println("\n **** BLOOD CIRCUIT ****");
  printStatus("Arterial Pressure", alarmState(artPressureAlarm));
  printStatus("Dialysate Pressure", alarmState(dialPressAlarm));
  printStatus("Venous Pressure: ", alarmState(venPressAlarm));
  printStatus("Blood Flow Rate: ", alarmState(bloodFlowAlarm));
  printStatus("Venous Temp: ", alarmState(venTempAlarm));
  printStatus("Air Detector: ", alarmState(airDetectAlarm));
  //Serial.println("Blood pump: ");

  Serial.println("\n **** DIALYSATE CIRCUIT ****");
  printStatus("Waste Pressure", alarmState(wastePressAlarm));
  printStatus("Dialysate Conductivity", alarmState(dialConductivityAlarm));
  printStatus("pH", alarmState(pHAlarm));
  printStatus("Dialysate Temp", alarmState(dialTempAlarm));
  printStatus("Water Level", alarmState(waterLevelAlarm));
  printStatus("Blood Leak", alarmState(bloodLeakAlarm));
  printStatus("Dialysate Level", alarmState(dialLevelAlarm));
}

void serialPrintValues() {
  Serial.println("\n **** BLOOD CIRCUIT ****");
  printAnalogue("Arterial Pressure", artPressVal, "mmHg");
  printAnalogue("Venous Pressure", venPressVal, "mmHg");
  printAnalogue("Blood Flow Rate", bloodFlowVal_S1, "mL/min");
  printAnalogue("Venous Temperature", venTempVal_S2, "°C");

  Serial.println("\n **** DIALYSATE CIRCUIT ****");
  printAnalogue("Dialysate Pressure", dialPressVal, "mmHg");
  printAnalogue("Waste Pressure", wastePressVal, "mmHg");
  printAnalogue("Dialysate Conductivity", dialConductivityVal_S1, "mS/cm");
  printAnalogue("Dialysate Temperature", dialTempVal_S1, "°C");
  printAnalogue("pH", pHVal_S1, "pH");
  printAnalogue("Water Level", waterLevelVal_S2, " % ");
  printAnalogue("Blood Leak Detector", bloodLeakVal_S2, "");
  printAnalogue("Dialysate Level", dialLevelVal_S2, " % ");
}

String alarmState(bool state) {
  return (state) ? "Alarm" : "Normal";
}

void printStatus(String description, String state) {
  // Check if E-Stop pressed
  if (!startCommandLatch) {
    return;
  }
  unsigned int strLen = description.length();

  // Append trailing spaces for alignment
  if (24 - strLen > 0) {
    for (int i = 0; i < (24 - strLen); i++) {
      description += " ";
    }
  }

  Serial.println(description + ": " + state);
}

void printAnalogue(String description, float value, String units) {
  // Check if E-Stop pressed
  if (!startCommandLatch) {
    return;
  }
  unsigned int strLen = description.length();

  // Append trailing spaces for alignment
  if (24 - strLen > 0) {
    for (int i = 0; i < (24 - strLen); i++) {
      description += " ";
    }
  }

  Serial.println((String)description + ": " + value + " " + units);
}
