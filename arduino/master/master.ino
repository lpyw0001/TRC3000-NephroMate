// TRC3000 - NephroMate
// Master
#include <Wire.h>
#include <LiquidCrystal.h>
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
int artPressPin = A0;
int dialPressPin = A1;
int venPressPin = A2;
int wastePressPin = A3;

// Analogue Values
long artPressVal = 0;
long dialPressVal = 0;
long venPressVal = 0;
long wastePressVal = 0;

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
long dialConductivityVal_S1 = 0;
long pHVal_S1 = 0;
long dialTempVal_S1 = 0;
long bloodFlowVal_S1 = 0;
bool venousClampFB_S1 = false;
bool bypassValveCMDFB_S1 = false;
bool bloodPumpRunningFB_S1 = false;
bool dialPumpRunningFB_S1 = false;
bool mixerRunningFB_S1 = false;

// From Slave 2
long waterLevelVal_S2 = 0;
long venTempVal_S2 = 0;
long bloodLeakVal_S2 = 0;
long dialLevelVal_S2 = 0;
bool heaterRunningFB_S2 = false;
bool wastePumpRunningFB_S2 = false;
bool deaeratorRunningFB_S2 = false;
bool heparinPumpRunningFB_S2 = false;

// Alarm Limits (note multiplied by FLOAT_SCALE)
const int PRESSURE_HIGH = 15000; // TO DO update value
const int PRESSURE_LOW = -23000; // TO DO update value
const int LEVEL_LOW = 1000;
const int TEMP_LOW = 3600;
const int TEMP_HIGH = 4200;
const int CONDUCTIVITY_LOW = 1200;
const int CONDUCTIVITY_HIGH = 1600;
const int PH_LOW = 680;
const int PH_HIGH = 760;
const int AIR_DETECT = 2000; // TO DO update value
const long FLOW_HIGH = 50000;
const long FLOW_LOW = 20000;
const int BLOOD_LEAK = 1000; // TO DO update value

// PID Constants & variables
int TARGET_TEMP = 37; // target temperature (degrees Celsius) for dialysate solution
int TARGET_FLOW = 250; // target user-configurable value in mL/min (usually 250-300, or 300-500, as prescribed by doctor)
int kp_temp = 1;
int ki_temp = 0;
int kd_temp = 0;
int kp_flow = 1;
int ki_flow = 0;
int kd_flow = 0;
int flow_PWM;
int temp_PWM;
long prevT = 0;
int eprev_temp = 0;
int eintegral_temp = 0;
int eprev_flow = 0;
int eintegral_flow = 0;

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
bool wastePressureAlarm = false;
bool anyAlarmTriggered = false;
bool bloodPumpFault = false;
bool wastePressureAlarmPrev = false;

// Initialise LCD
LiquidCrystal lcd(12, 11, 6, 7, 4, 5); // (rs,enable,d4,d5,d6,d7)

// Timing and Sequencing Variables
unsigned long currentTime = 0;
unsigned long prevTime = 0;
long runTimeRemaining = 0;
unsigned int cyclePeriod = 80; // time in ms to alternate the screen values (note 40 in tinkercad =/= 40 real time)
unsigned int runTimeMin = 10; // time in ms to perform hemodialysis (refer comment above) (4000)
unsigned long runTimeMs = 10;
unsigned int hepRunTime = 1000; // Duration to run Heparin infusion (200)
unsigned long hepRunTimeMs = 10;
long hepRunTimeRemaining = 0;
int cycle = 0; // alternate values displayed on LCD screen and in serial monitor
bool firstLoop = true;
int cycleState = 0;
bool finished = false;
int lcdCycleState = 0;
int flagCount = 0;
bool serialInputFlag = false;
bool fluidFlag = false;
bool runTimeFlag = false;
bool hepFlag = false;
bool weightFlag = false;

// Other Process Control
long desiredFluidRemoval = 0;
bool bypassValveCMDPrev = false;
bool bypassValveCMD = false;
long patientWeight = 0;
int UFRate = 0;
bool UFFlag = false;

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

  const String sesDurStr = "Duration (min): ";
  const String desFluidStr = "Fluid Removal (L): ";
  const String weightStr = "Patient weight (kg): ";
  const String hepStr = "Hep Duration (min) 'z' for none: " ;

  if (!serialInputFlag) {
    //Serial.println(F("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")); //'Clear' serial monitor output
    serialInputFlag = true;
  }

  if (!finished) {
    displayUpdateString("Machine off.", "Press start.", 0);
  }

  if (!runTimeFlag) {
    runTimeMin = SerialDataEntry(sesDurStr, 99);
    runTimeFlag = true;
  }

  if (!fluidFlag) {
    desiredFluidRemoval = SerialDataEntry(desFluidStr, 1);
    desiredFluidRemoval = desiredFluidRemoval * 1000; //convert to mL
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

  UFRate = FLOAT_SCALE * ((FLOAT_SCALE * desiredFluidRemoval) / (FLOAT_SCALE * runTimeMin / 60)) / (FLOAT_SCALE * patientWeight);
  if (!UFFlag && UFRate != 0) {
    Serial.print(F("UF Rate: "));
    Serial.print(UFRate / FLOAT_SCALE);
    Serial.print(F("."));
    Serial.print(UFRate % FLOAT_SCALE);
    Serial.println(F(" mL/h/kg"));
  }
  if ((UFRate / FLOAT_SCALE) > 13) {
    Serial.println(F("UF Rate too high"));
    hepFlag = false;
    weightFlag = false;
    fluidFlag = false;
    runTimeFlag = false;
  }
  else if (UFRate > 0) {
    UFFlag = true;
  }

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

  if (runTimeFlag && fluidFlag) {
    while ((startCommandLatch) & (currentTime < runTimeMs)) {
      flagCount = 0;
      if (firstLoop) {
        runTimeMs = runTimeMin * 100;
        hepRunTimeMs = hepRunTime * 100;
        displayUpdateString("Priming Complete", "Alarm tests pass", 1);
        delay(50); // hold message on screen for first loop
        Wire.beginTransmission(0x10);
        I2C_writeAnything(startCommandLatch);
        Wire.endTransmission();
        Wire.beginTransmission(0x11);
        I2C_writeAnything(startCommandLatch);
        Wire.endTransmission();
        Serial.println(F("\n\nSession started\n"));
        displayUpdateString("Haemodialysis", "machine running", 1);
        delay(10);
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
      artPressVal = scaleInput(artPressVal, 0, 466, -300, -30);
      dialPressVal = scaleInput(dialPressVal, 0, 466, 0, 400);
      venPressVal = scaleInput(venPressVal, 0, 466, 50, 250);
      wastePressVal = scaleInput(wastePressVal, 0, 466, 0, 400);

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

      // Request IO from slave 0x10 (4 x analogue values, 5 x boolean values)
      Wire.requestFrom(0x10, 21);
      I2C_readAnything(dialConductivityVal_S1);
      I2C_readAnything(pHVal_S1);
      I2C_readAnything(dialTempVal_S1);
      I2C_readAnything(bloodFlowVal_S1);
      I2C_readAnything(venousClampFB_S1);
      I2C_readAnything(bypassValveCMDFB_S1);
      I2C_readAnything(bloodPumpRunningFB_S1);
      I2C_readAnything(dialPumpRunningFB_S1);
      I2C_readAnything(mixerRunningFB_S1);

      // Request IO from slave 0x11 (4 x analogue inputs, 4 x boolean values)
      Wire.requestFrom(0x11, 20);
      I2C_readAnything(waterLevelVal_S2);
      I2C_readAnything(venTempVal_S2);
      I2C_readAnything(bloodLeakVal_S2);
      I2C_readAnything(dialLevelVal_S2);
      I2C_readAnything(heaterRunningFB_S2);
      I2C_readAnything(wastePumpRunningFB_S2);
      I2C_readAnything(deaeratorRunningFB_S2);
      I2C_readAnything(heparinPumpRunningFB_S2);

      // Check Alarm Conditions
      artPressureAlarm = (artPressVal > PRESSURE_HIGH || artPressVal < PRESSURE_LOW);
      venPressAlarm = (venPressVal > PRESSURE_HIGH || venPressVal < PRESSURE_LOW);
      airDetectAlarm = airDetectValue > AIR_DETECT;
      dialConductivityAlarm = dialConductivityVal_S1 > CONDUCTIVITY_HIGH || dialConductivityVal_S1 < CONDUCTIVITY_LOW;
      pHAlarm = pHVal_S1 > PH_HIGH || pHVal_S1 < PH_LOW;
      dialTempAlarm = dialTempVal_S1 > TEMP_HIGH || dialTempVal_S1 < TEMP_LOW;
      bloodFlowAlarm = bloodFlowVal_S1 > FLOW_HIGH || bloodFlowVal_S1 < FLOW_LOW;
      waterLevelAlarm = waterLevelVal_S2 < LEVEL_LOW;
      venTempAlarm = venTempVal_S2 < TEMP_LOW || venTempVal_S2 > TEMP_HIGH;
      bloodLeakAlarm = bloodLeakVal_S2 > BLOOD_LEAK;
      dialLevelAlarm = dialLevelVal_S2 < LEVEL_LOW;
      wastePressureAlarm = wastePressVal > PRESSURE_HIGH;

      anyAlarmTriggered = artPressureAlarm || venPressAlarm || airDetectAlarm || dialConductivityAlarm || pHAlarm || dialTempAlarm || bloodFlowAlarm || waterLevelAlarm || venTempAlarm || bloodLeakAlarm || dialLevelAlarm;

      // Check Device Fault Conditions
      bloodPumpFault = airDetectAlarm || venPressAlarm || artPressureAlarm || bloodFlowAlarm || waterLevelAlarm || venTempAlarm;
      bypassValveCMD = pHAlarm || dialConductivityAlarm || dialTempAlarm || dialLevelAlarm;

      // Stop Operating if air detected
      if (airDetectAlarm) {
        stopCommandISR();
      }

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
      runTimeRemaining = (runTimeMs > currentTime) ? (runTimeMs - currentTime) / 100 : 0; // arbitrary scaling to return a reasonable value
      hepRunTimeRemaining = (hepRunTimeMs > currentTime) ? (hepRunTimeMs - currentTime) / 100 : 0;

      if (currentTime - prevTime > cyclePeriod) {
        printStatus();
        prevTime = currentTime;
      }

      // PID Control
      double dt = ((double)(currentTime - prevT)) / (1.0e3); // convert from ms to s
      prevT = currentTime;
      temp_PWM = PIDcontrol(TARGET_TEMP, dialTempVal_S1, dt, kp_temp, ki_temp, kd_temp, eprev_temp, eintegral_temp);
      flow_PWM = PIDcontrol(TARGET_FLOW, bloodFlowVal_S1, dt, kp_flow, ki_flow, kd_flow, eprev_flow, eintegral_flow);
      int temp_PWMPrev;
      int flow_PWMPrev;

      // Second Check if Emergency Stop Button has been pressed and relay stop to slave devices
      Wire.beginTransmission(0x11); // slave 2
      I2C_writeAnything(startCommandLatch);
      I2C_writeAnything(temp_PWM); // heater motor PWM
      I2C_writeAnything(wastePressureAlarm);
      I2C_writeAnything(hepRunTimeRemaining);
      Wire.endTransmission();

      // Send fault conditions to slave devices if any values have changed
      // Only one receive function so need to send all values
      if (startCommandLatch != startCommandLatchPrev || bloodPumpFault != bloodPumpFaultPrev || airDetectAlarm != airDetectAlarmPrev || flow_PWM != flow_PWMPrev || bypassValveCMD != bypassValveCMDPrev) {
        Wire.beginTransmission(0x10); // slave 1
        I2C_writeAnything(startCommandLatch);
        I2C_writeAnything(bloodPumpFault);
        I2C_writeAnything(airDetectAlarm); // Activate dialysate and venous clamps
        I2C_writeAnything(flow_PWM); // blood pump PWM
        I2C_writeAnything(bypassValveCMD);
        Wire.endTransmission();
      }

      // Update Previous Values
      startCommandLatchPrev = startCommandLatch;
      bloodPumpFaultPrev = bloodPumpFault;
      airDetectAlarmPrev = airDetectAlarm;
      wastePressureAlarmPrev = wastePressureAlarm;
      temp_PWMPrev = temp_PWM;
      flow_PWMPrev = flow_PWM;
      bypassValveCMDPrev = bypassValveCMD;

    }
    if (currentTime > runTimeMs && !finished) {
      Serial.println(F("\nDialysis complete\n"));
      displayUpdateString("Haemodialysis", "Complete", 1);
      finished = true;
    }
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
  Serial.println(F("\n\n===\n EMERGENCY STOP \n===\n\n"));
}

// ---------- //
// FUNCTIONS  //
// ---------- //
// Update LCD Screen (text only)
void displayUpdateString(const char *text1, const char *text2, bool clear) {
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
// fixed point version of the map() function
// y = ((y2-y1/(x2-x1))*(x-x1)+y1
// x1,x2 are the input min/max
// y1,y2 are the output min/max
// x is the value to scale
// y is the scaled value
long scaleInput(long rawValue, long rawMin, long rawMax, long scaledMin, long scaledMax) {
  return ((FLOAT_SCALE * (scaledMax - scaledMin) / (rawMax - rawMin)) * (rawValue - rawMin) + FLOAT_SCALE * scaledMin);
}
// =========================//
// Serial Logging Functions
// =========================//
String alarmState(bool state) {
  return (state) ? "Alarm" : "Normal";
}

void printAnalogueStatus(String description, long value, String units, String state, bool scale) {
  // Check if E-Stop pressed
  if (!startCommandLatch) {
    return;
  }
  unsigned int strDescLen = description.length();
  unsigned int strUnitsLen = units.length();
  unsigned int strValueLen = 0;
  unsigned int totalLen = 0;
  long valueTemp = 0;

  if (scale) {
    valueTemp = value / FLOAT_SCALE;
  }
  else {
    valueTemp = value;
  }

  if (abs(valueTemp) < 10) {
    strValueLen = 0;
  }
  else if (abs(valueTemp) < 100) {
    strValueLen = 1;
  }
  else if (abs(valueTemp) < 1000) {
    strValueLen = 2;
  }
  else {
    strValueLen = 3;
  }

  if (valueTemp < 0) {
    strValueLen++; // account for negative sign
  }

  if (scale && (value % FLOAT_SCALE) == 0) {
    strValueLen--;
  }

  // Append trailing spaces for alignment
  if (24 - strDescLen > 0) {
    for (int i = 0; i < (24 - strDescLen); i++) {
      description += " ";
    }
  }

  totalLen = strUnitsLen + strValueLen;

  if (12 - totalLen > 0) {
    for (int i = 0; i < (12 - totalLen); i++) {
      units += " ";
    }
  }

  Serial.print(description);
  Serial.print(": ");

  if (scale) {
    Serial.print(value / FLOAT_SCALE);
    Serial.print(F("."));
    Serial.print(abs(value % FLOAT_SCALE));
    Serial.println(" " + units + ": " + state);
  }
  else {
    Serial.print((String)value);
    Serial.print(" "); // additional spaces for padding
    Serial.print(units);
    Serial.print("   : " ); // additional spaces for padding
    Serial.println(state);
  }
}

void printAnalogue(String description, double value, String units) {
  // Check if E-Stop pressed
  if (!startCommandLatch) {
    return;
  }
  unsigned int strDescLen = description.length();

  // Append trailing spaces for alignment
  if (24 - strDescLen > 0) {
    for (int i = 0; i < (24 - strDescLen); i++) {
      description += " ";
    }
  }

  Serial.print(description);
  Serial.print(": ");
  Serial.print((String)value);
  Serial.print(" ");
  Serial.println(units);
}

String servoState(bool state) {
  return (state) ? "Active" : "Off";
}

String valveState(bool state) {
  return (state) ? "Open" : "Closed";
}

String motorState(bool state) {
  return (state) ? "Running" : "Off";
}

int PIDcontrol(int target, int val, int dt, int kp, int ki, int kd, int & eprev, int & eintegral) {
  int e = target - val;
  int de_dt = (e - eprev) / dt;
  eintegral = eintegral + e * dt;
  eprev = e;
  int u = (int) kp * e + ki * eintegral + kd * de_dt;
  if (u < 0) return 0; // set minimum pump PWM to be zero
  if (u > 0) return min(abs(u), 255); // maximum PWM value is 255
}
void state0() {
  Serial.println(F("\n**** RUNTIME ****"));
  printAnalogue("Session time rem", runTimeRemaining, "min");
}

void state1() {
  printAnalogue("Hep time rem", hepRunTimeRemaining, "min");
  Serial.println(F("\n**** BLOOD CIRCUIT ****"));
}

void state2() {
  printAnalogueStatus("Arterial Press", artPressVal, "mmHg", alarmState(artPressureAlarm), true);
  printAnalogueStatus("Venous Press", venPressVal, "mmHg", alarmState(venPressAlarm), true);
}

void state3() {
  printAnalogueStatus("Bld Flow", bloodFlowVal_S1, "mL/min", alarmState(bloodFlowAlarm), true);
  printAnalogueStatus("Venous Temp", venTempVal_S2, "\xB0""C", alarmState(venTempAlarm), true);
  Serial.println(F("\n**** DIALYSATE CIRCUIT ****"));
}

void state4() {
  printAnalogueStatus("Dial Press", dialPressVal, "mmHg", alarmState(dialPressAlarm), true);
  printAnalogueStatus("Waste Press", wastePressVal, "mmHg", alarmState(wastePressAlarm), true);
}

void state5() {
  printAnalogueStatus("Dial Cond", dialConductivityVal_S1, "mS/cm", alarmState(dialConductivityAlarm), true);
  printAnalogueStatus("Dial Temp", dialTempVal_S1, "\xB0""C", alarmState(dialTempAlarm), true);
}

void state6() {
  printAnalogueStatus("pH", pHVal_S1, "pH", alarmState(pHAlarm), true);
  printAnalogueStatus("Water Lvl", waterLevelVal_S2, "%", alarmState(waterLevelAlarm), true);
}

void state7() {
  printAnalogueStatus("Bld Leak Detect", bloodLeakVal_S2, "", alarmState(bloodLeakAlarm), true);
  printAnalogueStatus("Dial Lvl", dialLevelVal_S2, "%", alarmState(dialLevelAlarm), true);
}

void state8() {
  Serial.println(F("\n**** DEVICES STATUS ****"));
  printAnalogueStatus("Bld Pump", (map(flow_PWM, 0, 255, 0, 100)), "%", motorState(bloodPumpRunningFB_S1), false);
}

void state9() {
  printAnalogueStatus("UF Pump", wastePumpRunningFB_S2, "%", motorState(wastePumpRunningFB_S2), false);
  printAnalogueStatus("Heater", (heaterRunningFB_S2 * 100), "%", motorState(heaterRunningFB_S2), false);
}

void state10() {
  printAnalogueStatus("Bypass Valve", (bypassValveCMDFB_S1 * 100), "%", valveState(bypassValveCMDFB_S1), false);
  printAnalogueStatus("Venous Clamp", (venousClampFB_S1 * 100), "%", servoState(venousClampFB_S1), false);
}

void printStatus() {
  if (cycleState == 0) {
    state0();
    cycleState++;
  }
  else if (cycleState == 1) {
    state1();
    cycleState++;
  }
  else if (cycleState == 2) {
    state2();
    cycleState++;
  }
  else if (cycleState == 3) {
    state3();
    cycleState++;
  }
  else if (cycleState == 4) {
    state4();
    cycleState++;
  }
  else if (cycleState == 5) {
    state5();
    cycleState++;
  }
  else if (cycleState == 6) {
    state6();
    cycleState++;
  }
  else if (cycleState == 7) {
    state7();
    cycleState++;
  }
  else if (cycleState == 8) {
    state8();
    cycleState++;
  }
  else if (cycleState == 9) {
    state9();
    cycleState++;
  }
  else if (cycleState == 10) {
    state10();
    cycleState = 0;
  }
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
