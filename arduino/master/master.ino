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
bool venousClampFB_S1 = false;
bool bypassValveFB_S1 = false;
bool bloodPumpRunningFB_S1 = false;
bool dialPumpRunningFB_S1 = false;
bool mixerRunningFB_S1 = false;

// From Slave 2
double waterLevelVal_S2 = 0;
double venTempVal_S2 = 0;
double bloodLeakVal_S2 = 0;
double dialLevelVal_S2 = 0;
bool heaterRunningFB_S2 = false;
bool wastePumpRunningFB_S2 = false;
bool deaeratorRunningFB_S2 = false;
bool heparinPumpRunningFB_S2 = false;

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
const double FLOW_HIGH = 500;
const double FLOW_LOW = 200;
const double BLOOD_LEAK = 1000; // TO DO update value

// PID Constants & variables
const double TARGET_TEMP = 37; // target temperature (degrees Celsius) for dialysate solution
const double TARGET_FLOW = 250; // target user-configurable value in mL/min (usually 250-300, or 300-500, as prescribed by doctor)
const double kp_temp = 1;
const double ki_temp = 0;
const double kd_temp = 0;
const double kp_flow = 1;
const double ki_flow = 0;
const double kd_flow = 0;
int flow_PWM;
int temp_PWM;
long prevT = 0;
double eprev_temp = 0;
double eintegral_temp = 0;
double eprev_flow = 0;
double eintegral_flow = 0;

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
String tempPrinting = "";

// Timing and Sequencing Variables
unsigned long currentTime = 0;
unsigned long prevTime = 0;
double runTimeRemaining = 0;
unsigned long cyclePeriod = 100; // time in ms to alternate the screen values (note 100ms in tinkercad =/= 100ms real time)
unsigned long runTime = 4000; // time in ms to perform hemodialysis (refer comment above) (4000)
unsigned long hepRunTime = 1000; // Duration to run Heparin infusion (200)
double hepRunTimeRemaining = 0;
bool cycle = true; // alternate values displayed on LCD screen and in serial monitor
bool firstLoop = true;
int cycleState = 0;
bool finished = false;

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


  /*while(Serial.available()==0){
    // Collect user input
    }


    Serial.print("\nEnter heparin pump runtime (min): ");

    while(Serial.available()==0){
    // Collect user input
    }
    hepRunTime = (long)Serial.parseInt();*/

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
      /*Serial.print("\nEnter haemodialysis runtime (min): ");
        if (Serial.available() > 0){
        runTime = (long)Serial.parseInt();
        }*/
      displayUpdateString("Priming Complete", "Alarm tests pass", 1);
      delay(50); // hold message on screen for first loop
      Wire.beginTransmission(0x10);
      I2C_writeAnything(startCommandLatch);
      Wire.endTransmission();
      Wire.beginTransmission(0x11);
      I2C_writeAnything(startCommandLatch);
      Wire.endTransmission();
      Serial.println("\nHaemodialysis started\n");
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
    // ** TO DO **: Verify operating scaled values
    artPressVal = scaleInput(artPressVal, 0, 466, -300.0, -30.0);
    dialPressVal = scaleInput(dialPressVal, 0, 466, 50.0, 250.0);
    venPressVal = scaleInput(venPressVal, 0, 466, 50.0, 250.0);
    wastePressVal = scaleInput(wastePressVal, 0, 466, 0, 400.0);

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
    // ** TO DO ** CHECK SIZE OF BOOL SENT OVER I2C
    Wire.requestFrom(0x10, 21);
    I2C_readAnything(dialConductivityVal_S1);
    I2C_readAnything(pHVal_S1);
    I2C_readAnything(dialTempVal_S1);
    I2C_readAnything(bloodFlowVal_S1);
    I2C_readAnything(venousClampFB_S1);
    I2C_readAnything(bypassValveFB_S1);
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
    hepRunTimeRemaining = (hepRunTime - currentTime) / 18;

    if (currentTime - prevTime > cyclePeriod) {
      if (cycle) {
        displayUpdateValue("Art Press", artPressVal, "Inf Press", dialPressVal);
        //serialPrintStatus();
        //serialPrintValuesStatus();
        printStatus();
      } else {
        displayUpdateValue("Ven Press", venPressVal, "Wst Press", wastePressVal);
        //serialPrintDeviceStatus();
        //printDevices();
      }
      prevTime = currentTime;
      cycle = !cycle;
    }

    // PID Control
    double dt = ((double)(currentTime - prevT)) / (1.0e3); // convert from ms to s
    prevT = currentTime;
    temp_PWM = PIDcontrol(TARGET_TEMP, dialTempVal_S1, dt, kp_temp, ki_temp, kd_temp, eprev_temp, eintegral_temp);
    flow_PWM = PIDcontrol(TARGET_FLOW, bloodFlowVal_S1, dt, kp_flow, ki_flow, kd_flow, eprev_flow, eintegral_flow);
    int temp_PWMPrev;
    int flow_PWMPrev;

    // Second Check if Emergency Stop Button has been pressed and relay stop to slave devices
    // if (startCommandLatch != startCommandLatchPrev || wastePressureAlarm != wastePressureAlarmPrev || temp_PWM != temp_PWMPrev ) {
    Wire.beginTransmission(0x11); // slave 2
    I2C_writeAnything(startCommandLatch);
    I2C_writeAnything(temp_PWM); // heater motor PWM
    I2C_writeAnything(wastePressureAlarm);
    I2C_writeAnything(hepRunTimeRemaining);
    Wire.endTransmission();
    //}

    // Send fault conditions to slave devices if any values have changed
    // Only one receive function so need to send all values
    if (startCommandLatch != startCommandLatchPrev || bloodPumpFault != bloodPumpFaultPrev || airDetectAlarm != airDetectAlarmPrev || flow_PWM != flow_PWMPrev) {
      Wire.beginTransmission(0x10); // slave 1
      I2C_writeAnything(startCommandLatch);
      I2C_writeAnything(bloodPumpFault);
      I2C_writeAnything(airDetectAlarm); // Activate dialysate and venous clamps
      I2C_writeAnything(flow_PWM); // blood pump PWM
      Wire.endTransmission();
    }

    // Update Previous Values
    startCommandLatchPrev = startCommandLatch;
    bloodPumpFaultPrev = bloodPumpFault;
    airDetectAlarmPrev = airDetectAlarm;
    wastePressureAlarmPrev = wastePressureAlarm;
    temp_PWMPrev = temp_PWM;
    flow_PWMPrev = flow_PWM;
  }

  if (currentTime > runTime && !finished) {
    Serial.println("\nHaemodialysis complete\n");
    finished = true;
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
// Update Screen
void displayUpdateValue(String text1, double value1, String text2, double value2) {
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

String alarmState(bool state) {
  return (state) ? "Alarm" : "Normal";
}

void printAnalogueStatus(String description, double value, String units, String state) {
  // Check if E-Stop pressed
  if (!startCommandLatch) {
    return;
  }
  unsigned int strDescLen = description.length();
  unsigned int strUnitsLen = units.length();
  unsigned int strValueLen = 0;
  unsigned int totalLen = 0;
  if (abs(value) < 10) {
    strValueLen = 0;
  } else if (abs(value) < 100) {
    strValueLen = 1;
  }  else if (abs(value) < 1000) {
    strValueLen = 2;
  }
  else {
    strValueLen = 3;
  }

  if (value < 0) {
    strValueLen += 1; // account for negative sign
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

  Serial.println((String)description + ": " + value + " " + units + ": " + state);
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

  Serial.println((String)description + ": " + value + " " + units);
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

int PIDcontrol(double target, double val, double dt, double kp, double ki, double kd, double & eprev, double & eintegral) {
  double e = target - val;
  double de_dt = (e - eprev) / dt;
  eintegral = eintegral + e * dt;
  eprev = e;
  int u = (int) kp * e + ki * eintegral + kd * de_dt;
  if (u < 0) return 0; // set minimum pump PWM to be zero
  if (u > 0) return min(abs(u), 255); // maximum PWM value is 255
}
void state0() {
  Serial.println("\n              **** RUNTIME ****");
  printAnalogue("Runtime remaining", runTimeRemaining, "min");
  printAnalogue("Heparin time remaining", hepRunTimeRemaining, "min");
}

void state1() {
  Serial.println("\n              **** BLOOD CIRCUIT ****");
  printAnalogueStatus("Arterial Pressure", artPressVal, "mmHg", alarmState(artPressureAlarm));
  printAnalogueStatus("Venous Pressure", venPressVal, "mmHg", alarmState(venPressAlarm));
  printAnalogueStatus("Blood Flow Rate", bloodFlowVal_S1, "mL/min", alarmState(bloodFlowAlarm));
  printAnalogueStatus("Venous Temperature", venTempVal_S2, "\xB0""C", alarmState(venTempAlarm));
}

void state2() {
  Serial.println("\n              **** DIALYSATE CIRCUIT ****");
  printAnalogueStatus("Dialysate Pressure", dialPressVal, "mmHg", alarmState(dialPressAlarm));
  printAnalogueStatus("Waste Pressure", wastePressVal, "mmHg", alarmState(wastePressAlarm));
  printAnalogueStatus("Dialysate Conductivity", dialConductivityVal_S1, "mS/cm", alarmState(dialConductivityAlarm));
  printAnalogueStatus("Dialysate Temperature", dialTempVal_S1, "\xB0""C", alarmState(dialTempAlarm));
  printAnalogueStatus("pH", pHVal_S1, "pH", alarmState(pHAlarm));
  printAnalogueStatus("Water Level", waterLevelVal_S2, " % ", alarmState(waterLevelAlarm));
  printAnalogueStatus("Blood Leak Detector", bloodLeakVal_S2, "", alarmState(bloodLeakAlarm));
  printAnalogueStatus("Dialysate Level", dialLevelVal_S2, " % ", alarmState(dialLevelAlarm));
}

void state3() {
  Serial.println("\n              **** DEVICES STATUS ****");
  printAnalogueStatus("Mixer", (mixerRunningFB_S1 * 100), "%", motorState(mixerRunningFB_S1));
  printAnalogueStatus("Blood Pump", (map(flow_PWM, 0, 255, 0, 100)), "%", motorState(bloodPumpRunningFB_S1));
  printAnalogueStatus("Dialysate Pump", (dialPumpRunningFB_S1 * 100), "%", motorState(dialPumpRunningFB_S1));
  printAnalogueStatus("Bypass Valve", (bypassValveFB_S1 * 100), "%", valveState(bypassValveFB_S1));
  printAnalogueStatus("Venous Clamp", (venousClampFB_S1 * 100), "%", servoState(venousClampFB_S1));
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
    cycleState = 0;
  }
}
