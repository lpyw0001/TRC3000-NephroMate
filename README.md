# TRC3000-NephroMate
Current Tinkercad Project file found here: 
https://www.tinkercad.com/things/0IEXT8VFWHw-trc3000-circuitry/editel?sharecode=CDYlMBWTPzYRRWJZkkAYgxNuBFqU5zzFL4ohuK_AvUA

For details refer to IO-List-B saved in "TRC3000 - NephroMate/02 - Technical Work/01 - Software". A summary of the devices used and operating philosophy is described below.

## Circuit Components
**Blood Circuit**
* Arterial Pressure
* Venous Pressure
* Blood flow rate
* Air Trap (Venous drip chamber)
* Air detector
* Venous Temperature
* Venous Clamps
* Blood Pump
* Heparin Pump

**Dialysis Circuit**
* Dialysate Pressure
* Dialysate Conductivity
* pH
* Dialysate Temperature
* Dialysate Level
* Water Level
* Blood Leak Detector
* Bypass Valve
* Dialysate Pump
* Mixer
* Deaerator
* Reverse Osmosis
* Heater

**Waste Removal**
* Waste Pressure
* Waste pump (Ultrafiltration)

## Operating Philosophy
The Haemodialysis machine is controlled by 3 x Arduino Unos communicating via I2C, operating with a master-slave architecture (1 x Master, 2 x Slaves). The machine starts operating for a configurable amount of time when the user presses the **start** button. The machine stops completely when any of the following occur:
* the predefined run-time has elapsed, or
* air is detected in the lines, or
* the user presses the **emergency stop** button.

When an emergency stop is triggered, all devices stop running, the servo's actuate to clamp the lines and the bypass valve diverts any residual dialysate flow to waste. Before commencing a session the user is required to enter the following parameters:
* Session length (minutes)
* Desired fluid removal (L)
* Patient weight (kg)
* Heparin duration (minutes) - if required

Once entered the machine calculates the ultrafiltration rate (UFR) and if this rate is unsafe will prompt the user to re-enter values until a safe rate is calculated. All analogue values are to be read and scaled on their attached microcontroller and the scaled value relayed to the master for process control. The master will print these values to the serial monitor. The current run-state of the machine can be viewed on the LCD panel. When an alarm is triggered, there will be both an audible and visual indicator.

**Blood Circuit**  
The heparin pump, if required, will run for the user input duration, at a fixed speed. The venous clamps will be actuated when an emergency stop is triggered, either by user input or air detection. The blood pump speed is controller via a PID loop to maintain an appropriate flow rate on the inflow line. The blood pump will stop when a fault condition is triggered, which includes the following:
* Emergency stop
* Arterial pressure alarm
* Venous pressure alarm
* Blood flow rate alarm
* Water level alarm
* Venous temperature alarm
* Blood leak detected

**Dialysate Circuit**  
The bypass valve actuates to divert dialysate to waste when any of the following fault conditions occur:
* Emergency stop
* Dialysate conductivity alarm
* pH alarm
* Dialysate temperature alarm
* Dialysate level alarm

The dialysate pump runs at a fixed speed continuously unless a dialysate fault condition occurs, which is any of the five listed above. The mixer, deaerator and reverse osmosis machine run continuously. The heater is controlled by a PID loop to maintain appropriate dialysate temperature. 

**Waste Circuit**  
The ultrafiltration pump runs at the speed calculated to maintain the required pressure for the desired fluid removal. The waste pump will stop when any of the following fault conditions are triggered:
* Emergency stop
* Waste pressure alarm

The circuits lines are primed between every session, and automatic verification of alarms is performed before every session.
