# TRC3000-NephroMate
Current Tinkercad Project file found here: 
https://www.tinkercad.com/things/0snkyoh6L9Z-trc3000-circuitry-progress-submission/editel?sharecode=y1236gPW94cCms9rNrWc50mD1iBehIU5KO_R6rGxnRg

For details refer to IO-List-B saved in "TRC3000 - NephroMate/02 - Technical Work/01 - Software". A summary of the devices used and operating philosophy is depicted below.

## Circuit Components
**Blood Circuit**
* Arterial Pressure
* Inflow Pressure
* Venous Pressure
* Air Trap (Venous drip chamber)
* Air detector
* Venous Temperature
* Venous Clamps
* Blood Pump
* Heparin Pump

**Dialysis Circuit**
* Dialysate Temperature
* Dialysate Conductivity
* Blood Leak Detector
* Waste Pressure
* pH
* Dialysate Pressure
* Dialysate Level
* Mixer
* Dialysate Clamp
* Waste Pump
* Deaerator
* Heater

## Operating Philosophy
The Haemodialysis machine is controlled by 3 x Arduino Unos communicating via I2C. The machine starts operating for a configurable amount of time when the user presses the **start** button. The machine stops when any of the following occur:
* the predefined run-time has elapsed, or
* a fault condition triggers an emergency stop, or
* the user presses the **emergency stop** button.

The blood pump speed is controlled via a PID loop to maintain appropriate pressure on the inflow line. The Heparin pump runs at a user set speed, for a user set duration (configured in software). The speed of the heater is controlled by a PID loop to maintain appropriate dialysate temperature.

The current value of all analogue instruments is displayed on the LCD panels, with alarms indicated by an LED and buzzer.

If the dialysate temperature or conductivity is out-of-range, the dialysis opearation is stopped, and the dialysis diverted to waste. Similarly, if any pressure alarms are triggered on the blood circuit, or the venous temperature alarm is triggered the blood pump is stopped.
