# Project 3: Windshield Wiper Subsystem

Team Members: Madisen Bailey, Vivaan Gupta, Niju Khanal  
---

## System Description

This project expands the Driver’s Education Smart Car system to include a windshield wiper subsystem in addition to the ignition subsystem.

The ignition subsystem enables engine start only when both seats are occupied and both seatbelts are fastened. When these safety conditions are satisfied, the yellow LED indicates ignition readiness. If the ignition button is pressed while enabled, the engine starts, the blue LED turns on, and the LCD displays “Engine started.” If the safety conditions are not met, ignition is inhibited, the buzzer sounds, and the LCD displays “Ignition inhibited” along with the specific safety violations. Once started, the engine remains running until the ignition button is pressed again.

The windshield wiper subsystem operates only when the engine is running and supports four modes: HI, LO, INT, and OFF. In HI and LO modes, the wipers continuously sweep between 0° and a maximum angle at different speeds. In INT mode, the wipers operate at low speed and pause at 0° for a selected delay time (SHORT, MEDIUM, or LONG). When switched to OFF or when the engine is turned off, the wipers complete their current cycle and return to 0°.

---

## Design Alternatives

### Motor Selection
The project allowed the choice between a continuous rotation servo and a positional servo motor. We selected a standard positional servo motor because it allows direct control of the wiper angle and guarantees returning to a known rest position (0°). A continuous servo would only allow speed control without fixed positioning, making it more difficult to implement correct shutdown behavior.

### ADC Threshold Mapping
We divided the ADC range into threshold intervals corresponding to the required number of modes (HI, LO, INT, OFF) and delay settings (SHORT, MEDIUM, LONG). This simplified the selection logic and ensured stable mode detection without requiring precise knob positioning.

### Initialization Structure
Instead of placing all configuration code directly inside app_main(), we separated hardware setup into initialization functions (GPIO, PWM/servo, ADC, and LCD setup). This improved readability, reduced repetition, and allowed the main loop to focus on system behavior and control logic.

---
## Testing Results

### Ignition Subsystem
| Specification | Test Process | Results |
| --- | --- | --- |
| Enable engine start (yellow LED) only when required safety conditions are met. | <ol><li>Press all seat and seatbelt buttons.</li><li>Leave one safety condition unmet.</li><li>Leave all conditions unmet.</li></ol> | <ol><li>Yellow LED illuminated when all conditions were satisfied.</li><li>Yellow LED remained off when any condition was missing.</li><li>Ignition was not enabled when no conditions were satisfied.</li></ol> |
| Print appropriate error messages if ignition is attempted without all safety conditions met. | <ol><li>Leave one or more safety conditions unmet.</li><li>Press ignition button.</li></ol> | <ol><li>Alarm activated.</li><li>LCD displayed correct error messages (e.g., “Passenger seat not occupied,” “Driver seatbelt not fastened”).</li></ol> |
| Start engine when ignition is enabled and ignition button is pressed. | <ol><li>Satisfy all safety conditions (yellow LED on).</li><li>Press ignition button.</li></ol> | <ol><li>Blue LED illuminated.</li><li>Yellow LED turned off.</li><li>“Engine started” message displayed.</li><li>Engine remained running after button release.</li></ol> |
| Stop engine when ignition button is pressed while engine is running. | <ol><li>Engine running (blue LED on).</li><li>Press ignition button.</li></ol> | <ol><li>Engine stopped.</li><li>Blue LED turned off.</li></ol> |

### Windshield Wiper Subsystem
| Specification | Test Process | Results |
| --- | --- | --- |
| Wipers operate only when engine is running. | <ol><li>Turn engine OFF.</li><li>Select HI, LO, or INT.</li></ol> | <ol><li>Wipers did not move when engine was OFF.</li></ol> |
| HI and LO modes run continuously at different speeds. | <ol><li>Turn engine ON.</li><li>Select HI mode.</li><li>Select LO mode.</li></ol> | <ol><li>Continuous wiping observed in both modes.</li><li>HI speed was greater than LO speed.</li></ol> |
| INT mode runs with selectable delay (SHORT, MEDIUM, LONG). | <ol><li>Turn engine ON.</li><li>Select INT mode.</li><li>Select SHORT, MEDIUM, and LONG delays.</li></ol> | <ol><li>Wipers completed one cycle.</li><li>Pause duration matched selected delay time.</li></ol> |
| Wipers return to 0° when switched OFF or engine turned OFF. | <ol><li>Operate wipers in any mode.</li><li>Switch to OFF or turn engine OFF.</li></ol> | <ol><li>Wipers completed current cycle.</li><li>Motor returned to 0° position.</li></ol> |
