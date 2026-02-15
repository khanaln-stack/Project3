# Project 3: Windshield Wiper Subsystem
Team Members: Madisen Bailey, Vivaan Gupta, Niju Khanal

## Project Description
This project implements an enhanced Driver’s Education Smart Car system that integrates ignition control and a windshield wiper subsystem to simulate realistic vehicle behavior. When the driver sits down, a welcome message is displayed once, and ignition is enabled only when required safety conditions (seat occupancy and seatbelt fastening) are met. When these conditions are satisfied, the yellow LED turns on to indicate ignition is ready. If the ignition button is pressed while enabled, the engine starts, the blue LED illuminates, and a confirmation message is displayed. If conditions are not satisfied, the system inhibits ignition, sounds the buzzer, and displays the specific reasons on the LCD. The engine remains running until the ignition button is pressed again.

The windshield wiper subsystem operates only when the engine is running and supports HI, LO, INT, and OFF modes. In INT mode, the delay time (SHORT, MEDIUM, LONG) is selected via potentiometer and displayed on the LCD. When switched to OFF or when the engine is turned off, the wipers complete their current cycle and return to 0 degrees.

## Design Alternatives
Several design approaches were considered during development. We evaluated using one large unified state machine versus modular subsystem logic; we chose a modular design to improve readability, debugging, and independent operation of the ignition and wiper systems while still responding to engine state. For mode and delay selection, we selected potentiometers with ADC threshold detection instead of multiple digital switches to reduce hardware complexity. For intermittent wiping, we used non-blocking timing rather than blocking delays so the system remains responsive during delay periods. Finally, instead of stopping the wipers immediately when switched OFF or when the engine turns off, we implemented logic to complete the current cycle and return to 0 degrees to better simulate realistic automotive behavior.

## Testing Results

### Ignition Subsystem
| Specification | Test Process | Results |
| --- | --- | --- |
| Enable engine start (yellow LED) only when required safety conditions are met. | <ol><li>Turn ON all seat and seatbelt slider switches..</li><li>Leave one safety condition unmet.</li><li>Leave all conditions unmet.</li></ol> | <ol><li>Yellow LED illuminated when all conditions were satisfied.</li><li>Yellow LED remained off when any condition was missing.</li><li>Ignition was not enabled when no conditions were satisfied.</li></ol> |
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
