# Pissmole Camping Control System (PCCS)

## Overview
The Pissmole Camping Control System (PCCS) is a Raspberry Pi-based solution designed to manage camper trailer electronics. It controls PWM lighting, lighting scenes based on time of sunsset/sunrise offset, general purpose relay control, GPS for sunset/sunrise/date/time and coordinates calculations, reed switch monitoring for panel doors and drawers, temperature, battery voltage and water tank level monitoring with a user-friendly interface featuring a neomorphism theme with dark and light options.

## Changelog

### Alpha 1
- Developed initial backend logic for system control.
- Designed user interface with a neomorphism theme, including layout and styling.
- Positioned data blocks and navigation elements for intuitive access.
- Integrated PCA board hardware, mapping lighting dimmer channels to specific outputs.
- Created a data dictionary to standardize data handling.

### Alpha 2
- Integrated temperature sensor, GPS module, voltage sensor, and tank level sensor.
- Enhanced GPS data display to use locally cached coordinates when GPS lock is unavailable.
- Improved tank level sensor logic to suppress display if the sensor is missing or faulty.
- Refined the neomorphism theme with better visual hierarchy and added pagination for lighting controls.
- Stabilized lighting relay states on system startup to prevent erratic behavior.
- Added a placeholder settings page for future configuration options.

### Alpha 3 (In development)
- Implementation of an event scheduler and scene editor for automated control sequences.
- Enable scene management for preset lighting and sensor configurations.
- Connect MOSFET dimmer to validate and optimize lighting dimmer performance.
- Add anti-bug/red lighting logic for green channels on select lighting outputs.
- Install remaining hardware (reed switches, battery voltage monitoring) for real-world testing.
- Add a shutdown button to safely power off the Raspberry Pi before disconnecting power.