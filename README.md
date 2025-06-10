# The Pissmole Camping Control System (PCCS)

## Overview
The Pissmole Camping Control System (PCCS) is a Raspberry Pi-based solution designed to manage camper trailer electronics. It controls PWM lighting, lighting scenes based on time of sunset/sunrise offset, general purpose relays, GPS for sunset/sunrise/date/time and coordinates calculations, reed switch monitoring for panel doors and drawers and triggering of lighting scenes, temperature, battery voltage and water tank level monitoring with a user-friendly interface featuring a neomorphism theme with dark and light options.

## Changelog

### Alpha 1
- Initial backend logic for system control.
- Designed user interface with a neomorphism theme.
- Positioned data blocks and navigation elements.
- Added PCA9685 board, mapping lighting dimmer channels to outputs.
- Created data dictionary.

### Alpha 2
- connection of temp sensor, GPS, voltage sensor and tank level sensor.
- Refinement of GPS data display (use locally cached data in absence of GPS fix).
- Refinement of tank level sensor function (display error text if sensor is missing/faulty).
- Refinement of neomorphism theme and pagination of lighting controls.
- Refinement of lighting relay states on startup.
- Implementation of placeholder settings page.

### Alpha 3 (In development)
- Implementation of event scheduler, scene editor and scenes.
- Connection to mosfet dimmer to confirm and refine lighting dimmer operation.
- Implementation of anti-bug/red lighting logic for green channels on some lighting channels.
- Installation of remaining hardware reed switches, battery voltage etc) to confirm real world functionality.
- Completion of shutdown button to cleanly shutdown RPI before cutting power.