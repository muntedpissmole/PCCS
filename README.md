# The Pissmole Camping Control System (PCCS)

## Overview
The Pissmole Camping Control System (PCCS) is a Raspberry Pi-based control system designed to manage camper trailer electronics. It controls dimmable lighting and relays, lighting scenes that can be executed on time of sunset/sunrise offset or fixed times, general purpose relays, GPS for sunset/sunrise/date/time and coordinates calculations, reed switch monitoring for panel doors and drawers and triggering of lighting channels or scenes. It also displys environmental data such as temperature, battery voltage, current time, sunset/sunrise times and water tank level with a user-friendly interface featuring a neomorphism theme with dark and light options.

## Installation
This project has been developed for Raspberry Pi 5 Bookworm. It runs on Flask in a venv.
Confirm Flask and venv modules are installed:
```
sudo apt install python3 python3-venv python3-pip -y
```

Create virtual environment called venv:
```
python3 -m venv venv
```

Activate virtual environment and install Flask:
```
source venv/bin/activate
pip install flask
```

Copy files to root folder.
Install requirements:
```
pip install -r requirements.txt
```

Run app.py:
```
python3 app.py
```
Or install as a service to start with the RPI.

## Hardware requirements:
- RPI 5
- Adafruit PCA9685 on I2C channel 40 and IRF540 mosfet driver boards to run your LEDs
- Adafruit Ultimate GPS Breakout PA1616S
- 2 x ADS1115 4 channel I2C analog to digital converter at addresses 48 and 49. I used solderable breadboard with jumpers
- 4 channel 5VDC relay module
- DS18B20 Temperature Sensor
- fuel level sensor that scales from 240ohm (full) to 33ohm (empty)
- DC0-25V Voltage Detection Module Voltage Sensor

### Connections
See config.json for lighting, relay and reed switch connections.
- Battery level sensor: ADS1115 address 49 channel A0.
- Water sensor: ADS1115 address 49 channel A1.
- Temperature Sensor: Pin 7 on the RPI.

## Development Path

### Alpha 1
- Initial backend logic for system control.
- Design of user interface.
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

### Alpha 3
- Implementation of hard coded scenes.
- Connection to mosfet dimmer to confirm and refine lighting dimmer operation.
- Implementation of anti-bug/red lighting logic for green channels on some lighting channels.
- Installation of remaining hardware reed switches, battery voltage etc) to confirm real world functionality.
- Completion of shutdown button to cleanly shutdown RPI before cutting power.

### Beta 1 (In Development)
- Implementation of scene editor.
- Bug fixes, graphical and functionality refinement.

### Beta 2
- Implementation of event scheduler.
- Field testing.

### Release v2.0
- Initial release. V2.0 is the spiritual successor to v1, which was an arduino based solution inside a 12v distribution hub.