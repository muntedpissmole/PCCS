# Changelog
All notable changes to the PCCS project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.4.0-beta 1] - 18-06-2025

### Added & Changed
- Shutdown and setting low/medium/high brightness on remote display
- Turning remote screen off and on based on state of kitchen panel reed switch (where the touchscreen lives, no point in having it on if the panel is shut)
- Improved handling of missing PCA9685 board by disabling lighting faders and scenes, displaying error text on percentage level
- Made temperature and battery voltage display the same error text and water tank level
- Moved version to Help/About settings tab, also shows your current coordinates
- Changed brightness level from slider to low/medium/high buttons to work in with limitations of Waveshare touchscreen brightness commands
- Implement overscroll for lighting pages
- Imrpovement of lighting slider thumbs
- Somehow gave the lighting widgets even more delicious neumorphism

### Fixed
- Scene active styles weren't showing when a scene was active due to relay_states function being accidentally deleted from app.py
- You can no longer select text when swiping left and right

---