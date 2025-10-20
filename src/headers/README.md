# Header File Organization

This document describes the new header file structure for the mower7.0 project.

## Directory Structure

```
src/
├── headers/
│   ├── config/
│   │   └── config.h              # Centralized configuration and constants
│   ├── sensors/
│   │   ├── adxl.h                # ADXL343 accelerometer interface
│   │   ├── bmp390.h              # BMP390/BMP580 barometric sensor interface
│   │   └── imu.h                 # BNO055 IMU sensor interface
│   ├── math/
│   │   ├── KalmanVelocityEstimator.h  # Kalman filter implementation
│   │   ├── Quaternion.h          # Quaternion math class
│   ├── utils/
│   │   ├── RollingAverage.h      # Rolling average utility class
│   │   ├── sdcard.h              # SD card operations
│   │   └── servos.h              # Servo control interface
│   ├── adxl.cpp
│   ├── bmp390.cpp
│   ├── ... (implementation files)
│
├── main.cpp
└── other source files...
```

## Config File (config.h)

All hardware pins, constants, and configuration parameters are now centralized in `headers/config/config.h`. This file includes:

- **Hardware Pin Configuration**: All GPIO pins, I2C addresses, and SPI configurations
- **Physical Constants**: GRAVITY = 9.81 m/s²
- **Flight Parameters**: Apogee targets, flight times, parachute terminal velocity
- **Servo Angles**: All servo deployment angles
- **Data Logging**: Settings for SD card logging precision

## Header Organization by Category

### Sensors (`headers/sensors/`)
- **adxl.h**: ADXL343 accelerometer
- **bmp390.h**: BMP390 and BMP580 barometric sensors
- **imu.h**: BNO055 IMU sensor

All sensor headers include `config.h` automatically.

### Math (`headers/math/`)
- **Quaternion.h**: Quaternion class for orientation calculations
- **KalmanVelocityEstimator.h**: Kalman filter for altitude/velocity estimation

### Utilities (`headers/utils/`)
- **RollingAverage.h**: Rolling average class for sensor smoothing
- **sdcard.h**: SD card file operations
- **servos.h**: Servo control and deployment functions

## Include Pattern

All headers now follow a consistent pattern:

```cpp
#include "../config/config.h"  // Always include config first
#include <library_includes>    // External libraries
#include "../relative/path/to/header.h"  // Internal headers
```

## Benefits

1. **Centralized Configuration**: All pins and constants in one place
2. **Organized Structure**: Headers grouped by functionality
3. **Easy Maintenance**: Clear separation of concerns
4. **No Redundancy**: Removes duplicate #defines across files
5. **Consistent Includes**: All headers automatically get access to global config
