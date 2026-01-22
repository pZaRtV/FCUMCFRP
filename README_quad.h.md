# quad.h - FCU Configuration Header

Comprehensive configuration header for the FCU Madgwick Control Filter Research Platform. This file contains all user-configurable parameters for the flight control system.

## Overview

`quad.h` is the central configuration file that defines:
- **Receiver Type Selection**: PWM, PPM, SBUS, DSM, IBUS, ELRS support
- **IMU Configuration**: Primary and secondary IMU selection
- **Monitor System**: UDP telemetry and validation setup
- **Sensor Ranges**: Gyroscope and accelerometer full-scale settings
- **Channel Mapping**: Logical to physical channel assignments
- **Pin Definitions**: Hardware pin assignments
- **Safety Checks**: Compile-time configuration validation

## Key Configuration Sections

### 1. Receiver Type Selection

```cpp
// Select ONE receiver type:
#define USE_ELRS_RX          // ExpressLRS Radio System
// #define USE_PWM_RX        // Standard PWM Receiver
// #define USE_PPM_RX        // PPM Receiver
// #define USE_SBUS_RX       // SBUS Receiver
// #define USE_DSM_RX        // DSM Receiver
// #define USE_IBUS_RX       // IBUS Receiver
```

**Supported Receivers:**
- **ELRS**: ExpressLTS Radio System (420kHz CRSF protocol)
- **PWM**: Standard PWM receiver (6 channels)
- **PPM**: Pulse Position Modulation (single wire)
- **SBUS**: Futaba SBUS protocol (inverted serial)
- **DSM**: Spektrum DSM protocol
- **IBUS**: FlySky IBUS protocol

### 2. IMU Configuration

```cpp
// Primary IMU (used by control loop)
#define USE_MPU6050_I2C        // Default control IMU
// #define USE_MPU9250_SPI    // Alternative primary IMU

// Secondary IMU (monitor/validation)
#define USE_MPU9250_MONITOR_I2C  // Ground-truth validation IMU
```

**IMU Configuration Details:**
- **Primary IMU**: Used for flight control loop and attitude estimation
- **Monitor IMU**: Independent validation system (does NOT affect control)
- **Dual IMU Setup**: MPU6050 (control) + MPU9250 (monitor) recommended
- **Independent Operation**: Monitor IMU runs on separate I2C bus (Wire1)

### 3. UDP Telemetry Configuration

```cpp
// Ethernet/W5500 Configuration
#define W5500_CS_PIN 10              // Chip select pin
#define W5500_SPI_BUS SPI            // SPI bus selection
#define UDP_LOCAL_PORT 8888          // Local port (receiving)
#define UDP_REMOTE_PORT 8889         // Remote port (sending)
#define UDP_REMOTE_IP_0 192          // Remote IP address
#define UDP_REMOTE_IP_1 168
#define UDP_REMOTE_IP_2 1
#define UDP_REMOTE_IP_3 100
#define MONITOR_UDP_RATE_MS 10       // 100Hz telemetry rate
```

**UDP Telemetry Features:**
- **Real-time Telemetry**: 100Hz data transmission
- **Event Detection**: Command and disturbance monitoring
- **Data Validation**: Control vs. Monitor IMU comparison
- **Research Grade**: Comprehensive flight data logging

### 4. Sensor Range Configuration

```cpp
// Gyroscope Full Scale Range (deg/sec)
#define GYRO_250DPS                 // ±250°/s (default)
// #define GYRO_500DPS             // ±500°/s
// #define GYRO_1000DPS            // ±1000°/s
// #define GYRO_2000DPS            // ±2000°/s

// Accelerometer Full Scale Range (G)
#define ACCEL_2G                    // ±2G (default)
// #define ACCEL_4G                // ±4G
// #define ACCEL_8G                // ±8G
// #define ACCEL_16G               // ±16G
```

### 5. Channel Mapping

#### PPM Channel Mapping
```cpp
#define PPM_MAP_CH1 3   // throttle
#define PPM_MAP_CH2 1   // aileron
#define PPM_MAP_CH3 2   // elevator
#define PPM_MAP_CH4 4   // rudder
#define PPM_MAP_CH5 5   // gear / throttle cut
#define PPM_MAP_CH6 6   // aux1 / transition
```

#### ELRS Channel Mapping
```cpp
#define ELRS_MAP_CH1 1  // throttle
#define ELRS_MAP_CH2 2  // aileron
#define ELRS_MAP_CH3 3  // elevator
#define ELRS_MAP_CH4 4  // rudder
#define ELRS_MAP_CH5 5  // gear / throttle cut
#define ELRS_MAP_CH6 6  // aux1 / transition
```

#### SBUS Channel Mapping
```cpp
#define SBUS_MAP_CH1 1  // throttle (sbusChannels[0])
#define SBUS_MAP_CH2 2  // aileron (sbusChannels[1])
#define SBUS_MAP_CH3 3  // elevator (sbusChannels[2])
#define SBUS_MAP_CH4 4  // rudder (sbusChannels[3])
#define SBUS_MAP_CH5 5  // gear / throttle cut (sbusChannels[4])
#define SBUS_MAP_CH6 6  // aux1 / transition (sbusChannels[5])
```

### 6. Pin Definitions

```cpp
// PWM/PPM Receiver Pins
const int ch1Pin = 15;  // throttle
const int ch2Pin = 16;  // aileron
const int ch3Pin = 17;  // elevator
const int ch4Pin = 20;  // rudder
const int ch5Pin = 21;  // gear / throttle cut
const int ch6Pin = 22;  // aux1

// PPM Input Pin
const int PPM_Pin = 23;

// I2C Bus Assignment
// Wire:   MPU6050 (SDA=18, SCL=19) - 1000kHz
// Wire1:  MPU9250 (SDA1=16, SCL1=17) - 400kHz

// SPI Bus Assignment (W5500 Ethernet)
// SPI:   SCK=13, MISO=12, MOSI=11, CS=10
```

## Advanced Configuration

### Monitor System Configuration

```cpp
// Monitor comparison mode
#define USE_MONITOR_ATTITUDE_COMPARISON  // Compare attitude angles (recommended)
// If undefined: Compare raw sensor data only (calibration mode)
```

**Monitor System Features:**
- **Independent Madgwick Filter**: Separate attitude estimation
- **Real-time Validation**: Control vs. Monitor IMU comparison
- **UDP Telemetry**: 100Hz data transmission to ground station
- **Event Detection**: Command and disturbance monitoring
- **Research Grade**: Comprehensive flight data analysis

### ELRS Specific Configuration

```cpp
#define ELRS_UART_PORT 3           // Serial3 (RX3=pin 15, TX3=pin 14)
#define ELRS_BAUD_RATE 420000      // CRSF baud rate
#define ELRS_TIMEOUT_MS 100        // Failsafe timeout
#define ELRS_CHANNELS 6            // Number of active channels
```

### PWM Passthrough Configuration

```cpp
#define USE_PWM_PASSTHROUGH_CH1    // Direct PWM throttle input
// Provides faster throttle response compared to PPM
```

## Safety and Validation

### Compile-Time Safety Checks

The header includes comprehensive sanity checks to ensure proper configuration:

```cpp
// Receiver validation (exactly one must be defined)
#if (defined(USE_PWM_RX) + defined(USE_PPM_RX) + ...) != 1
#error "You must define exactly one receiver type"
#endif

// IMU validation (exactly one primary IMU)
#if (defined(USE_MPU6050_I2C) + defined(USE_MPU9250_SPI)) != 1
#error "You must define exactly one IMU"
#endif

// Sensor range validation (exactly one per type)
#if (defined(GYRO_250DPS) + defined(GYRO_500DPS) + ...) != 1
#error "You must define exactly one gyro range"
#endif
```

## Hardware Requirements

### Required Hardware
- **Teensy 4.0/4.1**: Flight controller board
- **MPU6050**: Primary IMU (I2C)
- **MPU9250**: Monitor IMU (I2C) - optional but recommended
- **W5500 Ethernet Shield**: UDP telemetry
- **Receiver**: ELRS/SBUS/DSM/PPM/PWM compatible

### Pin Connections
```
Teensy 4.0 Pin Assignment:
├── I2C Bus 0 (Wire): MPU6050
│   ├── SDA: Pin 18
│   └── SCL: Pin 19
├── I2C Bus 1 (Wire1): MPU9250
│   ├── SDA1: Pin 16
│   └── SCL1: Pin 17
├── SPI Bus: W5500 Ethernet
│   ├── SCK: Pin 13
│   ├── MISO: Pin 12
│   ├── MOSI: Pin 11
│   └── CS: Pin 10
├── PWM/PPM Inputs
│   ├── CH1: Pin 15 (throttle)
│   ├── CH2: Pin 16 (aileron)
│   ├── CH3: Pin 17 (elevator)
│   ├── CH4: Pin 20 (rudder)
│   ├── CH5: Pin 21 (gear)
│   ├── CH6: Pin 22 (aux1)
│   └── PPM: Pin 23
└── Serial3: ELRS Receiver
    ├── RX3: Pin 15
    └── TX3: Pin 14
```

## Configuration Examples

### Basic Flight Setup
```cpp
#define USE_ELRS_RX              // Modern radio system
#define USE_MPU6050_I2C          // Standard IMU
#define USE_MPU9250_MONITOR_I2C  // Enable monitoring
#define GYRO_500DPS              // Moderate gyro range
#define ACCEL_4G                 // Moderate accel range
```

### Research Platform Setup
```cpp
#define USE_SBUS_RX              // Professional radio
#define USE_MPU6050_I2C          // Control IMU
#define USE_MPU9250_MONITOR_I2C  // Validation IMU
#define USE_MONITOR_ATTITUDE_COMPARISON  // Full validation
#define GYRO_1000DPS             // High-performance gyro
#define ACCEL_8G                 // High-performance accel
```

### Minimal Setup
```cpp
#define USE_PWM_RX               // Simple PWM receiver
#define USE_MPU6050_I2C          // Basic IMU only
// #define USE_MPU9250_MONITOR_I2C  // No monitoring
#define GYRO_250DPS              // Standard gyro range
#define ACCEL_2G                 // Standard accel range
```

## Troubleshooting

### Common Configuration Issues

1. **Multiple Receivers Defined**: Ensure only one receiver type is defined
2. **No IMU Selected**: Must define exactly one primary IMU
3. **Pin Conflicts**: Check for pin assignment conflicts
4. **I2C Bus Conflicts**: Wire and Wire1 use different pins
5. **SPI Conflicts**: Ensure W5500 CS pin doesn't conflict

### Network Configuration

1. **IP Address**: Set UDP_REMOTE_IP_* to match ground station
2. **Port Conflicts**: Ensure UDP ports don't conflict with other services
3. **MAC Address**: Change if multiple devices on same network
4. **Ethernet Shield**: Verify W5500 connections

### IMU Configuration

1. **I2C Addresses**: MPU6050 (0x68), MPU9250 (0x69)
2. **Bus Speeds**: Wire (1000kHz), Wire1 (400kHz)
3. **Sensor Ranges**: Match expected flight dynamics
4. **Monitor System**: Requires both IMUs for full functionality

## Version Information

- **File Version**: Beta 1.1
- **Last Updated**: 2025-01-10
- **Author**: Patrick Andrasena T.
- **Base Project**: dRehmFlight by Nicholas Rehm

## Related Files

- `FCUMCFRP_B_1.4.5.ino`: Main flight control code
- `imuMonitor.ino`: Monitor IMU and UDP telemetry
- `UDPClient.py`: Ground station data receiver
- `FlightlogAnalysis.py`: Post-flight analysis tool

## Notes

- This configuration header is included by all main sketch files
- Changes require recompilation of the entire project
- Monitor system requires both IMUs for full functionality
- UDP telemetry requires network connectivity to ground station
- Safety checks prevent invalid configurations at compile time
