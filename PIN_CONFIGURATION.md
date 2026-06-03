# FCUMCFRP_B_1.4.5 - Pin Configuration and Sanity Check

**Project**: FCU Madgwick Control Filter Research Platform  
**Version**: B-1.4.5  
**Target Platform**: Teensy 4.0/4.1  
**Last Updated**: 2026-02-01  

---

## 📋 Overview

This document provides a comprehensive pin configuration reference and sanity check for the FCUMCFRP_B_1.4.5 flight control project. It includes all required pin connections, their purposes, and configuration notes for proper setup.

---

## 🔌 Pin Assignment Summary

### **Motor Control Pins**
| Pin | Function | ESC Type | PWM Range | Notes |
|-----|----------|----------|-----------|-------|
| 0 | Motor 1 (Front Left) | OneShot125/PWM | 125-250μs / 1000-2000μs | CCW rotation |
| 1 | Motor 2 (Front Right) | OneShot125/PWM | 125-250μs / 1000-2000μs | CW rotation |
| 2 | Motor 3 (Back Right) | OneShot125/PWM | 125-250μs / 1000-2000μs | CCW rotation |
| 3 | Motor 4 (Back Left) | OneShot125/PWM | 125-250μs / 1000-2000μs | CW rotation |
| 4 | Motor 5 (Auxiliary) | OneShot125/PWM | 125-250μs / 1000-2000μs | Currently unused |
| 5 | Motor 6 (Auxiliary) | OneShot125/PWM | 125-250μs / 1000-2000μs | Currently unused |

### **Servo Control Pins**
| Pin | Function | PWM Range | Notes |
|-----|----------|-----------|-------|
| 6 | Servo 1 | 900-2100μs | Available for auxiliary control |
| 7 | Servo 2 | 900-2100μs | Available for auxiliary control |
| 8 | Servo 3 | 900-2100μs | Available for auxiliary control |
| 9 | Servo 4 | 900-2100μs | Available for auxiliary control |

### **Radio Receiver Pins**
| Pin | Function | Receiver Type | Notes |
|-----|----------|---------------|-------|
| 15 | Channel 1 (Throttle) | PWM/PPM/DSM/ELRS | PWM passthrough available |
| 16 | Channel 2 (Aileron/Roll) | PWM/PPM | Also Wire1 SDA1 for monitor IMU |
| 17 | Channel 3 (Elevator/Pitch) | PWM/PPM | Also Wire1 SCL1 for monitor IMU |
| 20 | Channel 4 (Rudder/Yaw) | PWM/PPM | |
| 21 | Channel 5 (Gear/Arm) | PWM/PPM/SBUS | SBUS uses RX5 |
| 22 | Channel 6 (Aux1) | PWM/PPM | Free auxiliary channel |
| 23 | PPM Signal | PPM Only | Single wire PPM input |

### **I2C Communication Pins**
| Pin | Function | I2C Bus | Device | Clock Speed | Notes |
|-----|----------|---------|--------|-------------|-------|
| 18 | SDA | Wire (I2C0) | MPU6050 (Primary) | 1MHz | Default control IMU |
| 19 | SCL | Wire (I2C0) | MPU6050 (Primary) | 1MHz | Default control IMU |
| 16 | SDA1 | Wire1 (I2C1) | MPU9250 (Monitor) | 400kHz | Monitor IMU (shared with CH2) |
| 17 | SCL1 | Wire1 (I2C1) | MPU9250 (Monitor) | 400kHz | Monitor IMU (shared with CH3) |

### **SPI Communication Pins**
| Pin | Function | SPI Bus | Device | Notes |
|-----|----------|---------|--------|-------|
| 10 | CS (Chip Select) | SPI | W5500 Ethernet | UDP telemetry |
| 11 | MOSI | SPI | W5500 Ethernet | |
| 12 | MISO | SPI | W5500 Ethernet | |
| 13 | SCK | SPI | W5500 Ethernet | Also onboard LED |

### **Serial Communication Pins**
| Pin | Function | UART | Device | Baud Rate | Notes |
|-----|----------|------|--------|-----------|-------|
| 14 | TX3 | Serial3 | ELRS/DSM | 420000/115200 | ELRS or DSM transmitter |
| 15 | RX3 | Serial3 | ELRS/DSM | 420000/115200 | ELRS or DSM transmitter |
| 21 | RX5 | Serial5 | SBUS | 100000 | SBUS receiver |

### **Special Purpose Pins**
| Pin | Function | Notes |
|-----|----------|-------|
| 13 | Onboard LED | Built-in LED, used for status indication |
| 36 | SPI2 CS | MPU9250 SPI (if used as primary) |

---

## 🔧 Configuration Details

### **Current Configuration (from quad.h)**
```cpp
// Receiver Type
#define USE_ELRS_RX

// Primary IMU
#define USE_MPU6050_I2C

// Monitor IMU
#define USE_MPU9250_MONITOR_I2C

// Ethernet UDP Telemetry
#define W5500_CS_PIN 10
#define UDP_REMOTE_PORT 8889
```

### **Motor Configuration**
```cpp
// ESC Protocol Selection
// #define USE_ONESHOT125_ESC  // Comment out for standard PWM

// Motor Pin Assignments
const int m1Pin = 0;  // Front Left
const int m2Pin = 1;  // Front Right  
const int m3Pin = 2;  // Back Right
const int m4Pin = 3;  // Back Left
const int m5Pin = 4;  // Auxiliary
const int m6Pin = 5;  // Auxiliary
```

---

## ✅ Sanity Check Checklist

### **Pre-Flight Hardware Checks**

#### **[ ] Motor Connections**
- [ ] Motor 1 (Front Left) connected to Pin 0
- [ ] Motor 2 (Front Right) connected to Pin 1
- [ ] Motor 3 (Back Right) connected to Pin 2
- [ ] Motor 4 (Back Left) connected to Pin 3
- [ ] ESC power connections secure
- [ ] ESC calibration completed
- [ ] Propeller direction verified (CCW/CW as per comments)

#### **[ ] Radio Receiver**
- [ ] ELRS receiver connected to RX3 (Pin 15) and TX3 (Pin 14)
- [ ] Receiver powered from 5V supply
- [ ] Binding completed with transmitter
- [ ] Channel mapping verified in quad.h

#### **[ ] IMU Sensors**
- [ ] MPU6050 (Primary) connected to I2C0:
  - [ ] SDA to Pin 18
  - [ ] SCL to Pin 19
  - [ ] 3.3V power
  - [ ] Ground connection
- [ ] MPU9250 (Monitor) connected to I2C1:
  - [ ] SDA1 to Pin 16 (shared with CH2)
  - [ ] SCL1 to Pin 17 (shared with CH3)
  - [ ] 3.3V power
  - [ ] Ground connection
  - [ ] I2C address set to 0x69

#### **[ ] Ethernet Telemetry**
- [ ] W5500 Ethernet module connected:
  - [ ] CS to Pin 10
  - [ ] MOSI to Pin 11
  - [ ] MISO to Pin 12
  - [ ] SCK to Pin 13
  - [ ] 3.3V power
  - [ ] Ground connection
- [ ] Ethernet cable connected to network
- [ ] UDP receiver running on ground station

#### **[ ] Power System**
- [ ] 5V power supply for receiver and servos
- [ ] 3.3V power for IMUs and logic
- [ ] Battery voltage monitoring (if implemented)
- [ ] Power distribution board secure
- [ ] All polarity connections verified

### **Software Configuration Checks**

#### **[ ] quad.h Configuration**
- [ ] Only ONE receiver type defined: `USE_ELRS_RX`
- [ ] Only ONE primary IMU defined: `USE_MPU6050_I2C`
- [ ] Only ONE gyro range defined: `GYRO_250DPS`
- [ ] Only ONE accel range defined: `ACCEL_2G`
- [ ] Monitor IMU enabled: `USE_MPU9250_MONITOR_I2C`
- [ ] UDP telemetry IP address correct
- [ ] Channel mapping matches transmitter

#### **[ ] Pin Conflict Resolution**
- [ ] Pin 16: Shared between CH2 and Wire1 SDA1 (acceptable)
- [ ] Pin 17: Shared between CH3 and Wire1 SCL1 (acceptable)
- [ ] Pin 15: Shared between CH1 and Serial3 RX3 (acceptable)
- [ ] Pin 13: Shared between SPI SCK and LED (acceptable)
- [ ] No other pin conflicts detected

---

## ⚠️ Important Notes

### **Pin Sharing Considerations**
1. **Pin 16 (CH2/SDA1)**: Can be used for PWM input OR I2C1, not both simultaneously
2. **Pin 17 (CH3/SCL1)**: Can be used for PWM input OR I2C1, not both simultaneously
3. **Pin 15 (CH1/RX3)**: Can be used for PWM input OR serial reception, not both simultaneously

### **I2C Bus Separation**
- **Wire (I2C0)**: 1MHz clock for MPU6050 primary control IMU
- **Wire1 (I2C1)**: 400kHz clock for MPU9250 monitor IMU
- Separate buses prevent interference and allow independent operation

### **SPI Bus Usage**
- Default SPI bus used for W5500 Ethernet module
- Pins 11, 12, 13 shared with potential other SPI devices
- Pin 10 dedicated as CS for W5500

### **UART Configuration**
- Serial3 used for ELRS/DSM receiver communication
- Serial5 used for SBUS receiver (if enabled)
- USB Serial used for debugging and configuration

---

## 🚨 Troubleshooting Guide

### **Common Issues**

#### **Motors Not Spinning**
1. Check arming sequence (CH5 low, CH1 low)
2. Verify ESC calibration
3. Check motor pin connections
4. Verify power supply to ESCs
5. Check `USE_ONESHOT125_ESC` configuration

#### **IMU Not Detected**
1. Check I2C connections (SDA/SCL)
2. Verify 3.3V power to IMU
3. Check I2C address configuration
4. Verify pull-up resistors on I2C lines
5. Check for bus conflicts

#### **Radio Not Working**
1. Verify receiver binding
2. Check serial connections for ELRS/DSM
3. Verify baud rate configuration
4. Check channel mapping in quad.h
5. Verify receiver power supply

#### **Ethernet Telemetry Not Working**
1. Check SPI connections to W5500
2. Verify CS pin (Pin 10) connection
3. Check IP address configuration
4. Verify network connectivity
5. Check UDP port configuration

---

## 📝 Configuration Examples

### **Minimal Setup (Quadcopter Only)**
```
Required Pins:
- Motors: 0, 1, 2, 3
- IMU: 18, 19 (MPU6050)
- Radio: Choose ONE option below
  • ELRS: 15, 14 (Serial3 RX/TX)
  • PPM: 23 (PPM signal)
  • SBUS: 21 (Serial5 RX)
  • PWM: 15, 16, 17, 20, 21, 22 (Channels 1-6)
- Power: 3.3V, 5V, GND
```

### **Full Research Setup**
```
Required Pins:
- Motors: 0, 1, 2, 3
- Servos: 6, 7, 8, 9 (optional)
- Primary IMU: 18, 19 (MPU6050, I2c)
- Monitor IMU: 16, 17 (MPU9250, I2c)
- Radio: Choose ONE option below
  • ELRS: 15, 14 (Serial3 RX/TX)
  • PPM: 23 (PPM signal)
  • SBUS: 21 (Serial5 RX)
  • PWM: 15, 16, 17, 20, 21, 22 (Channels 1-6)
- Ethernet:
  - 10: CS
  - 11: MOSI
  - 12: MISO
  - 13: SCK
- Power: 3.3V, 5V, GND
```

### **Radio Receiver Pin Details**
```
ELRS (ExpressLRS):
- Pin 15: RX3 (Serial receive)
- Pin 14: TX3 (Serial transmit)
- Baud Rate: 420000
- Protocol: CRSF

PPM (Pulse Position Modulation):
- Pin 23: PPM signal input
- All channels on single wire
- Frame rate: ~50Hz

SBUS (Serial Bus):
- Pin 21: RX5 (Serial receive)
- Baud Rate: 100000
- 16 channels available
- Inverted signal

PWM (Standard):
- Pin 15: Channel 1 (Throttle)
- Pin 16: Channel 2 (Aileron/Roll)
- Pin 17: Channel 3 (Elevator/Pitch)
- Pin 20: Channel 4 (Rudder/Yaw)
- Pin 21: Channel 5 (Gear/Arm)
- Pin 22: Channel 6 (Aux1)
- 50Hz update rate per channel
```

---

## 📚 Reference Documents

- [README_FCUMCFRP_B_1.4.5.ino.md](README_FCUMCFRP_B_1.4.5.ino.md) - Main flight controller documentation
- [README_imuMonitor.ino.md](README_imuMonitor.ino.md) - Monitor IMU documentation
- [quad.h](quad.h) - Configuration header file
- [Teensy 4.0 Pinout Chart](https://www.pjrc.com/teensy/teensy41.html) - Official pin reference

---

**⚡ Quick Reference: Essential Pins for Basic Flight**
- **Motors**: 0, 1, 2, 3
- **IMU**: 18 (SDA), 19 (SCL)
- **Radio**: 15 (RX3), 14 (TX3)
- **Power**: 3.3V, 5V, GND

---

*This document should be updated whenever pin configurations change. Always verify connections before powering on the system.*
