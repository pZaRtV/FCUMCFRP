// DRehmFlight Teensy Flight Controller Quad Specific Config
// Author         : Patrick Andrasena T.
// Project Start  : 8/21/2025
// Last Updated   : 4/14/2026
// Version        : Beta 1.2

#ifndef FC_QUAD_H
#define FC_QUAD_H

#include <SPI.h>

//USER CONFIGURATION//
// Select ONE receiver type:
// #define USE_PWM_RX
// #define USE_PPM_RX
// #define USE_HYBRID_RX
// #define USE_SBUS_RX
// #define USE_DSM_RX
#define USE_IBUS_RX
// #define USE_ELRS_RX

// If using DSM RX, set the number of channels:
// #define NUM_DSM_CHANNELS 6

// Select ONE primary IMU (used by control loop):
#define USE_MPU6050_I2C
// #define USE_MPU9250_SPI

// #define USE_MPU9250_MONITOR_I2C

// Ethernet/W5500 Configuration for UDP telemetry
#define W5500_CS_PIN 10
#define W5500_SPI_BUS SPI
#define UDP_LOCAL_PORT 8888
#define UDP_REMOTE_PORT 8889
#define UDP_REMOTE_IP_0 192
#define UDP_REMOTE_IP_1 168
#define UDP_REMOTE_IP_2 1
#define UDP_REMOTE_IP_3 100
#define MONITOR_UDP_RATE_MS 10

#define USE_MONITOR_ATTITUDE_COMPARISON

// Select ONE full scale gyro range (deg/sec):
#define GYRO_250DPS
// #define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS

// Select ONE full scale accelerometer range (G's):
#define ACCEL_2G
// #define ACCEL_4G
// #define ACCEL_8G
// #define ACCEL_16G

// #define USE_ONESHOT125_ESC

// PPM channel mapping (logical -> physical PPM slot, 1-based)
// CH1=Throttle, CH2=Aileron, CH3=Elevator, CH4=Rudder, CH5=Gear, CH6=Aux1
#define PPM_MAP_CH1 1
#define PPM_MAP_CH2 2
#define PPM_MAP_CH3 3
#define PPM_MAP_CH4 4
#define PPM_MAP_CH5 5
#define PPM_MAP_CH6 6

// PPM sync/channel timing thresholds (µs) — override here if your TX differs
// Standard PPM: sync gap >3000 µs, channel pulses 1000-2000 µs
// PPM_SYNC_MIN_US: minimum gap duration to be recognised as end-of-frame
// Reduce to ~2200 if your TX has a short sync gap; increase if you see false syncs
#define PPM_SYNC_MIN_US 2500UL
#define PPM_CH_MIN_US   700UL
#define PPM_CH_MAX_US   2300UL

// PWM channel mapping
#define PWM_MAP_CH1 1
#define PWM_MAP_CH2 2
#define PWM_MAP_CH3 3
#define PWM_MAP_CH4 4
#define PWM_MAP_CH5 5
#define PWM_MAP_CH6 6

// ELRS Channel Mapping
#define ELRS_MAP_CH1 1
#define ELRS_MAP_CH2 2
#define ELRS_MAP_CH3 3
#define ELRS_MAP_CH4 4
#define ELRS_MAP_CH5 5
#define ELRS_MAP_CH6 6

// SBUS Channel Mapping
#define SBUS_MAP_CH1 1
#define SBUS_MAP_CH2 2
#define SBUS_MAP_CH3 3
#define SBUS_MAP_CH4 4
#define SBUS_MAP_CH5 5
#define SBUS_MAP_CH6 6

// DSM Channel Mapping
#define DSM_MAP_CH1 1
#define DSM_MAP_CH2 2
#define DSM_MAP_CH3 3
#define DSM_MAP_CH4 4
#define DSM_MAP_CH5 5
#define DSM_MAP_CH6 6

// ELRS Specific Configuration
#define ELRS_UART_PORT 3
#define ELRS_BAUD_RATE 420000
#define ELRS_TIMEOUT_MS 100
#define ELRS_CHANNELS 6

  // i-BUS Configuration
  // FS-iA6B dedicated i-BUS port → Teensy Serial3 RX (pin 15)
// Serial3 is free — not used by motors, servos,IMU, or Ethernet
#define IBUS_SERIAL       Serial3
#define IBUS_UART_PIN     15    // Serial3 RX pin on Teensy 4.0

// i-BUS channel mapping — remapped to match FS-i6 Mode 2 output
// FS-i6 physical output: CH1=Roll, CH2=Pitch, CH3=Throttle, CH4=Yaw
// FCU logical assignment: CH1=Throttle, CH2=Roll, CH3=Pitch, CH4=Yaw
#define IBUS_MAP_CH1      3    // throttle ← FS-i6 sends throttle on CH3
#define IBUS_MAP_CH2      1    // aileron  ← FS-i6 sends roll on CH1
#define IBUS_MAP_CH3      2    // elevator ← FS-i6 sends pitch on CH2
#define IBUS_MAP_CH4      4    // rudder   ← yaw already correct
#define IBUS_MAP_CH5      5    // gear     ← already correct
#define IBUS_MAP_CH6      6    // aux1     ← already correct


// FIX 10: USE_PWM_PASSTHROUGH_CH1 has been REMOVED.
//
// When USE_PPM_RX is active, defining USE_PWM_PASSTHROUGH_CH1 causes getCh1()
// to be compiled (in the old radioComm.ino guard:
//   #if defined USE_PWM_RX || defined USE_PWM_PASSTHROUGH_CH1).
// getCh1() writes to channel_1_raw from ch1Pin (pin 15), which conflicts with
// the PPM ISR also writing channel_1_raw from PPM_Pin (pin 23).
// Result: channel_1_raw gets corrupted every time a PWM edge fires on pin 15,
// which on a floating pin is constantly — keeping channel_1_raw near 0 and
// triggering failSafe() every frame.
//
// If you genuinely need PWM passthrough for throttle alongside PPM for the
// other channels, re-enable USE_HYBRID_RX instead and implement dedicated
// handling that does not share channel_N_raw with the PPM path.
//
// #define USE_PWM_PASSTHROUGH_CH1   <-- KEEP THIS COMMENTED when using PPM

// Pin definitions
#if defined USE_PWM_RX || defined USE_PPM_RX
  const int ch1Pin = 15;
  const int ch2Pin = 16;
  const int ch3Pin = 17;
  const int ch4Pin = 20;
  const int ch5Pin = 21;
  const int ch6Pin = 22;
#endif

#if defined USE_PPM_RX || defined USE_HYBRID_RX
  const int PPM_Pin = 23;
#endif

//END USER CONFIGURATION//

// ========== Sanity checks ========= //
#if (defined(USE_PWM_RX) + defined(USE_PPM_RX) + defined(USE_SBUS_RX) + defined(USE_DSM_RX) + defined(USE_IBUS_RX) + defined(USE_HYBRID_RX) + defined(USE_ELRS_RX)) != 1
#error "You must define exactly one receiver type in quad.h."
#endif

#if (defined(USE_MPU6050_I2C) + defined(USE_MPU9250_SPI)) != 1
#error "You must define exactly one IMU in quad.h."
#endif

#if (defined(GYRO_250DPS) + defined(GYRO_500DPS) + defined(GYRO_1000DPS) + defined(GYRO_2000DPS)) != 1
#error "You must define exactly one gyro range in quad.h."
#endif

#if (defined(ACCEL_2G) + defined(ACCEL_4G) + defined(ACCEL_8G) + defined(ACCEL_16G)) != 1
#error "You must define exactly one accelerometer range in quad.h."
#endif

// Safety: catch USE_PWM_PASSTHROUGH_CH1 being re-enabled alongside USE_PPM_RX
#if defined(USE_PPM_RX) && defined(USE_PWM_PASSTHROUGH_CH1)
#error "USE_PWM_PASSTHROUGH_CH1 must not be defined when USE_PPM_RX is active. See quad.h FIX 10 comment."
#endif

#endif // FC_QUAD_H
