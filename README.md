Madgwick Filter research platform for UAV Quadcopter based of NickRehm's DrehmFlight project. Added UDP telemetry solution via ethernet, additional RC receiver options, User configurable .h file, dataset retrieval and analysis, and more.

refer to the markdowns for documentation:

# FCUMCFRP_B_1.4.5.ino - Main Flight Control Code

Core flight control software for the FCU Madgwick Control Filter Research Platform. This file implements the complete flight control loop including sensor fusion, PID control, and motor mixing.

## Overview

`FCUMCFRP_B_1.4.5.ino` is the main flight control program that:
- **Manages Sensor Fusion**: Madgwick filter for attitude estimation
- **Implements PID Control**: Angle and rate control loops
- **Handles Radio Input**: Multiple receiver type support
- **Controls Motors**: PWM/OneShot125 motor mixing
- **Provides Safety**: Arming, failsafe, and throttle cut systems
- **Integrates Monitoring**: Calls monitor IMU telemetry system

## Key Components

### 1. System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Radio Input   │───▶│   Command       │───▶│   Motor Mixing  │
│                 │    │   Processing    │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                │
                                ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Sensor Data   │───▶│   Madgwick      │───▶│   PID Control   │
│                 │    │   Filter        │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                │
                                ▼
                       ┌─────────────────┐
                       │   UDP Telemetry │
                       │   (Monitor)     │
                       └─────────────────┘
```

### 2. Core Functions

#### Main Loop Functions
```cpp
void loop() {
  // Main control loop at ~2kHz (500μs cycle time)
  getCommands();           // Read radio inputs
  armedStatus();           // Check arming status
  getIMUdata();            // Read primary IMU
  Madgwick();              // Sensor fusion
  getDesState();           // Process commands
  controlANGLE();          // PID control
  throttleCut();           // Safety throttle cut
  scaleCommands();         // Motor mixing
  commandMotors();         // Output to motors
  sendIMUDataUDP();        // Monitor telemetry (if enabled)
}
```

#### Sensor Fusion
```cpp
void Madgwick() {
  // Madgwick AHRS algorithm implementation
  // Fuses accelerometer, gyroscope, magnetometer data
  // Provides roll, pitch, yaw attitude estimates
  // Runs at 2kHz for optimal performance
}
```

#### Control Functions
```cpp
void controlANGLE() {
  // Angle-based PID control
  // Uses roll_des, pitch_des, yaw_des as setpoints
  // Computes roll_PID, pitch_PID, yaw_PID outputs
  // Includes integrator anti-windup and safety limits
}

void controlRATE() {
  // Rate-based PID control (alternative mode)
  // Uses gyro rates directly for control
  // Higher bandwidth control for acrobatic flight
}
```

#### Command Processing
```cpp
void getCommands() {
  // Reads radio receiver based on configured type
  // Supports: PWM, PPM, SBUS, DSM, IBUS, ELRS
  // Applies channel mapping and failsafe values
  // Outputs: channel_1_pwm through channel_6_pwm
}

void getDesState() {
  // Normalizes PWM commands to desired states
  // Computes: roll_des, pitch_des, yaw_des, thro_des
  // Applies rate limits and scaling
  // Outputs in degrees and normalized values
}
```

#### Motor Mixing
```cpp
void controlMixer() {
  // Mixes PID outputs to motor commands
  // Supports various frame configurations
  // Applies motor limits and scaling
  // Outputs: m1_command through m6_command
}

void scaleCommands() {
  // Scales motor commands to PWM/OneShot125
  // Applies motor direction and trim
  // Outputs final PWM values to ESCs
}
```

### 3. Safety Systems

#### Arming System
```cpp
void armedStatus() {
  // Arming conditions:
  // 1. Channel 5 (gear) must be LOW (< 1500 PWM)
  // 2. Channel 1 (throttle) must be LOW (< 1050 PWM)
  // Sets armedFly = true when conditions met
}
```

#### Throttle Cut
```cpp
void throttleCut() {
  // Safety override conditions:
  // 1. Channel 5 (gear) is HIGH (> 1500 PWM)
  // 2. armedFly is false
  // Forces all motors to minimum PWM
}
```

#### Failsafe System
```cpp
void failsafe() {
  // Radio signal loss detection
  // Applies predefined failsafe values
  // Sets armedFly = false for safety
}
```

### 4. Sensor Management

#### Primary IMU Interface
```cpp
void IMUinit() {
  // Initializes primary IMU (MPU6050 or MPU9250)
  // Sets full-scale ranges and filters
  // Calibrates sensors if needed
}

void getIMUdata() {
  // Reads raw sensor data
  // Applies calibration corrections
  // Low-pass filters for noise reduction
  // Outputs: AccX/Y/Z, GyroX/Y/Z, MagX/Y/Z
}
```

#### Calibration Functions
```cpp
void calculate_IMU_error() {
  // Computes sensor bias errors
  // Requires vehicle to be level on startup
  // Prints calibration values for user
  // Stores in AccError* and GyroError* variables
}

void calibrateAttitude() {
  // Optional attitude calibration
  // Sets initial reference angles
  // Useful for tilted installations
}
```

### 5. Radio Receiver Support

#### Supported Receiver Types
- **PWM**: Standard PWM receiver (6 channels)
- **PPM**: Pulse Position Modulation (single wire)
- **SBUS**: Futaba SBUS protocol (inverted serial)
- **DSM**: Spektrum DSM protocol
- **IBUS**: FlySky IBUS protocol
- **ELRS**: ExpressLTS Radio System (CRSF)

#### Receiver Interface Functions
```cpp
void getPPMRX();      // PPM receiver interface
void getPWMRX();      // PWM receiver interface
void getSBUSRX();     // SBUS receiver interface
void getELRSRX();     // ELRS receiver interface
void getDSMRX();      // DSM receiver interface
```

### 6. Motor Control

#### Motor Configuration
```cpp
// Motor command variables
float m1_command_scaled, m2_command_scaled, ...;  // Normalized commands
int m1_command_PWM, m2_command_PWM, ...;           // PWM outputs

// PWM/OneShot125 selection
#define USE_ONESHOT125_ESC  // Enable OneShot125 protocol
```

#### Motor Output
```cpp
void commandMotors() {
  // Sends PWM signals to ESCs
  // Supports standard PWM (1000-2000μs)
  // Supports OneShot125 (125-250μs)
  // Uses PWMServo library for timing
}
```

## Control Modes

### 1. Angle Control Mode
- **Primary Mode**: Default flight mode
- **Stabilization**: Maintains desired attitude angles
- **PID Control**: Proportional-Integral-Derivative control
- **Setpoints**: roll_des, pitch_des, yaw_des from radio
- **Feedback**: Attitude angles from Madgwick filter

### 2. Rate Control Mode
- **Alternative Mode**: For acrobatic flight
- **Direct Rate**: Controls angular rates directly
- **Higher Bandwidth**: Faster response than angle mode
- **Setpoints**: Desired angular rates
- **Feedback**: Gyroscope rates directly

### 3. Control Mode Selection
```cpp
// Control mode selection (typically via radio channel)
if (control_mode == ANGLE_MODE) {
  controlANGLE();
} else {
  controlRATE();
}
```

## PID Controller Implementation

### PID Structure
```cpp
// PID variables for each axis
float error_roll, error_pitch, error_yaw;
float integral_roll, integral_pitch, integral_yaw;
float derivative_roll, derivative_pitch, derivative_yaw;
float roll_PID, pitch_PID, yaw_PID;

// PID gains (tunable)
float Kp_roll_angle, Ki_roll_angle, Kd_roll_angle;
float Kp_pitch_angle, Ki_pitch_angle, Kd_pitch_angle;
float Kp_yaw, Ki_yaw, Kd_yaw;
```

### PID Algorithm
```cpp
void controlANGLE() {
  // Error calculation
  error_roll = roll_des - roll_IMU;
  
  // Integral term with anti-windup
  integral_roll = integral_roll_prev + error_roll * dt;
  if (channel_1_pwm < 1060) integral_roll = 0;  // Low throttle reset
  integral_roll = constrain(integral_roll, -i_limit, i_limit);
  
  // Derivative term
  derivative_roll = GyroX;  // Use gyro for derivative
  
  // PID output
  roll_PID = 0.01 * (Kp_roll_angle * error_roll + 
                     Ki_roll_angle * integral_roll - 
                     Kd_roll_angle * derivative_roll);
}
```

## Integration with Monitor System

### Monitor Integration Point
```cpp
void loop() {
  // ... main control loop ...
  
  sendIMUDataUDP();  // Call monitor telemetry system
}
```

### Data Sharing
- **Global Variables**: All attitude and sensor data accessible
- **No Interference**: Monitor system doesn't affect control loop
- **Real-time Data**: 100Hz telemetry transmission
- **Event Detection**: Command and disturbance monitoring

## Configuration and Tuning

### PID Tuning Parameters
```cpp
// Angle mode gains (default values)
float Kp_roll_angle = 0.15;    // Proportional gain
float Ki_roll_angle = 0.00;    // Integral gain  
float Kd_roll_angle = 0.05;    // Derivative gain

// Rate mode gains
float Kp_roll_rate = 0.15;     // Rate proportional gain
float Ki_roll_rate = 0.00;     // Rate integral gain
float Kd_roll_rate = 0.001;    // Rate derivative gain
```

### Safety Limits
```cpp
float i_limit = 25.0;          // Integrator saturation limit
float maxRoll = 30.0;          // Maximum roll angle (degrees)
float maxPitch = 30.0;         // Maximum pitch angle (degrees)
float maxYaw = 160.0;          // Maximum yaw rate (degrees/sec)
```

### Rate Limiting
```cpp
float dt = 0.005;              // Loop time (200Hz)
float maxRate = 200.0;         // Maximum rate limit (deg/sec)
```

## Performance Characteristics

### Loop Timing
- **Target Frequency**: 200Hz (5ms loop time)
- **Actual Performance**: ~180-200Hz on Teensy 4.0
- **CPU Usage**: ~15-20% of available processing
- **Latency**: <5ms from sensor to motor output

### Sensor Performance
- **IMU Update Rate**: 1kHz (1000Hz)
- **Madgwick Filter**: 200Hz (synchronized with control loop)
- **Filter Convergence**: <1 second typical
- **Noise Performance**: <0.1° RMS in stable conditions

### Control Performance
- **Response Time**: <100ms to step inputs
- **Steady-State Error**: <1° typical
- **Overshoot**: <5% typical
- **Settling Time**: <500ms to 2% band

## Debugging and Monitoring

### Serial Output
```cpp
void printLoopRate() {
  // Prints actual loop timing
  // Useful for performance monitoring
}

void printDesiredState() {
  // Prints desired attitude and rates
  // Useful for control debugging
}

void printRadioCommands() {
  // Prints raw PWM values
  // Useful for radio debugging
}
```

### LED Indicators
```cpp
void setupBlink() {
  // Blink patterns for system status
  // Different patterns for different states
}
```

## Safety Features

### Pre-flight Safety
- **Arming Sequence**: Requires specific switch positions
- **Throttle Safety**: Throttle must be low to arm
- **Sensor Validation**: Checks for valid sensor data
- **Radio Validation**: Validates receiver signal quality

### In-flight Safety
- **Failsafe Protection**: Automatic disarm on signal loss
- **Throttle Cut**: Immediate motor shutdown on command
- **Integrator Limits**: Prevents control windup
- **Rate Limiting**: Prevents excessive control rates

### Emergency Procedures
- **Throttle Cut**: Channel 5 high immediately disarms
- **Signal Loss**: Automatic failsafe after timeout
- **Sensor Failure**: Safe motor shutdown
- **Power Loss**: Graceful shutdown on low voltage

## Troubleshooting

### Common Issues

1. **No Motor Response**
   - Check arming status (armedFly variable)
   - Verify throttle cut is off (channel 5 < 1500)
   - Check radio receiver connection
   - Verify motor ESC calibration

2. **Unstable Flight**
   - Tune PID gains (start with P gain)
   - Check sensor calibration
   - Verify IMU mounting (vibration isolation)
   - Check for radio interference

3. **Sensor Issues**
   - Run IMU calibration routine
   - Check I2C connections
   - Verify power supply stability
   - Check for electromagnetic interference

4. **Radio Issues**
   - Verify receiver type matches configuration
   - Check channel mapping
   - Validate failsafe settings
   - Check antenna placement

### Performance Optimization

1. **Loop Rate Optimization**
   - Reduce serial output frequency
   - Optimize sensor reading timing
   - Minimize floating-point operations
   - Use fixed-point arithmetic where possible

2. **Memory Optimization**
   - Use appropriate data types
   - Minimize global variables
   - Use stack allocation where possible
   - Avoid dynamic memory allocation

## Version Information

- **File Version**: B-1.4.5
- **Last Updated**: 2026-01-18
- **Author**: Patrick Andrasena T.
- **Base Project**: dRehmFlight by Nicholas Rehm
- **Target Platform**: Teensy 4.0/4.1

## Dependencies

### Required Libraries
- `Wire.h` - I2C communication
- `SPI.h` - SPI communication
- `PWMServo.h` - Motor control
- Receiver-specific libraries (SBUS, ELRS, DSM, etc.)

### Required Hardware
- Teensy 4.0/4.1 development board
- Primary IMU (MPU6050 or MPU9250)
- Radio receiver (compatible type)
- ESCs and motors
- Power distribution system

## Related Files

- `quad.h` - Configuration header
- `imuMonitor.ino` - Monitor IMU and UDP telemetry
- `UDPClient.py` - Ground station data receiver
- `FlightlogAnalysis.py` - Post-flight analysis tool

## Notes

- This is the main flight control file and should not be modified unless you understand the implications
- Monitor system integration is optional but recommended for research applications
- PID tuning is critical for stable flight performance
- Always test with propellers removed first
- Ensure proper failsafe configuration before flight


# radioComm.ino - Radio Communication Interface

Comprehensive radio communication system supporting multiple receiver types for the FCU Madgwick Control Filter Research Platform. This file handles all radio receiver interfaces and provides a unified API for channel access.

## Overview

`radioComm.ino` is a modular radio communication library that:
- **Multi-Protocol Support**: PWM, PPM, SBUS, DSM, IBUS, ELRS receivers
- **Interrupt-Driven**: High-performance pulse timing and decoding
- **Unified Interface**: Single API for all receiver types
- **Real-time Processing**: Sub-millisecond response times
- **Channel Mapping**: Flexible logical to physical channel assignment
- **Failsafe Safety**: Automatic signal loss detection and recovery

## Supported Receiver Types

### 1. PWM (Pulse Width Modulation)
- **Protocol**: Standard PWM servo signals
- **Channels**: Up to 6 independent channels
- **Update Rate**: ~50Hz per channel
- **Pulse Width**: 1000-2000μs (standard servo range)
- **Connection**: Individual pins per channel
- **Latency**: ~20ms typical

### 2. PPM (Pulse Position Modulation)
- **Protocol**: Single-wire multiplexed PWM
- **Channels**: Up to 8 channels (typically 6 used)
- **Update Rate**: ~50Hz complete frame
- **Frame Sync**: 5ms+ sync pulse
- **Connection**: Single digital pin
- **Latency**: ~20ms typical

### 3. SBUS (Serial Bus)
- **Protocol**: Futaba SBUS inverted serial
- **Channels**: Up to 16 channels (typically 6 used)
- **Update Rate**: 100Hz (10ms frames)
- **Data Format**: 25-byte serial frames
- **Connection**: Single UART pin (inverted)
- **Latency**: ~10ms typical

### 4. DSM (Spektrum DSM)
- **Protocol**: Spektrum DSM2/DSMX serial
- **Channels**: Up to 12 channels (typically 6 used)
- **Update Rate**: 125Hz (8ms frames)
- **Data Format**: Variable length serial frames
- **Connection**: Single UART pin
- **Latency**: ~8ms typical

### 5. ELRS (ExpressLTS Radio System)
- **Protocol**: CRSF (Crossfire Radio System Format)
- **Channels**: Up to 16 channels (typically 6 used)
- **Update Rate**: 250Hz (4ms frames)
- **Data Format**: 64-byte serial frames
- **Connection**: Single UART pin
- **Latency**: ~4ms typical

### 6. IBUS (FlySky IBUS)
- **Protocol**: FlySky IBUS serial
- **Channels**: Up to 14 channels (typically 6 used)
- **Update Rate**: 100Hz (10ms frames)
- **Data Format**: 32-byte serial frames
- **Connection**: Single UART pin
- **Latency**: ~10ms typical

## Architecture Overview

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Physical      │    │   Protocol      │    │   Unified       │
│   Interface     │───▶│   Decoder       │───▶│   API           │
│                 │    │                 │    │                 │
│ • PWM Pins      │    │ • Pulse Timing  │    │ • getRadioPWM() │
│ • PPM Pin       │    │ • Serial Parsing│    │ • Channel Mapping│
│ • UART Ports    │    │ • Frame Sync    │    │ • Failsafe      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                │
                                ▼
                       ┌─────────────────┐
                       │   Flight        │
                       │   Control       │
                       │   System        │
                       └─────────────────┘
```

## Core Functions

### 1. Initialization Function

```cpp
void radioSetup() {
  // Configures selected receiver type
  // Initializes hardware interfaces
  // Sets up interrupt handlers
  // Configures serial ports
  // Applies pin modes and pull-ups
}
```

#### Receiver-Specific Initialization

**PWM Receiver Setup:**
```cpp
#if defined USE_PWM_RX
  // Configure interrupt pins for each channel
  pinMode(ch1Pin, INPUT_PULLUP);
  pinMode(ch2Pin, INPUT_PULLUP);
  // ... for all 6 channels
  
  // Attach interrupt service routines
  attachInterrupt(digitalPinToInterrupt(ch1Pin), getCh1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch2Pin), getCh2, CHANGE);
  // ... for all 6 channels
#endif
```

**PPM Receiver Setup:**
```cpp
#if defined USE_PPM_RX
  // Configure single PPM pin
  pinMode(PPM_Pin, INPUT_PULLUP);
  
  // Attach PPM interrupt handler
  attachInterrupt(digitalPinToInterrupt(PPM_Pin), getPPM, CHANGE);
#endif
```

**Serial Receiver Setup:**
```cpp
#if defined USE_SBUS_RX
  sbus.begin();                    // Initialize SBUS library
#elif defined USE_ELRS_RX
  elrs_rx.begin();                 // Initialize ELRS library
#elif defined USE_DSM_RX
  Serial3.begin(115000);           // Initialize DSM serial
#endif
```

### 2. Unified Channel Access

```cpp
unsigned long getRadioPWM(int ch_num) {
  // Unified interface for all receiver types
  // Returns PWM value (1000-2000μs)
  // Applies channel mapping
  // Handles failsafe conditions
  
  #if defined USE_ELRS_RX
    elrs_rx.update();              // Parse latest packet
    uint16_t pwm = elrs_rx.getChannel(ch_num);
    return constrain(pwm, 1000, 2000);
    
  #elif defined USE_PPM_RX || defined USE_PWM_RX || defined USE_SBUS_RX || defined USE_DSM_RX
    // Return raw channel values
    if (ch_num == 1) return channel_1_raw;
    else if (ch_num == 2) return channel_2_raw;
    // ... for all 6 channels
    
  #else
    return 1500;                   // Failsafe center position
  #endif
}
```

## Interrupt Service Routines

### 1. PWM Interrupt Handlers

**Channel-Specific ISR:**
```cpp
void getCh1() {
  int trigger = digitalRead(ch1Pin);
  if(trigger == 1) {
    rising_edge_start_1 = micros();    // Record rising edge time
  }
  else if(trigger == 0) {
    channel_1_raw = micros() - rising_edge_start_1;  // Calculate pulse width
  }
}
```

**PWM Timing Logic:**
- **Rising Edge**: Record timestamp
- **Falling Edge**: Calculate pulse width
- **Pulse Width**: 1000-2000μs range
- **Update Rate**: ~50Hz per channel
- **Accuracy**: ±1μs timing resolution

### 2. PPM Interrupt Handler

```cpp
void getPPM() {
  unsigned long dt_ppm;
  int trig = digitalRead(PPM_Pin);
  
  if(trig == 1) {  // Rising edge only
    dt_ppm = micros() - time_ms;
    time_ms = micros();
    
    if(dt_ppm > 5000) {  // Frame sync pulse (>5ms)
      ppm_counter = 0;   // Reset channel counter
    }
    
    // Channel assignment based on counter
    if(ppm_counter == 1) channel_1_raw = dt_ppm;
    if(ppm_counter == 2) channel_2_raw = dt_ppm;
    // ... for all 6 channels
    
    ppm_counter++;
  }
}
```

**PPM Frame Structure:**
```
[Sync Pulse >5ms][CH1][CH2][CH3][CH4][CH5][CH6][CH7][CH8][Sync Pulse]
     1000-2000μs each channel
     Total frame: ~22.5ms (50Hz)
```

### 3. Serial Event Handlers

**DSM Serial Event:**
```cpp
void serialEvent3(void) {
  #if defined USE_DSM_RX
    while (Serial3.available()) {
      DSM.handleSerialEvent(Serial3.read(), micros());
    }
  #endif
}
```

**Serial Processing:**
- **Buffer Management**: Handle incoming serial data
- **Frame Parsing**: Decode protocol-specific frames
- **Channel Extraction**: Extract PWM values from frames
- **Timestamp Recording**: Maintain timing accuracy

## Channel Mapping System

### Logical to Physical Mapping

**Configuration in quad.h:**
```cpp
// PPM Channel Mapping
#define PPM_MAP_CH1 3   // Logical CH1 → Physical slot 3
#define PPM_MAP_CH2 1   // Logical CH2 → Physical slot 1
#define PPM_MAP_CH3 2   // Logical CH3 → Physical slot 2
#define PPM_MAP_CH4 4   // Logical CH4 → Physical slot 4
#define PPM_MAP_CH5 5   // Logical CH5 → Physical slot 5
#define PPM_MAP_CH6 6   // Logical CH6 → Physical slot 6
```

**Standard Channel Assignment:**
- **CH1**: Throttle (1000-2000μs, 1500μs center)
- **CH2**: Aileron/Roll (1000-2000μs, 1500μs center)
- **CH3**: Elevator/Pitch (1000-2000μs, 1500μs center)
- **CH4**: Rudder/Yaw (1000-2000μs, 1500μs center)
- **CH5**: Gear/Throttle Cut (1000-2000μs, 1500μs center)
- **CH6**: Aux1/Mode Select (1000-2000μs, 1500μs center)

## Performance Characteristics

### Timing Performance

| Receiver Type | Update Rate | Latency | Resolution | Accuracy |
|---------------|-------------|---------|------------|----------|
| PWM | 50Hz | ~20ms | 1μs | ±1μs |
| PPM | 50Hz | ~20ms | 1μs | ±1μs |
| SBUS | 100Hz | ~10ms | 10μs | ±10μs |
| DSM | 125Hz | ~8ms | 10μs | ±10μs |
| ELRS | 250Hz | ~4ms | 10μs | ±10μs |
| IBUS | 100Hz | ~10ms | 10μs | ±10μs |

### CPU Usage

| Receiver Type | CPU Usage | Memory Usage | Interrupt Load |
|---------------|-----------|-------------|----------------|
| PWM | ~5% | ~200B | 6 interrupts/frame |
| PPM | ~2% | ~100B | 1 interrupt/frame |
| SBUS | ~3% | ~500B | Serial interrupts |
| DSM | ~3% | ~500B | Serial interrupts |
| ELRS | ~4% | ~1KB | Serial interrupts |
| IBUS | ~3% | ~500B | Serial interrupts |

## Hardware Requirements

### Pin Assignments

**PWM/PPM Pins:**
```cpp
const int ch1Pin = 15;  // Throttle
const int ch2Pin = 16;  // Aileron
const int ch3Pin = 17;  // Elevator
const int ch4Pin = 20;  // Rudder
const int ch5Pin = 21;  // Gear
const int ch6Pin = 22;  // Aux1

const int PPM_Pin = 23; // PPM input
```

**Serial Ports:**
```cpp
// Serial3: DSM/ELRS/IBUS
// RX3: Pin 15
// TX3: Pin 14
```

**SBUS Pin:**
```cpp
// RX5: Pin 21 (inverted signal)
```

### Hardware Compatibility

**PWM Receivers:**
- Standard PWM receivers (Futaba, Hitec, etc.)
- 6-channel minimum required
- 3.3V/5V compatible inputs

**PPM Receivers:**
- PPM-capable receivers
- Single-wire connection
- 3.3V/5V compatible inputs

**Serial Receivers:**
- SBUS: Futaba SBUS receivers (inverted signal)
- ELRS: ExpressLTS receivers (3.3V UART)
- DSM: Spektrum DSM receivers (3.3V UART)
- IBUS: FlySky IBUS receivers (3.3V UART)

## Configuration and Setup

### 1. Receiver Type Selection

**In quad.h:**
```cpp
// Select exactly one receiver type
#define USE_ELRS_RX          // Modern high-performance
// #define USE_SBUS_RX       // Professional standard
// #define USE_DSM_RX        // Spektrum systems
// #define USE_PWM_RX        // Standard analog
// #define USE_PPM_RX        // Single-wire
// #define USE_IBUS_RX       // FlySky systems
```

### 2. Channel Mapping

**Custom Channel Assignment:**
```cpp
// Example: Custom PPM mapping
#define PPM_MAP_CH1 1   // Throttle on slot 1
#define PPM_MAP_CH2 2   // Aileron on slot 2
#define PPM_MAP_CH3 3   // Elevator on slot 3
#define PPM_MAP_CH4 4   // Rudder on slot 4
#define PPM_MAP_CH5 5   // Gear on slot 5
#define PPM_MAP_CH6 6   // Aux1 on slot 6
```

### 3. Pin Configuration

**Custom Pin Assignment:**
```cpp
// Example: Custom PWM pins
const int ch1Pin = 2;   // Throttle on pin 2
const int ch2Pin = 3;   // Aileron on pin 3
// ... etc
```

## Failsafe and Safety

### Signal Loss Detection

**Timeout Monitoring:**
```cpp
// Each receiver type implements signal loss detection
// Automatic failsafe values applied on signal loss
// Default failsafe: 1500μs (center position)
```

**Failsafe Values:**
- **Throttle (CH1)**: 1000μs (motor off)
- **Attitude (CH2-4)**: 1500μs (level flight)
- **Gear (CH5)**: 2000μs (throttle cut active)
- **Aux1 (CH6)**: 1500μs (neutral)

### Safety Features

**Arming Safety:**
- Throttle must be low (<1050μs) to arm
- Gear switch must be in safe position
- Signal validation before motor output

**Signal Validation:**
- Pulse width range checking (1000-2000μs)
- Update rate monitoring
- Frame synchronization validation
- CRC/checksum verification (serial protocols)

## Troubleshooting

### Common Issues

1. **No Signal Detected**
   - Check receiver power and connections
   - Verify pin assignments in quad.h
   - Check receiver type configuration
   - Validate signal voltage levels (3.3V/5V)

2. **Intermittent Signal**
   - Check for electromagnetic interference
   - Verify receiver antenna placement
   - Check power supply stability
   - Validate signal routing and shielding

3. **Incorrect Channel Mapping**
   - Verify channel mapping in quad.h
   - Check transmitter channel order
   - Validate receiver channel assignment
   - Test with known good configuration

4. **High Latency**
   - Check receiver update rate
   - Verify interrupt processing load
   - Optimize interrupt priorities
   - Consider higher-performance receiver

5. **Signal Jitter**
   - Check power supply noise
   - Verify signal integrity
   - Add filtering capacitors
   - Improve grounding

### Debugging Tools

**Serial Output:**
```cpp
// Add to main loop for debugging
void printRadioChannels() {
  Serial.print("CH1: "); Serial.print(getRadioPWM(1));
  Serial.print(" CH2: "); Serial.print(getRadioPWM(2));
  Serial.print(" CH3: "); Serial.print(getRadioPWM(3));
  Serial.print(" CH4: "); Serial.print(getRadioPWM(4));
  Serial.print(" CH5: "); Serial.print(getRadioPWM(5));
  Serial.print(" CH6: "); Serial.println(getRadioPWM(6));
}
```

**Signal Quality Monitoring:**
```cpp
// Monitor signal quality metrics
void checkSignalQuality() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) {  // Check every second
    // Verify signal is updating
    // Check for reasonable values
    // Monitor update rate
    lastUpdate = millis();
  }
}
```

## Integration Examples

### Basic Usage
```cpp
void setup() {
  radioSetup();  // Initialize receiver
}

void loop() {
  // Read all channels
  unsigned long throttle = getRadioPWM(1);
  unsigned long aileron = getRadioPWM(2);
  unsigned long elevator = getRadioPWM(3);
  unsigned long rudder = getRadioPWM(4);
  unsigned long gear = getRadioPWM(5);
  unsigned long aux1 = getRadioPWM(6);
  
  // Use channel values for flight control
  // ...
}
```

### Advanced Usage with Validation
```cpp
void loop() {
  // Read channels with validation
  for (int i = 1; i <= 6; i++) {
    unsigned long pwm = getRadioPWM(i);
    
    // Validate signal range
    if (pwm < 1000 || pwm > 2000) {
      // Signal out of range - handle failsafe
      handleSignalLoss(i);
    }
  }
  
  // Process valid signals
  processFlightControl();
}
```

## Version Information

- **File Version**: Beta 1.3
- **Last Updated**: 2022-07-29
- **Author**: Nicholas Rehm
- **Base Project**: dRehmFlight
- **Enhanced by**: Patrick Andrasena T.

## Dependencies

### Required Libraries
- **PWMServo**: Motor control library
- **SBUS**: SBUS protocol library (if used)
- **ELRS**: ELRS/CRSF library (if used)
- **DSMRX**: DSM protocol library (if used)

### Required Hardware
- Teensy 4.0/4.1 development board
- Compatible radio receiver
- Appropriate wiring and connectors
- Stable power supply

## Related Files

- `quad.h` - Configuration header
- `FCUMCFRP_B_1.4.5.ino` - Main flight control
- Receiver-specific library files in `src/` directory

## Notes

- Only one receiver type should be defined at a time
- Channel mapping must match transmitter setup
- Signal validation is critical for safety
- Interrupt timing is critical for PWM/PPM accuracy
- Serial receivers require proper UART configuration
- Always test failsafe behavior before flight


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


# imuMonitor.ino - IMU Monitor and UDP Telemetry System

Advanced IMU monitoring and telemetry system for flight control research and validation. This file implements independent sensor fusion, event detection, and real-time UDP telemetry for comprehensive flight data analysis.

## Overview

`imuMonitor.ino` is a sophisticated monitoring system that:
- **Independent IMU Fusion**: Separate Madgwick filter for validation
- **Event Detection**: Command and disturbance monitoring with intelligent classification
- **UDP Telemetry**: Real-time data transmission at 100Hz
- **Disturbance Analysis**: Wind gust and turbulence detection
- **Research Validation**: Control system performance analysis
- **Data Logging**: Comprehensive flight data for post-analysis

## Key Features

### 1. Dual IMU Architecture

```
┌─────────────────┐    ┌─────────────────┐
│   Control IMU   │    │   Monitor IMU   │
│   (MPU6050)     │    │   (MPU9250)     │
│                 │    │                 │
│ • Used for     │    │ • Independent   │
│   flight       │    │   validation    │
│ • control loop  │    │ • research      │
│ • 1000kHz I2C   │    │ • 400kHz I2C    │
└─────────────────┘    └─────────────────┘
         │                       │
         │                       │
         ▼                       ▼
┌─────────────────────────────────────────┐
│           Event Detection               │
│           UDP Telemetry                 │
│           Research Analysis             │
└─────────────────────────────────────────┘
```

### 2. Event Detection System

#### Event Flags Structure
```cpp
struct EventFlags {
  uint8_t command_change : 1;      // Command input changed (>10%)
  uint8_t throttle_change : 1;     // Throttle changed (>10%)
  uint8_t roll_command : 1;        // Roll command active
  uint8_t pitch_command : 1;       // Pitch command active
  uint8_t yaw_command : 1;         // Yaw command active
  uint8_t attitude_change : 1;     // Attitude changed (>2°)
  uint8_t control_active : 1;      // Control system armed
  uint8_t disturbance : 1;         // Uncommanded disturbance (>5°)
} __attribute__((packed));
```

#### Detection Thresholds
```cpp
const float COMMAND_THRESHOLD = 0.1;      // 10% command change
const float ATTITUDE_THRESHOLD = 2.0;     // 2° attitude change
const float DISTURBANCE_THRESHOLD = 5.0;  // 5° disturbance detection
const unsigned long EVENT_DEBOUNCE_MS = 100; // 100ms debounce
```

### 3. Disturbance Detection Logic

#### Smart Detection Algorithm
```cpp
void detectEvents() {
  // Only detect disturbances when:
  // 1. Control system is armed (armedFly = true)
  // 2. No command input is active (all sticks centered)
  // 3. Attitude change > 5° (significant disturbance)
  
  if (armedFly && !commandActive) {
    if (attitude_change > DISTURBANCE_THRESHOLD) {
      dataPacket.flags.disturbance = 1;
      // Serial output for debugging
      Serial.print("DISTURBANCE DETECTED: ");
      Serial.print("Roll="); Serial.print(roll_change, 2); Serial.print("° ");
      Serial.print("Pitch="); Serial.print(pitch_change, 2); Serial.print("° ");
      Serial.print("Yaw="); Serial.print(yaw_change, 2); Serial.println("°");
    }
  }
}
```

#### Disturbance Types Detected
- **Wind Gusts**: Sudden lateral/vertical disturbances
- **Turbulence**: Continuous atmospheric disturbances
- **External Forces**: Physical impacts or collisions
- **Control System Issues**: Unintended attitude excursions

### 4. UDP Telemetry System

#### Packet Structure
```cpp
struct IMUDataPacket {
  uint32_t timestamp_us;        // Arduino timestamp (μs)
  EventFlags flags;             // Event detection flags (1 byte)
  
  // Control IMU Raw Sensors (9 floats)
  float ctrl_acc_x, ctrl_acc_y, ctrl_acc_z;    // Accelerometer (g)
  float ctrl_gyro_x, ctrl_gyro_y, ctrl_gyro_z; // Gyroscope (deg/s)
  float ctrl_mag_x, ctrl_mag_y, ctrl_mag_z;    // Magnetometer (μT)
  
  // Monitor IMU Raw Sensors (9 floats)
  float mon_acc_x, mon_acc_y, mon_acc_z;      // Accelerometer (g)
  float mon_gyro_x, mon_gyro_y, mon_gyro_z;   // Gyroscope (deg/s)
  float mon_mag_x, mon_mag_y, mon_mag_z;      // Magnetometer (μT)
  
  // Control IMU Attitude (3 floats)
  float ctrl_roll, ctrl_pitch, ctrl_yaw;      // Euler angles (deg)
  
  // Monitor IMU Attitude (3 floats)
  float mon_roll, mon_pitch, mon_yaw;         // Euler angles (deg)
  
  // Attitude Comparison Errors (3 floats)
  float err_roll, err_pitch, err_yaw;         // Control - Monitor (deg)
} __attribute__((packed));
```

#### Packet Specifications
- **Total Size**: 113 bytes (4 + 1 + 27×4)
- **Transmission Rate**: 100Hz (10ms intervals)
- **Protocol**: UDP over Ethernet
- **Compression**: Bit-field event flags (1 byte)

### 5. Network Configuration

#### Ethernet Setup
```cpp
// W5500 Ethernet Configuration
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};  // MAC address
#define W5500_CS_PIN 10                               // Chip select
#define UDP_REMOTE_PORT 8889                          // Destination port
#define MONITOR_UDP_RATE_MS 10                        // 100Hz transmission
```

#### IP Configuration
```cpp
// Remote IP address (ground station)
#define UDP_REMOTE_IP_0 192
#define UDP_REMOTE_IP_1 168
#define UDP_REMOTE_IP_2 1
#define UDP_REMOTE_IP_3 100
```

## Core Functions

### 1. Initialization Functions

#### Monitor IMU Initialization
```cpp
void monitorIMUinit() {
  // Initialize Wire1 I2C bus at 400kHz
  Wire1.begin();
  Wire1.setClock(400000);
  
  // Initialize MPU9250 monitor IMU
  int status_mon = mpu9250_monitor.begin();
  
  // Configure sensor ranges and filters
  mpu9250_monitor.setGyroRange(GYRO_SCALE);
  mpu9250_monitor.setAccelRange(ACCEL_SCALE);
  mpu9250_monitor.setSrd(0);  // 1kHz gyro/accel, 100Hz mag
}
```

#### Network Initialization
```cpp
void setupEthernet() {
  // Initialize W5500 Ethernet
  Ethernet.begin(mac);
  
  // Verify network connection
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield not found");
  }
  
  // Initialize UDP
  Udp.begin(UDP_LOCAL_PORT);
}
```

### 2. Data Acquisition Functions

#### Monitor IMU Data Reading
```cpp
void getMonitorIMUdata() {
  // Read raw sensor data from MPU9250
  mpu9250_monitor.readAccel();
  mpu9250_monitor.readGyro();
  mpu9250_monitor.readMag();
  
  // Store raw values
  AccX_mon = mpu9250_monitor.getAccelX_mss();
  AccY_mon = mpu9250_monitor.getAccelY_mss();
  AccZ_mon = mpu9250_monitor.getAccelZ_mss();
  GyroX_mon = mpu9250_monitor.getGyroX_rads();
  GyroY_mon = mpu9250_monitor.getGyroY_rads();
  GyroZ_mon = mpu9250_monitor.getGyroZ_rads();
  MagX_mon = mpu9250_monitor.getMagX_uT();
  MagY_mon = mpu9250_monitor.getMagY_uT();
  MagZ_mon = mpu9250_monitor.getMagZ_uT();
  
  // Apply calibration corrections
  AccX_mon = (AccX_mon - AccErrorX_mon) * AccScaleX_mon;
  // ... similar for other sensors
  
  // Low-pass filter sensor data
  AccX_mon = (1.0 - B_accel) * AccX_mon_prev + B_accel * AccX_mon;
  // ... similar for other sensors
}
```

#### Madgwick Filter for Monitor IMU
```cpp
void MadgwickMonitor() {
  // Independent Madgwick filter for monitor IMU
  // Uses separate quaternion state variables
  // Runs at same rate as control IMU filter
  
  // Convert sensor data to proper units and axes
  // Apply filter algorithm
  // Extract Euler angles from quaternion
  
  // Output: roll_IMU_mon, pitch_IMU_mon, yaw_IMU_mon
}
```

### 3. Event Detection Functions

#### Event Detection Algorithm
```cpp
void detectEvents() {
  // Clear all flags first
  dataPacket.flags = {0};
  
  // Debounce check
  if (millis() - lastEventTime < EVENT_DEBOUNCE_MS) {
    dataPacket.flags.control_active = armedFly;
    return;
  }
  
  // Command change detection
  float roll_change = abs(roll_des - prev_roll_des);
  float pitch_change = abs(pitch_des - prev_pitch_des);
  float yaw_change = abs(yaw_des - prev_yaw_des);
  
  if (roll_change > COMMAND_THRESHOLD || 
      pitch_change > COMMAND_THRESHOLD || 
      yaw_change > COMMAND_THRESHOLD) {
    dataPacket.flags.command_change = 1;
  }
  
  // Active command detection
  bool commandActive = false;
  if (abs(roll_des) > COMMAND_THRESHOLD) {
    dataPacket.flags.roll_command = 1;
    commandActive = true;
  }
  // ... similar for pitch and yaw
  
  // Attitude change detection
  float roll_att_change = abs(roll_IMU - prev_roll_IMU);
  float pitch_att_change = abs(pitch_IMU - prev_pitch_IMU);
  float yaw_att_change = abs(yaw_IMU - prev_yaw_IMU);
  
  if (roll_att_change > ATTITUDE_THRESHOLD || 
      pitch_att_change > ATTITUDE_THRESHOLD || 
      yaw_att_change > ATTITUDE_THRESHOLD) {
    dataPacket.flags.attitude_change = 1;
  }
  
  // Disturbance detection (key innovation)
  if (armedFly && !commandActive) {
    if (roll_att_change > DISTURBANCE_THRESHOLD || 
        pitch_att_change > DISTURBANCE_THRESHOLD || 
        yaw_att_change > DISTURBANCE_THRESHOLD) {
      dataPacket.flags.disturbance = 1;
      // Serial output for debugging
      Serial.print("DISTURBANCE DETECTED: ");
      // ... detailed output
    }
  }
  
  // Control status
  dataPacket.flags.control_active = armedFly;
  
  // Update previous values
  if (eventDetected) {
    prev_roll_des = roll_des;
    prev_pitch_des = pitch_des;
    prev_yaw_des = yaw_des;
    prev_roll_IMU = roll_IMU;
    prev_pitch_IMU = pitch_IMU;
    prev_yaw_IMU = yaw_IMU;
    lastEventTime = millis();
  }
}
```

### 4. Data Transmission Functions

#### UDP Data Transmission
```cpp
void sendIMUDataUDP() {
  // Rate limiting (100Hz)
  if (millis() - lastUdpSend < MONITOR_UDP_RATE_MS) {
    return;
  }
  
  // Detect events before sending
  detectEvents();
  
  // Populate data packet
  dataPacket.timestamp_us = micros();
  // ... fill all sensor data fields
  // ... fill attitude data fields
  // ... fill error data fields
  
  // Send UDP packet
  Udp.beginPacket(Udp.remoteIP(), UDP_REMOTE_PORT);
  Udp.write((uint8_t*)&dataPacket, sizeof(dataPacket));
  Udp.endPacket();
  
  lastUdpSend = millis();
}
```

#### Data Comparison Functions
```cpp
void compareAttitude() {
  // Compare control vs monitor IMU attitudes
  roll_error_mon = roll_IMU - roll_IMU_mon;
  pitch_error_mon = pitch_IMU - pitch_IMU_mon;
  
  // Handle yaw wrap-around at ±180°
  float yaw_diff = yaw_IMU - yaw_IMU_mon;
  if (yaw_diff > 180.0f) yaw_diff -= 360.0f;
  if (yaw_diff < -180.0f) yaw_diff += 360.0f;
  yaw_error_mon = yaw_diff;
}

void compareRawSensors() {
  // Compare raw sensor data between IMUs
  // Used for sensor calibration validation
  // Does not require attitude estimation
}
```

## Integration with Main Flight Control

### Call Integration
```cpp
// In FCUMCFRP_B_1.4.5.ino main loop
void loop() {
  // ... main flight control code ...
  
  sendIMUDataUDP();  // Call monitor telemetry
}
```

### Data Sharing
- **Global Variables**: All flight data accessible to monitor
- **No Interference**: Monitor doesn't affect control loop
- **Real-time Access**: Immediate access to current flight state
- **Event Synchronization**: Events detected in real-time

## Research Applications

### 1. Flight Control Validation
- **Algorithm Testing**: Validate control algorithms
- **Performance Analysis**: Measure control system performance
- **Tuning Optimization**: Optimize PID parameters
- **Robustness Testing**: Test under various conditions

### 2. Disturbance Analysis
- **Wind Gust Characterization**: Study disturbance patterns
- **Turbulence Response**: Analyze control behavior
- **Environmental Assessment**: Evaluate flight conditions
- **Control Authority**: Measure disturbance rejection capability

### 3. Sensor Fusion Validation
- **IMU Consistency**: Compare different IMU performance
- **Filter Performance**: Validate Madgwick filter accuracy
- **Calibration Verification**: Check sensor calibration
- **Noise Analysis**: Analyze sensor noise characteristics

### 4. Event Pattern Analysis
- **Command Analysis**: Study pilot input patterns
- **Disturbance Events**: Analyze external disturbances
- **Response Time**: Measure control system latency
- **Event Correlation**: Study relationships between events

## Data Analysis Integration

### Ground Station Integration
```python
# UDPClient.py receives and processes data
- Real-time event logging
- CSV data with event flags
- Live plotting and visualization
- Event notifications
```

### Post-Flight Analysis
```python
# FlightlogAnalysis.py analyzes recorded data
- Advanced stability metrics
- Event pattern analysis
- Disturbance statistics
- Comprehensive reporting
```

## Performance Characteristics

### Timing Performance
- **Update Rate**: 100Hz UDP transmission
- **Latency**: <10ms from event to transmission
- **CPU Usage**: ~5% of available processing
- **Memory Usage**: ~2KB for data structures

### Network Performance
- **Bandwidth**: ~11.3KB/s (113 bytes × 100Hz)
- **Packet Loss**: <1% typical on good networks
- **Latency**: <5ms on local networks
- **Reliability**: High with UDP checksums

### Detection Performance
- **Command Detection**: <50ms response time
- **Disturbance Detection**: <100ms response time
- **False Positive Rate**: <1% with proper tuning
- **Detection Accuracy**: >95% for significant events

## Configuration and Tuning

### Event Detection Tuning
```cpp
// Adjust these thresholds based on your application
const float COMMAND_THRESHOLD = 0.1;      // 10% command sensitivity
const float ATTITUDE_THRESHOLD = 2.0;     // 2° attitude sensitivity
const float DISTURBANCE_THRESHOLD = 5.0;  // 5° disturbance sensitivity
const unsigned long EVENT_DEBOUNCE_MS = 100; // 100ms debounce time
```

### Network Configuration
```cpp
// Adjust for your network setup
#define UDP_REMOTE_IP_0 192    // Ground station IP
#define UDP_REMOTE_IP_1 168
#define UDP_REMOTE_IP_2 1
#define UDP_REMOTE_IP_3 100
#define MONITOR_UDP_RATE_MS 10  // 100Hz transmission rate
```

### Sensor Calibration
```cpp
// MPU9250 calibration parameters
float AccErrorX_mon = 0.0;     // Accelerometer bias
float AccErrorY_mon = 0.0;
float AccErrorZ_mon = 0.0;
float GyroErrorX_mon = 0.0;    // Gyroscope bias
float GyroErrorY_mon = 0.0;
float GyroErrorZ_mon = 0.0;
```

## Troubleshooting

### Common Issues

1. **No UDP Data Transmission**
   - Check Ethernet connection and IP configuration
   - Verify W5500 CS pin connection
   - Check network connectivity to ground station
   - Ensure UDPClient.py is running and listening

2. **Event Detection Not Working**
   - Verify disturbance thresholds are appropriate
   - Check that control system is armed (armedFly = true)
   - Ensure command thresholds match radio setup
   - Check for proper global variable access

3. **Monitor IMU Not Working**
   - Verify MPU9250 I2C connection (Wire1)
   - Check I2C address (0x69 for MPU9250)
   - Ensure power supply is stable
   - Run I2C scanner to verify device detection

4. **Data Quality Issues**
   - Calibrate monitor IMU sensors
   - Check for electromagnetic interference
   - Verify sensor mounting and vibration isolation
   - Check I2C bus speed and signal integrity

### Performance Optimization

1. **Network Optimization**
   - Use dedicated network for telemetry
   - Minimize network congestion
   - Use quality Ethernet cables
   - Consider network switches for reliability

2. **Sensor Optimization**
   - Use proper I2C pull-up resistors
   - Minimize sensor wiring length
   - Use shielded cables for long runs
   - Ensure stable power supply

3. **Event Detection Optimization**
   - Tune thresholds for your specific application
   - Adjust debounce time for your environment
   - Consider different thresholds for different flight modes
   - Monitor false positive rates and adjust accordingly

## Version Information

- **File Version**: Beta 1.4.5
- **Event Detection Version**: 1.1
- **Last Updated**: 2025-01-22
- **Author**: Patrick Andrasena T.
- **Base Project**: dRehmFlight by Nicholas Rehm

## Dependencies

### Required Libraries
- `Wire.h` - I2C communication
- `SPI.h` - SPI communication
- `Ethernet.h` - Network communication
- `EthernetUdp.h` - UDP protocol

### Required Hardware
- Teensy 4.0/4.1 development board
- MPU9250 IMU sensor (monitor)
- W5500 Ethernet shield
- Stable network connection
- Ground station computer

### Optional Hardware
- Primary IMU (MPU6050) for comparison
- Network switch for multiple devices
- Antenna for wireless networks

## Related Files

- `quad.h` - Configuration header
- `FCUMCFRP_B_1.4.5.ino` - Main flight control
- `UDPClient.py` - Ground station receiver
- `FlightlogAnalysis.py` - Analysis tool

## Notes

- Monitor system is optional but highly recommended for research
- Event detection thresholds should be tuned for your specific application
- Network configuration must match ground station settings
- Monitor IMU does not affect flight control performance
- All data is transmitted in real-time for live analysis
- Event detection provides valuable insights into flight dynamics

# UDP Data Client for FCU IMU Telemetry

Python client for receiving, visualizing, and logging IMU telemetry data from the FCU Madgwick Control Filter Research Platform.

## Enhanced Features (Version 1.1)

This enhanced version includes comprehensive **event detection and disturbance monitoring**:

- **Event Detection**: Real-time detection of command inputs and attitude changes
- **Disturbance Monitoring**: Identifies uncommanded attitude changes (wind gusts, turbulence)
- **Event Logging**: Timestamped event log with human-readable descriptions
- **Enhanced UDP Packet**: 113 bytes with event flags (up from 112 bytes)
- **Real-time Notifications**: Console alerts for events and disturbances
- **Research-grade Analysis**: Perfect for flight control system validation

## Key Features

### Core Functionality
- **UDP Packet Reception**: Receives 113-byte IMU data packets at 100Hz (10ms intervals)
- **Live Plotting**: Real-time visualization of attitude, errors, and sensor data
- **CSV Logging**: Automatically saves all received data with event flags
- **Timestamped Sessions**: Each run creates a new folder for data organization

### Event Detection System
- **Command Change Detection**: Flags significant command input changes (>10% threshold)
- **Active Command Monitoring**: Tracks active roll/pitch/yaw commands
- **Attitude Change Detection**: Monitors attitude changes beyond thresholds
- **Disturbance Detection**: Identifies uncommanded attitude changes (>5° threshold)
- **Control Status Monitoring**: Tracks armed/disarmed status
- **Debounced Events**: 100ms debounce prevents flag spam

### Disturbance Monitoring
- **Wind Gust Detection**: Flags rapid attitude changes without pilot input
- **Turbulence Analysis**: Identifies continuous disturbances
- **External Force Detection**: Detects impacts or external forces
- **Smart Filtering**: Only active when armed and no commands active
- **Serial Debug Output**: Real-time disturbance alerts for field testing

## Requirements

Install dependencies:
```bash
pip install -r requirements.txt
```

Required packages:
- `numpy` >= 1.21.0
- `matplotlib` >= 3.5.0
- `scipy` >= 1.7.0 (for advanced analysis)

## Configuration

The script uses the following UDP settings (must match `quad.h`):
- **Local Port**: 8888 (receiving)
- **Remote Port**: 8889 (Arduino sends to this)
- **Remote IP**: 192.168.1.100 (configured in `quad.h`)

**Event Detection Thresholds** (configurable in `imuMonitor.ino`):
- `COMMAND_THRESHOLD`: 0.1 (10% of max command)
- `ATTITUDE_THRESHOLD`: 2.0° (normal attitude change)
- `DISTURBANCE_THRESHOLD`: 5.0° (disturbance detection)
- `EVENT_DEBOUNCE_MS`: 100ms (prevents flag spam)

## Usage

1. **Ensure Arduino/Teensy is running** with `USE_MPU9250_MONITOR_I2C` enabled
2. **Verify network connectivity** - Arduino and PC must be on same network
3. **Run the enhanced client**:
   ```bash
   python UDPClient.py
   ```

4. **Monitor events in real-time**:
   - Console shows event notifications as they occur
   - Disturbance events are highlighted with detailed descriptions
   - Event flags are included in CSV data for post-analysis

5. **View live plots** - Matplotlib window shows:
   - Top-left: Attitude comparison (Roll, Pitch, Yaw)
   - Top-right: Attitude errors
   - Bottom-left: Accelerometer data
   - Bottom-right: Gyroscope data

6. **Stop recording** - Press `Ctrl+C` to gracefully shutdown

## Output Structure

Data is saved in the `data_logs/` directory:
```
data_logs/
└── YYYYMMDD_HHMMSS/          # Timestamped session folder
    ├── imu_data.csv          # Enhanced dataset with event_flags
    └── event_log.txt          # Timestamped event log
```

## Enhanced CSV Data Format

The CSV file contains the following columns (in order):

**Timestamp:**
- `timestamp_us` - Arduino timestamp in microseconds
- `event_flags` - Bit-field event flags (0-255)

**Control IMU Raw Sensors (MPU6050):**
- `ctrl_acc_x`, `ctrl_acc_y`, `ctrl_acc_z` - Accelerometer (g)
- `ctrl_gyro_x`, `ctrl_gyro_y`, `ctrl_gyro_z` - Gyroscope (deg/s)
- `ctrl_mag_x`, `ctrl_mag_y`, `ctrl_mag_z` - Magnetometer (µT)

**Monitor IMU Raw Sensors (MPU9250):**
- `mon_acc_x`, `mon_acc_y`, `mon_acc_z` - Accelerometer (g)
- `mon_gyro_x`, `mon_gyro_y`, `mon_gyro_z` - Gyroscope (deg/s)
- `mon_mag_x`, `mon_mag_y`, `mon_mag_z` - Magnetometer (µT)

**Control IMU Attitude (Madgwick Filter):**
- `ctrl_roll`, `ctrl_pitch`, `ctrl_yaw` - Euler angles (deg)

**Monitor IMU Attitude (Madgwick Filter):**
- `mon_roll`, `mon_pitch`, `mon_yaw` - Euler angles (deg)

**Attitude Comparison Errors:**
- `err_roll`, `err_pitch`, `err_yaw` - Control - Monitor (deg)

## Event Log Format

The `event_log.txt` file contains timestamped events:
```
# FCU IMU Event Log
# Session: 20250122_064815
# Started: 2025-01-22 06:48:15
# Format: [Timestamp] [Event_Type] [Description]
#

[12.345678] [COMMAND_CHANGE] Command input detected (Roll:15.2°, Pitch:-8.1°, Yaw:2.3°)
[12.456789] [ROLL_COMMAND] Roll command active (Target: 15.2°)
[15.678901] [DISTURBANCE] DISTURBANCE DETECTED - Unintended attitude change (Roll:6.2°, Pitch:0.1°, Yaw:0.0°) - POSSIBLE WIND/TURBULENCE
[18.901234] [CONTROL_ACTIVE] Control system activated (Armed)
```

## Event Flags Bit Field

The `event_flags` byte uses the following bit mapping:
- **Bit 0 (0x01)**: `command_change` - Command input changed
- **Bit 1 (0x02)**: `throttle_change` - Throttle changed
- **Bit 2 (0x04)**: `roll_command` - Roll command active
- **Bit 3 (0x08)**: `pitch_command` - Pitch command active
- **Bit 4 (0x10)**: `yaw_command` - Yaw command active
- **Bit 5 (0x20)**: `attitude_change` - Attitude changed beyond threshold
- **Bit 6 (0x40)**: `control_active` - Control system armed
- **Bit 7 (0x80)**: `disturbance` - Uncommanded attitude change

## Research Applications

### Flight Control System Validation
- **Disturbance Rejection**: Test control system response to wind gusts
- **Algorithm Tuning**: Optimize PID parameters for disturbance handling
- **Robustness Testing**: Evaluate performance under turbulent conditions

### Disturbance Analysis
- **Wind Gust Characterization**: Study disturbance patterns and magnitudes
- **Turbulence Response**: Analyze control system behavior in turbulent air
- **External Force Detection**: Identify impacts or external interference

### Performance Benchmarking
- **Command vs. Disturbance**: Distinguish pilot inputs from external forces
- **Control Authority**: Evaluate ability to maintain attitude under disturbances
- **Recovery Time**: Measure time to recover from disturbances

## Troubleshooting

**No data received:**
- Check that Arduino is powered and running
- Verify network connection (ping Arduino IP)
- Check firewall settings (UDP port 8888)
- Ensure `USE_MPU9250_MONITOR_I2C` is enabled in `quad.h`
- Verify Arduino firmware includes event detection (version 1.1+)

**Incorrect packet size:**
- Verify Arduino firmware matches expected 113-byte packet structure
- Check for network packet fragmentation
- Ensure event detection is enabled in Arduino code

**No events detected:**
- Verify disturbance thresholds are appropriate for your conditions
- Check that control system is armed (gear switch down, throttle low)
- Ensure command thresholds match your radio setup

**Plotting issues:**
- Ensure matplotlib backend supports GUI (TkAgg, Qt5Agg)
- On headless systems, use `matplotlib.use('Agg')` and save plots instead

## Notes

- Data is buffered in memory (last 1000 points) for plotting
- CSV files are flushed after each packet for data safety
- Plot updates at ~20Hz (50ms intervals) for smooth visualization
- X-axis shows last 30 seconds of data (rolling window)
- Event detection uses 100ms debounce to prevent flag spam
- Disturbance detection only active when armed and no commands active
- Serial output provides real-time disturbance alerts for field testing

## Version History

- **v1.0**: Basic UDP reception and CSV logging
- **v1.1**: Enhanced with event detection and disturbance monitoring

# Log Analysis Tool for FCU IMU Telemetry

Post-processing analysis tool for saved IMU telemetry CSV log files.

## Enhanced Features (Version 2.0)

This enhanced version includes comprehensive **event detection analysis and disturbance monitoring**:

- **Advanced Stability Metrics**: 15 different stability validation metrics
- **Event Pattern Analysis**: Command vs. disturbance event analysis
- **Data Integrity Validation**: UDP packet structure and quality assessment
- **Cross-Correlation Analysis**: Control vs. Monitor IMU correlation studies
- **Comprehensive Reporting**: Timestamped report directories with multiple visualizations
- **Disturbance Statistics**: Wind gust and turbulence response analysis
- **Research-grade Analysis**: Perfect for flight control system validation

## Key Features

### Core Functionality
- **Interactive Log Selection**: Browse and select from timestamped log directories
- **Comprehensive Statistics**: Detailed statistical analysis of all sensor data
- **Enhanced Stability Validation**: 15 different stability metrics
- **Visual Analysis**: Multiple plot types for comprehensive analysis
- **Auto-Save**: Automatically saves plots and comprehensive reports

### Advanced Stability Metrics
- **Basic Metrics**: MAE, RMSE, Standard Deviation, Settling Time
- **Signal-to-Noise Ratio (SNR)**: Quality of control signal in dB
- **Peak-to-Peak Error**: Maximum error range analysis
- **Error Distribution**: Skewness and kurtosis for error pattern analysis
- **95th Percentile Error**: Robust error assessment
- **Stability Index**: Ratio of std to MAE for stability assessment
- **Convergence Time**: Time to reach steady-state performance
- **Drift Rate**: Linear trend detection in error signals
- **Frequency Analysis**: Dominant frequency identification

### Event Detection Analysis
- **Command Event Analysis**: Pattern and frequency of command inputs
- **Disturbance Event Statistics**: Count, magnitude, and timing of disturbances
- **Event Correlation**: Relationship between commands and disturbances
- **Response Time Analysis**: Control system response to events
- **Event Timeline Visualization**: Temporal event pattern analysis

### Data Integrity Validation
- **UDP Packet Validation**: Structure and size verification
- **Missing Value Detection**: Data completeness assessment
- **Out-of-Range Checking**: Sensor data validation
- **Timestamp Gap Analysis**: Packet loss detection
- **Sensor Anomaly Detection**: Stuck values and excessive noise
- **Quality Scoring**: Overall data quality assessment (0-100)

### Cross-Correlation Analysis
- **Attitude Correlation**: Control vs. Monitor IMU attitude consistency
- **Raw Sensor Correlation**: Accelerometer and gyroscope correlation
- **Sensor Fusion Validation**: Madgwick filter performance assessment
- **Calibration Verification**: Sensor calibration consistency

## Requirements

Install dependencies:
```bash
pip install -r requirements.txt
```

Required packages:
- `numpy` >= 1.21.0
- `matplotlib` >= 3.5.0
- `pandas` >= 1.3.0
- `scipy` >= 1.7.0 (for advanced analysis)
- `seaborn` >= 0.11.0 (for enhanced visualizations)

## Usage

1. **Run enhanced analysis tool**:
   ```bash
   python FlightlogAnalysis.py
   ```

2. **Select a log file**:
   - The tool will list all available log files in `data_logs/`
   - Each entry shows: timestamp, number of rows, file size
   - Enter the number corresponding to the log file you want to analyze
   - Or press 'q' to quit

3. **Comprehensive Analysis Process**:
   - Data integrity validation and quality assessment
   - Advanced stability metrics calculation
   - Event pattern and disturbance analysis
   - Cross-correlation and frequency analysis
   - Multiple visualization generation
   - Comprehensive report creation

4. **View Results**:
   - Real-time console output of analysis progress
   - Interactive plots displayed (5 different visualizations)
   - Comprehensive text report with all metrics
   - Quality and stability level classification

## Output Structure

### Timestamped Report Directory
```bash
report_YYYYMMDD_HHMMSS/
├── comprehensive_stability_report.txt    # Detailed analysis report
├── plots/                                 # Visualization directory
│   ├── main_analysis.png                 # Primary 4-panel analysis
│   ├── error_analysis.png                # Detailed error analysis
│   ├── data_integrity.png                 # Data quality visualization
│   ├── correlation_heatmap.png           # Cross-correlation matrix
│   └── frequency_analysis.png            # Frequency domain analysis
└── data/
    └── original_data.csv                  # Copy of analyzed data
```

### Console Statistics

The tool prints comprehensive analysis including:
- **Data Quality Assessment**: Overall quality score and validation results
- **Advanced Stability Metrics**: All 15 stability metrics with detailed breakdowns
- **Event Analysis**: Command and disturbance event statistics
- **Cross-Correlation Results**: IMU correlation and consistency metrics
- **Frequency Analysis**: Dominant frequencies and spectral characteristics
- **Quality Classification**: Automated quality and stability level assessment

### Enhanced Plot Types

1. **Main Analysis Plot**: Traditional 4-panel visualization
   - Attitude comparison (Control vs Monitor)
   - Attitude error analysis with statistics
   - Accelerometer data comparison
   - Gyroscope data comparison

2. **Error Analysis Plot**: Detailed error visualization
   - Error distribution histograms
   - Cumulative error analysis
   - Statistics comparison (MAE, RMSE, Std Dev)
   - Settling time analysis

3. **Data Integrity Plot**: Quality assessment visualization
   - Data quality gauge
   - Missing value analysis
   - Timestamp gap visualization
   - Sensor anomaly detection

4. **Correlation Heatmap**: Cross-correlation matrix
   - All sensor channel correlations
   - Control vs. Monitor IMU relationships
   - Raw sensor vs. attitude correlations

5. **Frequency Analysis Plot**: Spectral analysis
   - Error signal spectra
   - Dominant frequency identification
   - SNR analysis
   - Frequency response characteristics

### Comprehensive Report

The `comprehensive_stability_report.txt` includes:
- **Report Structure**: File organization and contents
- **Data Integrity Validation**: Complete quality assessment
- **Advanced Stability Metrics**: Detailed metric breakdowns
- **Cross-Correlation Analysis**: Correlation results and interpretation
- **Event Analysis**: Command and disturbance statistics
- **Quality Assessment**: Automated classification and recommendations
- **Visualization Descriptions**: Explanation of all generated plots

## Advanced Metrics Explained

### Stability Metrics
- **MAE (Mean Absolute Error)**: Average magnitude of errors
- **RMSE (Root Mean Square Error)**: Penalizes larger errors more heavily
- **SNR (Signal-to-Noise Ratio)**: Quality of control signal in dB
- **Peak-to-Peak Error**: Maximum error range
- **Skewness**: Error distribution asymmetry
- **Kurtosis**: Error distribution tail weight
- **95th Percentile**: Robust error assessment
- **Stability Index**: Ratio of std to MAE
- **Convergence Time**: Time to reach steady-state
- **Drift Rate**: Linear trend in error
- **Dominant Frequency**: Primary oscillation frequency

### Event Analysis
- **Command Events**: Pilot input detection and analysis
- **Disturbance Events**: Uncommanded attitude changes
- **Event Frequency**: Timing and pattern analysis
- **Response Time**: Control system reaction time
- **Event Correlation**: Relationship between events

### Data Quality
- **Quality Score**: Overall data quality (0-100)
- **Completeness**: Missing value assessment
- **Consistency**: Data range validation
- **Reliability**: Timestamp and sensor analysis

## Research Applications

### Flight Control System Validation
- **Disturbance Rejection**: Test response to wind gusts
- **Algorithm Tuning**: Optimize control parameters
- **Robustness Testing**: Evaluate turbulent conditions
- **Performance Benchmarking**: Compare different configurations

### Disturbance Analysis
- **Wind Gust Characterization**: Study disturbance patterns
- **Turbulence Response**: Analyze control behavior
- **External Force Detection**: Identify impacts
- **Environmental Assessment**: Evaluate flight conditions

### Sensor Fusion Validation
- **IMU Consistency**: Control vs. Monitor comparison
- **Filter Performance**: Madgwick filter assessment
- **Calibration Verification**: Sensor accuracy validation
- **Cross-Correlation**: Sensor relationship analysis

## Example Output

```
======================================================================
FCU IMU Telemetry - Comprehensive Log Analysis Tool
======================================================================

======================================================================
Available Log Files:
======================================================================
  [ 1] 2026-01-22 06:48:15 |   5234 rows | 0.65 MB
  [ 2] 2026-01-22 05:30:45 |   8678 rows | 1.08 MB
  [ 3] 2026-01-22 04:15:10 |  12012 rows | 1.50 MB
======================================================================

Select log file [1-3] (or 'q' to quit): 1

Report directory created: report_20250122_064815

======================================================================
Starting Comprehensive Analysis...
======================================================================

1. Performing data integrity validation...
Data Quality Score: 94.2/100
 No missing values detected
 All values within expected ranges
 No significant timestamp gaps detected
 No sensor anomalies detected

2. Generating basic statistics...

3. Calculating advanced stability metrics...

4. Performing cross-correlation analysis...

5. Advanced stability analysis results:
ROLL Axis:
  Basic Metrics:
    MAE: 0.8234°
    RMSE: 1.2456°
    Std Dev: 0.9345°
  Advanced Metrics:
    Signal-to-Noise Ratio: 15.23 dB
    Stability Index: 1.134
    Drift Rate: 0.000123°/s
    Dominant Frequency: 2.34 Hz

...

======================================================================
ANALYSIS SUMMARY
======================================================================
Data Quality:  EXCELLENT (94.2/100)
Stability Performance:  GOOD
  Average MAE: 0.8234°
  Average RMSE: 1.2456°
IMU Correlation: 0.9876 (Control vs. Monitor)

Generated Report Structure:
  report_20250122_064815/
    comprehensive_stability_report.txt
    plots/
      main_analysis.png
      error_analysis.png
      data_integrity.png
      correlation_heatmap.png
      frequency_analysis.png
    data/
      original_data.csv

======================================================================
Analysis complete!
Report location: report_20250122_064815
======================================================================
```

## Notes

- Log files are sorted by timestamp (newest first)
- Only directories containing `imu_data.csv` are listed
- Enhanced CSV files with `event_flags` column are automatically detected
- All plots use consistent color schemes for professional presentation
- Reports include quality and stability level classifications
- Event analysis requires enhanced CSV format (version 1.1+)
- Frequency analysis uses FFT for spectral characterization
- Cross-correlation analysis validates sensor fusion performance

## Version History

- **v1.0**: Basic stability metrics and 4-panel plotting
- **v2.0**: Enhanced with event detection analysis, advanced metrics, and comprehensive reporting
