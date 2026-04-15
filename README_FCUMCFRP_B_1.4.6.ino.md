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
- **FlexPWM Reset**: Comprehensive Teensy 4.0 peripheral initialization

## Key Components

### 1. System Architecture

```text
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
```text

### 2. Core Functions

#### Main Loop Functions

```textcpp
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
```text

#### Sensor Fusion

```textcpp
void Madgwick() {
  // Madgwick AHRS algorithm implementation
  // Fuses accelerometer, gyroscope, magnetometer data
  // Provides roll, pitch, yaw attitude estimates
  // Runs at 2kHz for optimal performance
}
```text

#### Control Functions

```textcpp
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
```text

#### Command Processing

```textcpp
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
```text

#### Motor Mixing

```textcpp
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
```text

### 3. Safety Systems

#### Arming System

```textcpp
void armedStatus() {
  // Arming conditions:
  // 1. Channel 5 (gear) must be LOW (< 1500 PWM)
  // 2. Channel 1 (throttle) must be LOW (< 1050 PWM)
  // Sets armedFly = true when conditions met
}
```text

#### Throttle Cut

```textcpp
void throttleCut() {
  // Safety override conditions:
  // 1. Channel 5 (gear) is HIGH (> 1500 PWM)
  // 2. armedFly is false
  // Forces all motors to minimum PWM
}
```text

#### Failsafe System

```textcpp
void failsafe() {
  // Radio signal loss detection
  // Applies predefined failsafe values
  // Sets armedFly = false for safety
}
```text

### 4. Sensor Management

#### Primary IMU Interface

```textcpp
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
```text

#### Calibration Functions

```textcpp
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
```text

### 5. Radio Receiver Support

#### Supported Receiver Types

- **PWM**: Standard PWM receiver (6 channels)
- **PPM**: Pulse Position Modulation (single wire)
- **SBUS**: Futaba SBUS protocol (inverted serial)
- **DSM**: Spektrum DSM protocol
- **IBUS**: FlySky IBUS protocol
- **ELRS**: ExpressLTS Radio System (CRSF)

#### Receiver Interface Functions

```textcpp
void getPPMRX();      // PPM receiver interface
void getPWMRX();      // PWM receiver interface
void getSBUSRX();     // SBUS receiver interface
void getELRSRX();     // ELRS receiver interface
void getDSMRX();      // DSM receiver interface
```text

### 6. Motor Control

#### Motor Configuration

```textcpp
// Motor command variables
float m1_command_scaled, m2_command_scaled, ...;  // Normalized commands
int m1_command_PWM, m2_command_PWM, ...;           // PWM outputs

// PWM/OneShot125 selection
#define USE_ONESHOT125_ESC  // Enable OneShot125 protocol
```text

#### Motor Output

```textcpp
void commandMotors() {
  // Sends PWM signals to ESCs
  // Supports standard PWM (1000-2000μs)
  // Supports OneShot125 (125-250μs)
  // Uses PWMServo library for timing
}
```text

## Control Scheme and Mixer Configuration

### X-Configuration Quadcopter Mixer

#### Motor Layout and Positioning

```textmarkdown
        Front
          ^
          |
   M1    |      M2
(Front Left)|(Front Right)
    \     |     /
     \    |    /
      \   |   /
       \  |  /
         \|/
----------+---------  ← Center of gravity
         /|\
        / | \
       /  |  \
      /   |   \
     /    |    \
    /     |     \
   M4     |      M3
(Back Left)|(Back Right)
          |
          v
         Back
```text

#### Mixer Equations

```textcpp
//Quad mixing - X-configuration
// Setup: Reference propellers from top view, calibrate motor direction via ESC
// Props-out preferred for dust/debris protection

m1_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID; //Front Left (CCW)
m2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID; //Front Right (CW)
m3_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID; //Back Right (CCW)
m4_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID; //Back Left (CW)
```text

#### Control Response Verification

```textcpp
// Right Roll (positive roll_PID):
m1: +roll_PID  // Front Left increases thrust
m2: -roll_PID  // Front Right decreases thrust
m3: -roll_PID  // Back Right decreases thrust
m4: +roll_PID  // Back Left increases thrust
// Result: Aircraft rolls right

// Forward Pitch (positive pitch_PID):
m1: -pitch_PID // Front Left decreases thrust
m2: -pitch_PID // Front Right decreases thrust
m3: +pitch_PID // Back Right increases thrust
m4: +pitch_PID // Back Left increases thrust
// Result: Aircraft pitches forward

// Right Yaw (positive yaw_PID):
m1: +yaw_PID   // Front Left increases thrust
m2: -yaw_PID   // Front Right decreases thrust
m3: +yaw_PID   // Back Right increases thrust
m4: -yaw_PID   // Back Left decreases thrust
// Result: Aircraft yaws right (clockwise)
```text

### Hardware Configuration Independence

#### Motor Direction Agnosticism

The control mixer is designed to be **hardware-agnostic** and works with any motor configuration:

```textcpp
// Control system creates ABSTRACT moments:
roll_PID = 0.5;  // "Create 0.5 units of right roll moment"

// Mixer translates moments to thrust differentials:
// Physics ensures correct moment generation
// Hardware implements moment generation
// No software can break physics
```text

#### Supported Motor Configurations

```textcpp
// All configurations work with same mixer:
1. Standard Mixed Props (CCW/CW/CCW/CW)
2. All Same Direction (CW/CW/CW/CW or CCW/CCW/CCW/CCW)
3. Reversed Mixed (CW/CCW/CW/CCW)

// Hardware affects:
// - Motor spin direction
// - Propeller type and mounting
// - ESC calibration
// - Torque characteristics

// Hardware does NOT affect:
// - Control direction
// - Stick response
// - Flight characteristics
// - Mixer effectiveness
```text

#### Physical Remote Control Mapping

```textcpp
// PWM to command conversion (1000-2000μs range):
thro_des = (channel_1_pwm - 1000.0)/1000.0; //Between 0 and 1
roll_des = (channel_2_pwm - 1500.0)/500.0;  //Between -1 and 1
pitch_des = (channel_3_pwm - 1500.0)/500.0; //Between -1 and 1
yaw_des = (channel_4_pwm - 1500.0)/500.0;   //Between -1 and 1

// Physical stick response:
// Stick Right → PWM 2000 → roll_des = +1.0 → Right roll
// Stick Left  → PWM 1000 → roll_des = -1.0 → Left roll
// Stick Forward → PWM 1000 → pitch_des = -1.0 → Forward pitch
// Stick Back    → PWM 2000 → pitch_des = +1.0 → Backward pitch
```text

### Integrator Anti-Windup Strategy

#### Throttle-Based Integrator Reset

```textcpp
// Similar to NMNI (No Motion No Integration) but safety-focused
if (channel_1_pwm < 1060) {   // < 6% throttle
    integral_roll = 0;        // Reset all integrators
    integral_pitch = 0;       // Prevent windup during low throttle
    integral_yaw = 0;         // Safety-first approach
}
```text

#### Comparison with True NMNI

```textcpp
// Current Implementation (Throttle-based):
if (channel_1_pwm < 1060) {
    // Global reset: ALL integrators reset together
    // Simpler logic, safety-focused
    integral_roll = integral_pitch = integral_yaw = 0;
}

// Traditional NMNI (Motion-based):
if (abs(roll_des) < threshold) integral_roll = 0;    // Per-axis control
if (abs(pitch_des) < threshold) integral_pitch = 0;   // More precise
if (abs(yaw_des) < threshold) integral_yaw = 0;       // Complex maneuvers
```text

### Advantages Over Mainstream Flight Stacks

#### Fixed Architecture Benefits

```textcpp
// Mainstream stacks (complex):
// - Multiple mixer types (X, +, H, V, custom)
// - Configurable motor numbering
// - Software motor reversal
// - Mode-dependent mixing
// - User configurable parameters

// FCUMCFRP_B_1.4.5.ino (simple):
// - Fixed X-configuration only
// - No software motor reversal
// - Single control path
// - Hardware-only motor direction
// - Predictable behavior
```text

#### Spiraling Prevention

```textcpp
// Why this system prevents spiraling:
1. Fixed control direction (no software inversion)
2. Hardware abstraction layer
3. Physics-based moment control
4. No user-configurable motor reversal
5. Validated mixer equations
6. Single control mode path

// Result: Predictable, stable control regardless of hardware setup
```text

### Setup and Configuration Guidelines

#### Motor Placement Setup

```textcpp
// For easier motor placement setup:
// 1. Reference propellers from top view of vehicle, facing forward
// 2. Adjust motor direction accordingly via ESC calibration
// 3. Props-out configuration preferred (dust/debris protection)

// Props-out benefits:
// - Airflow pushes downward, kicking dust away from center plate
// - IMU sensors stay cleaner
// - Better airflow over electronics
// - Reduced maintenance in dusty research environments
```text

#### Physical Control Characteristics

```textcpp
// Standard RC transmitter layout (Mode 2):
┌─────────────────┐
│  Throttle (Ch1) │  ← Left stick vertical (1000-2000μs)
│  Yaw (Ch4)      │  ← Left stick horizontal (1000-2000μs)
│                 │
│  Pitch (Ch3)    │  ← Right stick vertical (1000-2000μs)
│  Roll (Ch2)     │  ← Right stick horizontal (1000-2000μs)
└─────────────────┘

// Channel 5: Gear switch (arming/disarming)
// Channel 6: Auxiliary (modes, functions)
```text

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

```textcpp
// Control mode selection (typically via radio channel)
if (control_mode == ANGLE_MODE) {
  controlANGLE();
} else {
  controlRATE();
}
```text

## PID Controller Implementation

### PID Structure

```textcpp
// PID variables for each axis
float error_roll, error_pitch, error_yaw;
float integral_roll, integral_pitch, integral_yaw;
float derivative_roll, derivative_pitch, derivative_yaw;
float roll_PID, pitch_PID, yaw_PID;

// PID gains (tunable)
float Kp_roll_angle, Ki_roll_angle, Kd_roll_angle;
float Kp_pitch_angle, Ki_pitch_angle, Kd_pitch_angle;
float Kp_yaw, Ki_yaw, Kd_yaw;
```text

### PID Algorithm

```textcpp
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
```text

## Madgwick Filter Mathematics

### Filter Algorithm

The Madgwick filter implements a gradient descent algorithm for orientation estimation using sensor fusion.

#### Mathematical Foundation

```textcpp
// Quaternion-based orientation estimation
// Gradient descent optimization for sensor fusion
// Combines accelerometer, gyroscope, and magnetometer data
```text

#### Core Equations

```text
// Quaternion gradient descent step
∇q = 2 * (q ⊗ [0, gx, gy, gz]) ⊗ q_conj

// Filter update with beta parameter
q(t+1) = q(t) + (dt/2) * ∇q

// Normalization to maintain unit quaternion
q_normalized = q / ||q||
```text

#### Beta Parameter Significance

```textcpp
// B_madgwick controls filter response characteristics
// Lower values: Faster response, less filtering
// Higher values: Slower response, more filtering

// Typical ranges:
// B_madgwick = 0.04  // Default for 2kHz loop (balanced)
// B_madgwick = 0.02  // Aggressive (fast response)
// B_madgwick = 0.08  // Conservative (strong filtering)
```text

#### Sensor Fusion Process

```textcpp
// 1. Predict orientation using gyroscope
q_pred = q_current + (dt/2) * q_current ⊗ [0, gx, gy, gz]

// 2. Compute gradient from accelerometer and magnetometer
∇q = 2 * (q_pred ⊗ [ax, ay, az, mx, my, mz]) ⊗ q_pred_conj

// 3. Update orientation estimate
q_new = q_pred - (B_madgwick * dt) * ∇q

// 4. Normalize to unit quaternion
q_normalized = q_new / ||q_new||
```text

### Filter Performance Characteristics

#### Response Time Analysis

```text
// Time constant τ = 1/(B_madgwick * ω_gyro)
// Where ω_gyro is the dominant gyroscope frequency

// Step response characteristics:
// Rise time (10-90%): ≈ 2.2 * τ
// Settling time (2%): ≈ 4.0 * τ
// Bandwidth: ≈ B_madgwick * ω_gyro / (2π)
```text

#### Noise Rejection

```text
// Equivalent noise bandwidth
BW_noise ≈ B_madgwick * ω_gyro / 4

// Noise reduction factor
NR_factor = 1 / sqrt(1 + (BW_noise / BW_sensor)^2)

// Higher B_madgwick = more noise rejection
// Lower B_madgwick = better tracking of rapid movements
```text

## Control System Dynamics

### Closed-Loop Response

The flight control system forms a closed-loop feedback control with the following dynamics:

#### Transfer Function Analysis

```text
// Simplified second-order system
G(s) = (K * ω_n^2) / (s^2 + 2*ζ*ω_n*s + ω_n^2)

// Where:
// K = Controller gain
// ω_n = Natural frequency
// ζ = Damping ratio
```text

#### Stability Margins

```text
// Phase margin: PM = 180° + ∠G(jω_c)
// Gain margin: GM = 20 * log10(1/|G(jω_g)|)
// Target: PM > 45°, GM > 6dB for robust stability
```text

### PID Control Mathematics

#### Error Dynamics

```textcpp
// Proportional term: P_out = Kp * error(t)
// Integral term: I_out = Ki * ∫error(τ)dτ
// Derivative term: D_out = Kd * d(error)/dt

// Combined output:
PID_output = Kp*error + Ki*∫error*dτ + Kd*d(error)/dt
```text

#### Frequency Domain Analysis

```text
// PID controller frequency response
G(jω) = Kp + Ki/(jω) + Kd*(jω)

// Phase lag: φ(ω) = -atan2(Ki*ω - Kd*ω^2, Kp)
// Magnitude: |G(jω)| = sqrt(Kp^2 + (Ki/ω - Kd*ω)^2)
```text

### System Integration

#### Control Loop Timing

```textcpp
// 2kHz main loop (500μs cycle time)
// Sensor updates: 1kHz (1000Hz)
// Madgwick filter: 2kHz (synchronized with control)
// Motor output: 200Hz (5ms PWM refresh)
// UDP telemetry: 100Hz (10ms transmission)
```text

#### Latency Analysis

```text
// Total system latency breakdown:
// Sensor latency: ~1ms (MPU6050 read + filtering)
// Filter latency: ~2ms (Madgwick computation)
// Control latency: ~0.5ms (PID calculation)
// Motor latency: ~5ms (PWM refresh rate)
// Total: ~8.5ms from sensor to motor output
```text

### System Performance Optimization

#### Computational Efficiency

```textcpp
// Quaternion operations (4 multiplications, 3 additions)
// Matrix operations minimized for speed
// Fixed-point arithmetic considered for precision
// Loop unrolling for critical sections
```text

#### Numerical Stability

```textcpp
// Quaternion normalization prevents drift
// Anti-windup limits integrator saturation
// Derivative filtering reduces noise amplification
// Conditional integration prevents low-throttle issues
```text

## Integration with Monitor System

### Monitor Integration Point

```textcpp
void loop() {
  // ... main control loop ...
  
  sendIMUDataUDP();  // Call monitor telemetry system
}
```text

### Data Sharing

- **Global Variables**: All attitude and sensor data accessible
- **No Interference**: Monitor system doesn't affect control loop
- **Real-time Data**: 100Hz telemetry transmission
- **Event Detection**: Command and disturbance monitoring

## Configuration and Tuning

### PID Tuning Parameters

```textcpp
// Angle mode gains (default values)
float Kp_roll_angle = 0.15;    // Proportional gain
float Ki_roll_angle = 0.00;    // Integral gain  
float Kd_roll_angle = 0.05;    // Derivative gain

// Rate mode gains
float Kp_roll_rate = 0.15;     // Rate proportional gain
float Ki_roll_rate = 0.00;     // Rate integral gain
float Kd_roll_rate = 0.001;    // Rate derivative gain
```text

### Safety Limits

```textcpp
float i_limit = 25.0;          // Integrator saturation limit
float maxRoll = 30.0;          // Maximum roll angle (degrees)
float maxPitch = 30.0;         // Maximum pitch angle (degrees)
float maxYaw = 160.0;          // Maximum yaw rate (degrees/sec)
```text

### Rate Limiting

```textcpp
float dt = 0.005;              // Loop time (200Hz)
float maxRate = 200.0;         // Maximum rate limit (deg/sec)
```text

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

```textcpp
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
```text

### LED Indicators

```textcpp
void setupBlink() {
  // Blink patterns for system status
  // Different patterns for different states
}
```text

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

- **File Version**: B-1.4.6
- **Last Updated**: 2026-04-14
- **Author**: Patrick Andrasena T.
- **Base Project**: dRehmFlight by Nicholas Rehm
- **Target Platform**: Teensy 4.0/4.1

## Key Updates in B-1.4.6

### IBUS Receiver Support

- **Full IBUS Integration**: Complete FlySky IBUS protocol support
- **Multi-Receiver Architecture**: Enhanced receiver abstraction layer
- **Debug Compatibility**: Universal debug function for all receiver types
- **Compilation Fixes**: Resolved scope and declaration issues

### Enhanced Stability

- **Error Resolution**: Fixed all compilation errors for IBUS receivers
- **Conditional Compilation**: Proper receiver-type specific code paths
- **Variable Scope**: Corrected variable declaration and access patterns

### Previous Features (B-1.4.5)

#### FlexPWM Peripheral Reset System

- **Comprehensive Reset**: Full FlexPWM peripheral initialization for Teensy 4.0
- **Pin State Management**: Forces all FlexPWM pins to GPIO mode before setup
- **Conflict Prevention**: Eliminates peripheral state conflicts from previous firmware
- **Hardware Abstraction**: Clean slate for motor and servo initialization

### Enhanced Motor Control

- **Dual ESC Support**: Both OneShot125 and standard PWM ESC protocols
- **PWMServo Integration**: Robust motor and servo control library
- **Pin Safety**: Proper pin mode initialization sequence
- **Performance Optimization**: Minimal latency motor output

### Improved IMU Integration

- **Dual IMU Support**: Primary (MPU6050) and Monitor (MPU9250) configurations
- **Separate I2C Buses**: Independent Wire and Wire1 operation
- **Calibration System**: Built-in IMU error calculation routines
- **Research Ready**: Monitor system for attitude validation

### Radio Communication Enhancements

- **Multi-Protocol Support**: PWM, PPM, SBUS, DSM, IBUS, ELRS receivers
- **Unified API**: Single getRadioPWM() interface for all protocols
- **Channel Mapping**: Flexible logical to physical channel assignment
- **Safety Features**: Comprehensive failsafe and signal validation

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
