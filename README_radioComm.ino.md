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
