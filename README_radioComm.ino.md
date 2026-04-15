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
- **PPM Bug Fixes**: Comprehensive PPM decoding reliability improvements

## Key Updates in B-1.4.7

### PPM System Reliability Fixes

- **FIX 1**: Removed channel_N_raw = 0 wipe on sync detection
- **FIX 2**: Corrected sync detection threshold to >2500 µs
- **FIX 3**: Removed all Serial.print() from ISR for performance
- **FIX 4**: channel_N_raw and ppm_isr_count declared volatile
- **FIX 5**: togglePPMDebug() removed from main loop
- **FIX 6**: Removed no-op map() call in ISR
- **FIX 7**: ppm_counter reset on first sync as well as subsequent ones
- **FIX 8**: PPM pin set to plain INPUT (not INPUT_PULLUP)
- **FIX 9**: ppm_counter declared volatile
- **FIX 10**: getCh1-6 guard changed to USE_PWM_RX only

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

```cpp
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
```cpp

## Core Functions

### 1. Initialization Function

```cppcpp
void radioSetup() {
  // Configures selected receiver type
  // Initializes hardware interfaces
  // Sets up interrupt handlers
  // Configures serial ports
  // Applies pin modes and pull-ups
}
```cpp

#### Receiver-Specific Initialization

**PWM Receiver Setup:**

```cppcpp
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
```cpp

**PPM Receiver Setup:**

```cppcpp
#if defined USE_PPM_RX
  // Configure single PPM pin
  pinMode(PPM_Pin, INPUT_PULLUP);
  
  // Attach PPM interrupt handler
  attachInterrupt(digitalPinToInterrupt(PPM_Pin), getPPM, CHANGE);
#endif
```cpp

**Serial Receiver Setup:**

```cppcpp
#if defined USE_SBUS_RX
  sbus.begin();                    // Initialize SBUS library
#elif defined USE_ELRS_RX
  elrs_rx.begin();                 // Initialize ELRS library
#elif defined USE_DSM_RX
  Serial3.begin(115000);           // Initialize DSM serial
#endif
```cpp

### 2. Unified Channel Access

```cppcpp
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
```cpp

## Interrupt Service Routines

### 1. PWM Interrupt Handlers

**Channel-Specific ISR:**

```cppcpp
void getCh1() {
  int trigger = digitalRead(ch1Pin);
  if(trigger == 1) {
    rising_edge_start_1 = micros();    // Record rising edge time
  }
  else if(trigger == 0) {
    channel_1_raw = micros() - rising_edge_start_1;  // Calculate pulse width
  }
}
```cpp

**PWM Timing Logic:**

- **Rising Edge**: Record timestamp
- **Falling Edge**: Calculate pulse width
- **Pulse Width**: 1000-2000μs range
- **Update Rate**: ~50Hz per channel
- **Accuracy**: ±1μs timing resolution

### 2. PPM Interrupt Handler

```cppcpp
void getPPM() {
  ppm_isr_count++;

  static unsigned long last_rising_edge = 0;
  static int last_pin_state = LOW;

  unsigned long current_time = micros();
  int current_pin_state = digitalRead(PPM_Pin);

  if (current_pin_state == HIGH && last_pin_state == LOW) {
    unsigned long pulse_position = current_time - last_rising_edge;

    // Skip first edge after boot
    if (last_rising_edge == 0) {
      last_rising_edge = current_time;
      last_pin_state = current_pin_state;
      return;
    }

    #ifndef PPM_SYNC_MIN_US
      #define PPM_SYNC_MIN_US 2500UL
    #endif
    #ifndef PPM_CH_MIN_US
      #define PPM_CH_MIN_US   700UL
    #endif
    #ifndef PPM_CH_MAX_US
      #define PPM_CH_MAX_US   2300UL
    #endif

    if (pulse_position >= PPM_SYNC_MIN_US) {
      // New frame — reset counter, keep existing channel values intact
      ppm_counter = 0;

    } else if (pulse_position >= PPM_CH_MIN_US && pulse_position <= PPM_CH_MAX_US) {
      ppm_counter++;
      unsigned long ch_value = constrain(pulse_position, 1000UL, 2000UL);

      if      (ppm_counter == 1) channel_1_raw = ch_value;
      else if (ppm_counter == 2) channel_2_raw = ch_value;
      else if (ppm_counter == 3) channel_3_raw = ch_value;
      else if (ppm_counter == 4) channel_4_raw = ch_value;
      else if (ppm_counter == 5) channel_5_raw = ch_value;
      else if (ppm_counter == 6) channel_6_raw = ch_value;
    }

    last_rising_edge = current_time;
  }

  last_pin_state = current_pin_state;
}
```cpp

**PPM Frame Structure:**

```cpp
[Sync Pulse >2.5ms][CH1][CH2][CH3][CH4][CH5][CH6][CH7][CH8][Sync Pulse]
     1000-2000μs each channel
     Total frame: ~22.5ms (50Hz)
```cpp

**Reliability Improvements:**

- **Edge Detection**: Only processes rising edges for reliability
- **Sync Detection**: Fixed threshold at 2500µs (not 5000µs)
- **Channel Preservation**: Doesn't zero channels on sync detection
- **Volatile Variables**: All ISR-shared variables properly declared
- **No Serial in ISR**: Removed all debug prints from interrupt context

### 3. Serial Event Handlers

**DSM Serial Event:**

```cppcpp
void serialEvent3(void) {
  #if defined USE_DSM_RX
    while (Serial3.available()) {
      DSM.handleSerialEvent(Serial3.read(), micros());
    }
  #endif
}
```cpp

**Serial Processing:**

- **Buffer Management**: Handle incoming serial data
- **Frame Parsing**: Decode protocol-specific frames
- **Channel Extraction**: Extract PWM values from frames
- **Timestamp Recording**: Maintain timing accuracy

## Channel Mapping System

### Logical to Physical Mapping

**Configuration in quad.h:**

```cppcpp
// PPM Channel Mapping
#define PPM_MAP_CH1 3   // Logical CH1 → Physical slot 3
#define PPM_MAP_CH2 1   // Logical CH2 → Physical slot 1
#define PPM_MAP_CH3 2   // Logical CH3 → Physical slot 2
#define PPM_MAP_CH4 4   // Logical CH4 → Physical slot 4
#define PPM_MAP_CH5 5   // Logical CH5 → Physical slot 5
#define PPM_MAP_CH6 6   // Logical CH6 → Physical slot 6
```cpp

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
| PWM           | 50Hz        | ~20ms   | 1μs        | ±1μs      |
| PPM           | 50Hz        | ~20ms   | 1μs        | ±1μs      |
| SBUS          | 100Hz       | ~10ms   | 10μs       | ±10μs     |
| DSM           | 125Hz       | ~8ms    | 10μs       | ±10μs     |
| ELRS          | 250Hz       | ~4ms    | 10μs       | ±10μs     |
| IBUS          | 100Hz       | ~10ms   | 10μs       | ±10μs     |

### CPU Usage

| Receiver Type | CPU Usage | Memory Usage | Interrupt Load      |
|---------------|-----------|-------------|---------------------|
| PWM           | ~5%       | ~200B       | 6 interrupts/frame  |
| PPM           | ~2%       | ~100B       | 1 interrupt/frame   |
| SBUS          | ~3%       | ~500B       | Serial interrupts   |
| DSM           | ~3%       | ~500B       | Serial interrupts   |
| ELRS          | ~4%       | ~1KB        | Serial interrupts   |
| IBUS          | ~3%       | ~500B       | Serial interrupts   |

## Hardware Requirements

### Pin Assignments

**PWM/PPM Pins:**

```cppcpp
const int ch1Pin = 15;  // Throttle
const int ch2Pin = 16;  // Aileron
const int ch3Pin = 17;  // Elevator
const int ch4Pin = 20;  // Rudder
const int ch5Pin = 21;  // Gear
const int ch6Pin = 22;  // Aux1

const int PPM_Pin = 23; // PPM input
```cpp

**Serial Ports:**

```cppcpp
// Serial3: DSM/ELRS/IBUS
// RX3: Pin 15
// TX3: Pin 14
```cpp

**SBUS Pin:**

```cppcpp
// RX5: Pin 21 (inverted signal)
```cpp

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

```cppcpp
// Select exactly one receiver type
#define USE_ELRS_RX          // Modern high-performance
// #define USE_SBUS_RX       // Professional standard
// #define USE_DSM_RX        // Spektrum systems
// #define USE_PWM_RX        // Standard analog
// #define USE_PPM_RX        // Single-wire
// #define USE_IBUS_RX       // FlySky systems
```cpp

### 2. Channel Mapping

**Custom Channel Assignment:**

```cppcpp
// Example: Custom PPM mapping
#define PPM_MAP_CH1 1   // Throttle on slot 1
#define PPM_MAP_CH2 2   // Aileron on slot 2
#define PPM_MAP_CH3 3   // Elevator on slot 3
#define PPM_MAP_CH4 4   // Rudder on slot 4
#define PPM_MAP_CH5 5   // Gear on slot 5
#define PPM_MAP_CH6 6   // Aux1 on slot 6
```cpp

### 3. Pin Configuration

**Custom Pin Assignment:**

```cppcpp
// Example: Custom PWM pins
const int ch1Pin = 2;   // Throttle on pin 2
const int ch2Pin = 3;   // Aileron on pin 3
// ... etc
```cpp

## Failsafe and Safety

### Signal Loss Detection

**Timeout Monitoring:**

```cppcpp
// Each receiver type implements signal loss detection
// Automatic failsafe values applied on signal loss
// Default failsafe: 1500μs (center position)
```cpp

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

```cppcpp
// Add to main loop for debugging
void printRadioChannels() {
  Serial.print("CH1: "); Serial.print(getRadioPWM(1));
  Serial.print(" CH2: "); Serial.print(getRadioPWM(2));
  Serial.print(" CH3: "); Serial.print(getRadioPWM(3));
  Serial.print(" CH4: "); Serial.print(getRadioPWM(4));
  Serial.print(" CH5: "); Serial.print(getRadioPWM(5));
  Serial.print(" CH6: "); Serial.println(getRadioPWM(6));
}
```cpp

**Signal Quality Monitoring:**

```cppcpp
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
```cpp

## Integration Examples

### Basic Usage

```cppcpp
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
```cpp

### Advanced Usage with Validation

```cppcpp
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
```cpp

## Version Information

- **File Version**: B-1.4.8 (IBUS Integration)
- **Last Updated**: 2026-04-14
- **Author**: Nicholas Rehm (Base), Patrick Andrasena T. (Enhancements)
- **Base Project**: dRehmFlight
- **Target Platform**: Teensy 4.0/4.1

## Key Updates in B-1.4.8

### IBUS Integration Features

- **Complete IBUS Support**: Full FlySky IBUS protocol implementation
- **Multi-Receiver Debug**: Universal debugPPMSignal() function for all receiver types
- **Compilation Stability**: Resolved all scope and declaration issues
- **Extern Declarations**: Proper cross-file variable referencing

### Enhanced Debug Capabilities

- **Receiver-Aware Debugging**: Conditional debug output based on receiver type
- **Signal Quality Monitoring**: Different quality metrics for different receivers
- **Universal Compatibility**: Single debug function works with all receiver types

### Previous PPM Improvements (B-1.4.7)

#### PPM System Reliability Fixes

1. **Channel Data Preservation**: Fixed channel wiping on sync detection
2. **Sync Threshold**: Corrected from 5000µs to 2500µs for better compatibility
3. **ISR Performance**: Removed Serial.print() calls from interrupt context
4. **Memory Safety**: Proper volatile declarations for ISR-shared variables
5. **Debug Control**: Moved debug functions out of main loop
6. **Code Optimization**: Removed unnecessary map() operations
7. **Counter Reset**: Fixed ppm_counter initialization on first sync
8. **Input Configuration**: Changed to plain INPUT for better signal detection
9. **Variable Safety**: Added volatile to ppm_counter
10. **Compilation Guard**: Fixed PWM function compilation conflicts

### Performance Benefits

- **Reliability**: Eliminates PPM signal dropouts and false syncs
- **Performance**: Faster ISR execution without serial operations
- **Compatibility**: Works with wider range of PPM receivers
- **Stability**: Proper memory barrier handling for shared variables
- **Debugging**: Controlled debug output without impacting performance

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
