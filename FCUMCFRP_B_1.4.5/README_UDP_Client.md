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
