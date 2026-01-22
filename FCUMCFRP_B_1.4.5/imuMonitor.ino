//FCU Madgwick Control Filter Research Platform
//Author: Patrick Andrasena T.
//Project Start: 8/21/2025
//Last Updated: 1/22/2026
//Version: Beta 1.4.5
//
//Project's base: 

//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022
//Version: Beta 1.3

//========================================================================================================================//

#include "quad.h"
#include <Ethernet.h>
#include <EthernetUdp.h>
// Wire1 is available from Wire.h on Teensy 4.0 (no separate Wire1.h needed)

//This file contains all necessary functions and code used for IMU monitoring (control system validation)
//Purpose: MPU9250 (I2C Wire1) monitor validates MPU6050 (I2C Wire) control IMU attitude estimates
//Both IMUs run independent Madgwick filters to provide comparable attitude angles for validation.
//Monitor uses Wire1 (SDA1=RX4=pin 16, SCL1=TX4=pin 17) at 400kHz, separate from MPU6050 on Wire at 1000kHz
//Data is sent via UDP over W5500 Ethernet (SPI) instead of serial for real-time telemetry
//Enable/disable via USE_MPU9250_MONITOR_I2C in quad.h

//
// EVENT DETECTION AND DISTURBANCE MONITORING
// ==========================================
//
// This enhanced version includes comprehensive event detection and disturbance monitoring:
//
// 1. EVENT FLAGS STRUCTURE:
//    - command_change: Command input changed significantly (>10% threshold)
//    - throttle_change: Throttle changed significantly (>10% threshold)
//    - roll_command/pitch_command/yaw_command: Active command input detected
//    - attitude_change: Attitude changed beyond threshold (2°)
//    - disturbance: Large attitude change WITHOUT command input (>5°)
//    - control_active: Control system armed and active
//
// 2. DISTURBANCE DETECTION LOGIC:
//    - Only triggers when control system is armed (armedFly = true)
//    - Only triggers when NO command input is active (all sticks centered)
//    - Detects attitude changes >5° suggesting external disturbances
//    - Flags wind gusts, turbulence, or external forces
//    - Provides real-time disturbance alerts via UDP and serial
//
// 3. UDP PACKET ENHANCEMENT:
//    - Original: 112 bytes (timestamp + 27 floats)
//    - Enhanced: 113 bytes (timestamp + event_flags + 27 floats)
//    - Event flags use bit-field for efficient transmission
//    - Backward compatible with existing analysis tools
//
// 4. DETECTION THRESHOLDS:
//    - COMMAND_THRESHOLD: 0.1 (10% of max command)
//    - ATTITUDE_THRESHOLD: 2.0° (normal attitude change)
//    - DISTURBANCE_THRESHOLD: 5.0° (disturbance detection)
//    - EVENT_DEBOUNCE_MS: 100ms (prevents flag spam)
//
// 5. USAGE:
//    - Event flags are automatically included in UDP telemetry
//    - Disturbance events printed to serial for debugging
//    - UDPClient.py logs all events with timestamps
//    - FlightlogAnalysis.py can analyze event patterns
//
// 6. RESEARCH APPLICATIONS:
//    - Distinguish pilot commands from external disturbances
//    - Study control system response to wind gusts
//    - Validate disturbance rejection algorithms
//    - Analyze flight dynamics in turbulent conditions
//
// Author: Patrick Andrasena T.
// Event Detection Version: 1.1
// Last Updated: 2025-01-22

#if defined USE_MPU9250_MONITOR_I2C

// Ethernet and UDP objects
EthernetUDP Udp;
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // MAC address (change if needed)
unsigned long lastUdpSend = 0;

// Event detection flags for command-based attitude changes
// Bit-field structure for efficient UDP transmission (1 byte total)
struct EventFlags {
  uint8_t command_change : 1;      // 1 if command input changed significantly (>10% threshold)
  uint8_t throttle_change : 1;     // 1 if throttle changed significantly (>10% threshold)
  uint8_t roll_command : 1;        // 1 if roll command active (>10% from center)
  uint8_t pitch_command : 1;       // 1 if pitch command active (>10% from center)
  uint8_t yaw_command : 1;         // 1 if yaw command active (>10% from center)
  uint8_t attitude_change : 1;     // 1 if attitude changed beyond threshold (2°)
  uint8_t control_active : 1;      // 1 if control system is armed and active
  uint8_t disturbance : 1;         // 1 if attitude change detected without command input (>5°)
} __attribute__((packed));

// Structured data packet for UDP transmission
// Packet format: [timestamp(uint32_t)] [event_flags(uint8_t)] [control_raw(9 floats)] [monitor_raw(9 floats)] [control_attitude(3 floats)] [monitor_attitude(3 floats)] [errors(3 floats)]
// Total: 4 + 1 + (27 * 4) = 113 bytes
struct IMUDataPacket {
  uint32_t timestamp_us;  // Timestamp in microseconds
  EventFlags flags;        // Event detection flags
  
  // Control IMU raw sensors (MPU6050)
  float ctrl_acc_x, ctrl_acc_y, ctrl_acc_z;
  float ctrl_gyro_x, ctrl_gyro_y, ctrl_gyro_z;
  float ctrl_mag_x, ctrl_mag_y, ctrl_mag_z;
  
  // Monitor IMU raw sensors (MPU9250)
  float mon_acc_x, mon_acc_y, mon_acc_z;
  float mon_gyro_x, mon_gyro_y, mon_gyro_z;
  float mon_mag_x, mon_mag_y, mon_mag_z;
  
  // Control IMU attitude (from Madgwick filter)
  float ctrl_roll, ctrl_pitch, ctrl_yaw;
  
  // Monitor IMU attitude (from Madgwick filter)
  float mon_roll, mon_pitch, mon_yaw;
  
  // Attitude comparison errors
  float err_roll, err_pitch, err_yaw;
} __attribute__((packed));  // Pack structure to avoid padding

IMUDataPacket dataPacket;

// Event detection variables
static float prev_roll_des = 0.0, prev_pitch_des = 0.0, prev_yaw_des = 0.0, prev_thro_des = 0.0;
static float prev_roll_IMU = 0.0, prev_pitch_IMU = 0.0, prev_yaw_IMU = 0.0;
static unsigned long lastEventTime = 0;
const float COMMAND_THRESHOLD = 0.1;    // 10% change threshold for command detection
const float ATTITUDE_THRESHOLD = 2.0;   // 2 degrees threshold for attitude change detection
const float DISTURBANCE_THRESHOLD = 5.0; // 5 degrees threshold for disturbance detection
const unsigned long EVENT_DEBOUNCE_MS = 100; // 100ms debounce for event detection

// Initialize optional secondary MPU9250 monitor (I2C Wire1) for ground-truth attitude
void monitorIMUinit() {
  // Initialize Wire1 I2C bus at 400kHz (separate from Wire used by MPU6050 at 1000kHz)
  Wire1.begin();
  Wire1.setClock(400000);  // 400kHz clock speed for monitor IMU
  
  int status_mon = mpu9250_monitor.begin();
  
  if (status_mon < 0) {
    Serial.println("MPU9250 monitor initialization unsuccessful");
    Serial.println("Check MPU9250 monitor wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status_mon);
    while(1) {}
  }
  
  // Configure monitor IMU using same range settings as primary IMU (from quad.h)
  // This ensures consistent scaling for comparison/ground-truth purposes
  #if defined GYRO_250DPS
    mpu9250_monitor.setGyroRange(mpu9250_monitor.GYRO_RANGE_250DPS);
  #elif defined GYRO_500DPS
    mpu9250_monitor.setGyroRange(mpu9250_monitor.GYRO_RANGE_500DPS);
  #elif defined GYRO_1000DPS
    mpu9250_monitor.setGyroRange(mpu9250_monitor.GYRO_RANGE_1000DPS);
  #elif defined GYRO_2000DPS
    mpu9250_monitor.setGyroRange(mpu9250_monitor.GYRO_RANGE_2000DPS);
  #endif
  
  #if defined ACCEL_2G
    mpu9250_monitor.setAccelRange(mpu9250_monitor.ACCEL_RANGE_2G);
  #elif defined ACCEL_4G
    mpu9250_monitor.setAccelRange(mpu9250_monitor.ACCEL_RANGE_4G);
  #elif defined ACCEL_8G
    mpu9250_monitor.setAccelRange(mpu9250_monitor.ACCEL_RANGE_8G);
  #elif defined ACCEL_16G
    mpu9250_monitor.setAccelRange(mpu9250_monitor.ACCEL_RANGE_16G);
  #endif
  
  mpu9250_monitor.setMagCalX(MagErrorX, MagScaleX);
  mpu9250_monitor.setMagCalY(MagErrorY, MagScaleY);
  mpu9250_monitor.setMagCalZ(MagErrorZ, MagScaleZ);
  mpu9250_monitor.setSrd(0); // 1kHz accel/gyro, 100Hz mag
  
  // Initialize W5500 Ethernet via SPI
  pinMode(W5500_CS_PIN, OUTPUT);
  digitalWrite(W5500_CS_PIN, HIGH);
  W5500_SPI_BUS.begin();
  
  // Start Ethernet connection (DHCP)
  Serial.print("Initializing Ethernet...");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // Try static IP as fallback
    IPAddress ip(192, 168, 1, 177);
    IPAddress gateway(192, 168, 1, 1);
    IPAddress subnet(255, 255, 255, 0);
    Ethernet.begin(mac, ip, gateway, gateway, subnet);
  }
  Serial.print("Ethernet initialized. IP: ");
  Serial.println(Ethernet.localIP());
  
  // Start UDP
  Udp.begin(UDP_LOCAL_PORT);
  Serial.print("UDP started on port ");
  Serial.println(UDP_LOCAL_PORT);
  Serial.print("Sending to: ");
  Serial.print(UDP_REMOTE_IP_0);
  Serial.print(".");
  Serial.print(UDP_REMOTE_IP_1);
  Serial.print(".");
  Serial.print(UDP_REMOTE_IP_2);
  Serial.print(".");
  Serial.print(UDP_REMOTE_IP_3);
  Serial.print(":");
  Serial.println(UDP_REMOTE_PORT);
  
  lastUdpSend = 0;
}

// Read secondary MPU9250 monitor (I2C Wire1) for ground-truth IMU data
void getIMUdataMonitor() {
  // Read and scale data from monitor IMU using library high-level getters
  // Accelerometer: m/s^2 -> g's (to match primary IMU units)
  // Gyro: rad/s -> deg/s (to match primary IMU units)
  mpu9250_monitor.readSensor();

  // Accelerometer (convert to g and apply LP filter like primary IMU)
  float AccX_raw = mpu9250_monitor.getAccelX_mss() / 9.807f;
  float AccY_raw = mpu9250_monitor.getAccelY_mss() / 9.807f;
  float AccZ_raw = mpu9250_monitor.getAccelZ_mss() / 9.807f;
  AccX_mon = (1.0 - B_accel)*AccX_mon_prev + B_accel*AccX_raw;
  AccY_mon = (1.0 - B_accel)*AccY_mon_prev + B_accel*AccY_raw;
  AccZ_mon = (1.0 - B_accel)*AccZ_mon_prev + B_accel*AccZ_raw;
  AccX_mon_prev = AccX_mon;
  AccY_mon_prev = AccY_mon;
  AccZ_mon_prev = AccZ_mon;

  // Gyro (convert to deg/sec and apply LP filter like primary IMU)
  float GyroX_raw = mpu9250_monitor.getGyroX_rads() * 57.29577951f;
  float GyroY_raw = mpu9250_monitor.getGyroY_rads() * 57.29577951f;
  float GyroZ_raw = mpu9250_monitor.getGyroZ_rads() * 57.29577951f;
  GyroX_mon = (1.0 - B_gyro)*GyroX_mon_prev + B_gyro*GyroX_raw;
  GyroY_mon = (1.0 - B_gyro)*GyroY_mon_prev + B_gyro*GyroY_raw;
  GyroZ_mon = (1.0 - B_gyro)*GyroZ_mon_prev + B_gyro*GyroZ_raw;
  GyroX_mon_prev = GyroX_mon;
  GyroY_mon_prev = GyroY_mon;
  GyroZ_mon_prev = GyroZ_mon;

  // Magnetometer (apply calibration and LP filter like primary IMU)
  float MagX_raw = mpu9250_monitor.getMagX_uT();
  float MagY_raw = mpu9250_monitor.getMagY_uT();
  float MagZ_raw = mpu9250_monitor.getMagZ_uT();
  MagX_raw = (MagX_raw - MagErrorX)*MagScaleX;
  MagY_raw = (MagY_raw - MagErrorY)*MagScaleY;
  MagZ_raw = (MagZ_raw - MagErrorZ)*MagScaleZ;
  MagX_mon = (1.0 - B_mag)*MagX_mon_prev + B_mag*MagX_raw;
  MagY_mon = (1.0 - B_mag)*MagY_mon_prev + B_mag*MagY_raw;
  MagZ_mon = (1.0 - B_mag)*MagZ_mon_prev + B_mag*MagZ_raw;
  MagX_mon_prev = MagX_mon;
  MagY_mon_prev = MagY_mon;
  MagZ_mon_prev = MagZ_mon;
}

// Separate Madgwick filter for monitor IMU (ground truth attitude)
// Always computed for UDP telemetry (both raw sensor and attitude data are sent)
void MadgwickMonitor(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq) {
  // Same Madgwick algorithm as primary IMU, but using separate quaternion state (q0_mon, q1_mon, q2_mon, q3_mon)
  // This allows independent attitude estimation for comparison/ground truth purposes
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Use 6DOF algorithm if magnetometer measurement invalid
  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    Madgwick6DOFMonitor(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
  }

  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Rate of change of quaternion from gyroscope (using monitor quaternion state)
  qDot1 = 0.5f * (-q1_mon * gx - q2_mon * gy - q3_mon * gz);
  qDot2 = 0.5f * (q0_mon * gx + q2_mon * gz - q3_mon * gy);
  qDot3 = 0.5f * (q0_mon * gy - q1_mon * gz + q3_mon * gx);
  qDot4 = 0.5f * (q0_mon * gz + q1_mon * gy - q2_mon * gx);

  // Compute feedback only if accelerometer measurement valid
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables (using monitor quaternion state)
    _2q0mx = 2.0f * q0_mon * mx;
    _2q0my = 2.0f * q0_mon * my;
    _2q0mz = 2.0f * q0_mon * mz;
    _2q1mx = 2.0f * q1_mon * mx;
    _2q0 = 2.0f * q0_mon;
    _2q1 = 2.0f * q1_mon;
    _2q2 = 2.0f * q2_mon;
    _2q3 = 2.0f * q3_mon;
    _2q0q2 = 2.0f * q0_mon * q2_mon;
    _2q2q3 = 2.0f * q2_mon * q3_mon;
    q0q0 = q0_mon * q0_mon;
    q0q1 = q0_mon * q1_mon;
    q0q2 = q0_mon * q2_mon;
    q0q3 = q0_mon * q3_mon;
    q1q1 = q1_mon * q1_mon;
    q1q2 = q1_mon * q2_mon;
    q1q3 = q1_mon * q3_mon;
    q2q2 = q2_mon * q2_mon;
    q2q3 = q2_mon * q3_mon;
    q3q3 = q3_mon * q3_mon;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3_mon + _2q0mz * q2_mon + mx * q1q1 + _2q1 * my * q2_mon + _2q1 * mz * q3_mon - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3_mon + my * q0q0 - _2q0mz * q1_mon + _2q1mx * q2_mon - my * q1q1 + my * q2q2 + _2q2 * mz * q3_mon - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2_mon + _2q0my * q1_mon + mz * q0q0 + _2q1mx * q3_mon - mz * q1q1 + _2q2 * my * q3_mon - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2_mon * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3_mon + _2bz * q1_mon) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2_mon * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1_mon * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3_mon * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2_mon + _2bz * q0_mon) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3_mon - _4bz * q1_mon) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2_mon * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2_mon - _2bz * q0_mon) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1_mon + _2bz * q3_mon) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0_mon - _4bz * q2_mon) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3_mon + _2bz * q1_mon) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0_mon + _2bz * q2_mon) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1_mon * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0_mon += qDot1 * invSampleFreq;
  q1_mon += qDot2 * invSampleFreq;
  q2_mon += qDot3 * invSampleFreq;
  q3_mon += qDot4 * invSampleFreq;

  // Normalize quaternion
  recipNorm = invSqrt(q0_mon * q0_mon + q1_mon * q1_mon + q2_mon * q2_mon + q3_mon * q3_mon);
  q0_mon *= recipNorm;
  q1_mon *= recipNorm;
  q2_mon *= recipNorm;
  q3_mon *= recipNorm;
  
  // Compute angles - NWU (same convention as primary IMU)
  roll_IMU_mon = atan2(q0_mon*q1_mon + q2_mon*q3_mon, 0.5f - q1_mon*q1_mon - q2_mon*q2_mon)*57.29577951f;
  pitch_IMU_mon = -asin(constrain(-2.0f * (q1_mon*q3_mon - q0_mon*q2_mon),-0.999999f,0.999999f))*57.29577951f;
  yaw_IMU_mon = -atan2(q1_mon*q2_mon + q0_mon*q3_mon, 0.5f - q2_mon*q2_mon - q3_mon*q3_mon)*57.29577951f;
}

// 6DOF Madgwick for monitor IMU (when magnetometer unavailable)
void Madgwick6DOFMonitor(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Rate of change of quaternion from gyroscope (using monitor quaternion state)
  qDot1 = 0.5f * (-q1_mon * gx - q2_mon * gy - q3_mon * gz);
  qDot2 = 0.5f * (q0_mon * gx + q2_mon * gz - q3_mon * gy);
  qDot3 = 0.5f * (q0_mon * gy - q1_mon * gz + q3_mon * gx);
  qDot4 = 0.5f * (q0_mon * gz + q1_mon * gy - q2_mon * gx);

  // Compute feedback only if accelerometer measurement valid
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables (using monitor quaternion state)
    _2q0 = 2.0f * q0_mon;
    _2q1 = 2.0f * q1_mon;
    _2q2 = 2.0f * q2_mon;
    _2q3 = 2.0f * q3_mon;
    _4q0 = 4.0f * q0_mon;
    _4q1 = 4.0f * q1_mon;
    _4q2 = 4.0f * q2_mon;
    _8q1 = 8.0f * q1_mon;
    _8q2 = 8.0f * q2_mon;
    q0q0 = q0_mon * q0_mon;
    q1q1 = q1_mon * q1_mon;
    q2q2 = q2_mon * q2_mon;
    q3q3 = q3_mon * q3_mon;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_mon - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2_mon + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3_mon - _2q1 * ax + 4.0f * q2q2 * q3_mon - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0_mon += qDot1 * invSampleFreq;
  q1_mon += qDot2 * invSampleFreq;
  q2_mon += qDot3 * invSampleFreq;
  q3_mon += qDot4 * invSampleFreq;

  // Normalise quaternion
  recipNorm = invSqrt(q0_mon * q0_mon + q1_mon * q1_mon + q2_mon * q2_mon + q3_mon * q3_mon);
  q0_mon *= recipNorm;
  q1_mon *= recipNorm;
  q2_mon *= recipNorm;
  q3_mon *= recipNorm;

  // Compute angles
  roll_IMU_mon = atan2(q0_mon*q1_mon + q2_mon*q3_mon, 0.5f - q1_mon*q1_mon - q2_mon*q2_mon)*57.29577951f;
  pitch_IMU_mon = -asin(constrain(-2.0f * (q1_mon*q3_mon - q0_mon*q2_mon),-0.999999f,0.999999f))*57.29577951f;
  yaw_IMU_mon = -atan2(q1_mon*q2_mon + q0_mon*q3_mon, 0.5f - q2_mon*q2_mon - q3_mon*q3_mon)*57.29577951f;
}

// Compare control IMU attitude (used for flight control) vs ground truth monitor IMU attitude
#if defined USE_MONITOR_ATTITUDE_COMPARISON
void compareAttitude() {
  // Compute attitude errors: control IMU - ground truth monitor IMU
  // Positive error means control IMU reads higher angle than ground truth
  roll_error_mon = roll_IMU - roll_IMU_mon;
  pitch_error_mon = pitch_IMU - pitch_IMU_mon;
  
  // Yaw error needs special handling due to wrap-around at ±180 degrees
  float yaw_diff = yaw_IMU - yaw_IMU_mon;
  if (yaw_diff > 180.0f) yaw_diff -= 360.0f;
  if (yaw_diff < -180.0f) yaw_diff += 360.0f;
  yaw_error_mon = yaw_diff;
}
#endif // USE_MONITOR_ATTITUDE_COMPARISON

// Event detection function to flag command-based attitude changes and disturbances
// This function analyzes both command inputs and attitude changes to detect different types of flight events
// Uses bit-field flags for efficient transmission and logging
void detectEvents() {
  // Clear all flags first (reset to 0)
  dataPacket.flags = {0};
  
  // Check if enough time has passed for event detection (debounce)
  // This prevents flag spam during continuous inputs
  unsigned long currentTime = millis();
  if (currentTime - lastEventTime < EVENT_DEBOUNCE_MS) {
    // During debounce period, only maintain control_active flag
    dataPacket.flags.control_active = armedFly;
    return;
  }
  
  bool eventDetected = false;
  bool commandActive = false;
  
  // Check for command changes (compare current vs previous desired values)
  float roll_change = abs(roll_des - prev_roll_des);
  float pitch_change = abs(pitch_des - prev_pitch_des);
  float yaw_change = abs(yaw_des - prev_yaw_des);
  float throttle_change = abs(thro_des - prev_thro_des);
  
  // Flag if any command changed beyond threshold
  if (roll_change > COMMAND_THRESHOLD || pitch_change > COMMAND_THRESHOLD || yaw_change > COMMAND_THRESHOLD) {
    dataPacket.flags.command_change = 1;
    eventDetected = true;
  }
  
  // Flag throttle changes separately
  if (throttle_change > COMMAND_THRESHOLD) {
    dataPacket.flags.throttle_change = 1;
    eventDetected = true;
  }
  
  // Check for active commands (beyond center stick position)
  // This helps distinguish between commanded and uncommanded attitude changes
  if (abs(roll_des) > COMMAND_THRESHOLD) {
    dataPacket.flags.roll_command = 1;
    commandActive = true;
    eventDetected = true;
  }
  
  if (abs(pitch_des) > COMMAND_THRESHOLD) {
    dataPacket.flags.pitch_command = 1;
    commandActive = true;
    eventDetected = true;
  }
  
  if (abs(yaw_des) > COMMAND_THRESHOLD) {
    dataPacket.flags.yaw_command = 1;
    commandActive = true;
    eventDetected = true;
  }
  
  // Check for attitude changes beyond threshold (actual vs previous attitude)
  float roll_attitude_change = abs(roll_IMU - prev_roll_IMU);
  float pitch_attitude_change = abs(pitch_IMU - prev_pitch_IMU);
  float yaw_attitude_change = abs(yaw_IMU - prev_yaw_IMU);
  
  if (roll_attitude_change > ATTITUDE_THRESHOLD || pitch_attitude_change > ATTITUDE_THRESHOLD || yaw_attitude_change > ATTITUDE_THRESHOLD) {
    dataPacket.flags.attitude_change = 1;
    eventDetected = true;
  }
  
  // DISTURBANCE DETECTION: Check for large attitude changes without corresponding command input
  // This is the key innovation - detecting external disturbances like wind gusts
  // Only detect disturbance if control system is active and no command is active
  if (armedFly && !commandActive) {
    // Check for large, rapid attitude changes that suggest external disturbances
    // Higher threshold than normal attitude changes to avoid false positives
    if (roll_attitude_change > DISTURBANCE_THRESHOLD || pitch_attitude_change > DISTURBANCE_THRESHOLD || yaw_attitude_change > DISTURBANCE_THRESHOLD) {
      dataPacket.flags.disturbance = 1;
      eventDetected = true;
      
      // Optional: Print disturbance detection to serial for debugging
      // This helps with field testing and threshold tuning
      Serial.print("DISTURBANCE DETECTED: ");
      if (roll_attitude_change > DISTURBANCE_THRESHOLD) {
        Serial.print("Roll=");
        Serial.print(roll_attitude_change, 2);
        Serial.print("° ");
      }
      if (pitch_attitude_change > DISTURBANCE_THRESHOLD) {
        Serial.print("Pitch=");
        Serial.print(pitch_attitude_change, 2);
        Serial.print("° ");
      }
      if (yaw_attitude_change > DISTURBANCE_THRESHOLD) {
        Serial.print("Yaw=");
        Serial.print(yaw_attitude_change, 2);
        Serial.print("° ");
      }
      Serial.println();
    }
  }
  
  // Set control active flag (armed status)
  dataPacket.flags.control_active = armedFly;
  
  // Update previous values if event detected (for next comparison)
  if (eventDetected) {
    prev_roll_des = roll_des;
    prev_pitch_des = pitch_des;
    prev_yaw_des = yaw_des;
    prev_thro_des = thro_des;
    prev_roll_IMU = roll_IMU;
    prev_pitch_IMU = pitch_IMU;
    prev_yaw_IMU = yaw_IMU;
    lastEventTime = currentTime;
  }
}

// Send IMU data via UDP over W5500 Ethernet
// This function packages both raw sensor data and attitude data with timestamp
// Both attitude and raw sensor data are ALWAYS sent, regardless of comparison mode setting
void sendIMUDataUDP() {
  // Rate limit UDP sends to avoid overwhelming network
  unsigned long currentMillis = millis();
  if (currentMillis - lastUdpSend < MONITOR_UDP_RATE_MS) {
    return;
  }
  lastUdpSend = currentMillis;
  
  // Detect events before populating packet
  detectEvents();
  
  // Populate packet with timestamp
  dataPacket.timestamp_us = micros();
  
  // Control IMU raw sensors (MPU6050) - ALWAYS sent
  dataPacket.ctrl_acc_x = AccX;
  dataPacket.ctrl_acc_y = AccY;
  dataPacket.ctrl_acc_z = AccZ;
  dataPacket.ctrl_gyro_x = GyroX;
  dataPacket.ctrl_gyro_y = GyroY;
  dataPacket.ctrl_gyro_z = GyroZ;
  dataPacket.ctrl_mag_x = MagX;
  dataPacket.ctrl_mag_y = MagY;
  dataPacket.ctrl_mag_z = MagZ;
  
  // Monitor IMU raw sensors (MPU9250) - ALWAYS sent
  dataPacket.mon_acc_x = AccX_mon;
  dataPacket.mon_acc_y = AccY_mon;
  dataPacket.mon_acc_z = AccZ_mon;
  dataPacket.mon_gyro_x = GyroX_mon;
  dataPacket.mon_gyro_y = GyroY_mon;
  dataPacket.mon_gyro_z = GyroZ_mon;
  dataPacket.mon_mag_x = MagX_mon;
  dataPacket.mon_mag_y = MagY_mon;
  dataPacket.mon_mag_z = MagZ_mon;
  
  // Control IMU attitude (from Madgwick filter) - ALWAYS sent
  dataPacket.ctrl_roll = roll_IMU;
  dataPacket.ctrl_pitch = pitch_IMU;
  dataPacket.ctrl_yaw = yaw_IMU;
  
  // Monitor IMU attitude (from Madgwick filter) - ALWAYS sent
  dataPacket.mon_roll = roll_IMU_mon;
  dataPacket.mon_pitch = pitch_IMU_mon;
  dataPacket.mon_yaw = yaw_IMU_mon;
  
  // Error/difference data - depends on comparison mode
  #if defined USE_MONITOR_ATTITUDE_COMPARISON
    // Attitude comparison errors (control - monitor)
    dataPacket.err_roll = roll_error_mon;
    dataPacket.err_pitch = pitch_error_mon;
    dataPacket.err_yaw = yaw_error_mon;
  #else
    // Raw sensor differences (reusing error variables for raw sensor differences)
    dataPacket.err_roll = roll_error_mon;   // Gyro X difference
    dataPacket.err_pitch = pitch_error_mon; // Gyro Y difference
    dataPacket.err_yaw = yaw_error_mon;     // Gyro Z difference
  #endif
  
  // Send UDP packet
  IPAddress remoteIP(UDP_REMOTE_IP_0, UDP_REMOTE_IP_1, UDP_REMOTE_IP_2, UDP_REMOTE_IP_3);
  Udp.beginPacket(remoteIP, UDP_REMOTE_PORT);
  Udp.write((uint8_t*)&dataPacket, sizeof(IMUDataPacket));
  Udp.endPacket();
}

// Legacy serial print functions (kept for debugging, but data now sent via UDP)
#if defined USE_MONITOR_ATTITUDE_COMPARISON
void printAttitudeComparison() {
  // Data now sent via UDP - this function kept for compatibility
  // Uncomment Serial prints below if you want to also print to serial
  /*
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("CTRL: R:"));
    Serial.print(roll_IMU);
    Serial.print(F(" P:"));
    Serial.print(pitch_IMU);
    Serial.print(F(" Y:"));
    Serial.print(yaw_IMU);
    Serial.print(F(" | GT: R:"));
    Serial.print(roll_IMU_mon);
    Serial.print(F(" P:"));
    Serial.print(pitch_IMU_mon);
    Serial.print(F(" Y:"));
    Serial.print(yaw_IMU_mon);
    Serial.print(F(" | ERR: R:"));
    Serial.print(roll_error_mon);
    Serial.print(F(" P:"));
    Serial.print(pitch_error_mon);
    Serial.print(F(" Y:"));
    Serial.println(yaw_error_mon);
  }
  */
}
#else
// Raw sensor comparison mode (no Madgwick filter on monitor)
void compareRawSensors() {
  // Compare raw sensor readings between control and monitor IMUs
  // This is lighter-weight than attitude comparison but less directly useful for control validation
  // Differences indicate sensor calibration, noise, or mounting differences
  float gyro_diff_x = GyroX - GyroX_mon;
  float gyro_diff_y = GyroY - GyroY_mon;
  float gyro_diff_z = GyroZ - GyroZ_mon;
  float accel_diff_x = AccX - AccX_mon;
  float accel_diff_y = AccY - AccY_mon;
  float accel_diff_z = AccZ - AccZ_mon;
  
  // Store differences in error variables (reusing same variable names for consistency)
  roll_error_mon = gyro_diff_x;   // Reuse for gyro X difference
  pitch_error_mon = gyro_diff_y;   // Reuse for gyro Y difference
  yaw_error_mon = gyro_diff_z;     // Reuse for gyro Z difference
}

void printRawSensorComparison() {
  // Data now sent via UDP - this function kept for compatibility
  // Uncomment Serial prints below if you want to also print to serial
  /*
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("CTRL_Gyro: X:"));
    Serial.print(GyroX);
    Serial.print(F(" Y:"));
    Serial.print(GyroY);
    Serial.print(F(" Z:"));
    Serial.print(GyroZ);
    Serial.print(F(" | MON_Gyro: X:"));
    Serial.print(GyroX_mon);
    Serial.print(F(" Y:"));
    Serial.print(GyroY_mon);
    Serial.print(F(" Z:"));
    Serial.print(GyroZ_mon);
    Serial.print(F(" | Diff: X:"));
    Serial.print(roll_error_mon);
    Serial.print(F(" Y:"));
    Serial.print(pitch_error_mon);
    Serial.print(F(" Z:"));
    Serial.println(yaw_error_mon);
  }
  */
}
#endif // USE_MONITOR_ATTITUDE_COMPARISON

#endif // USE_MPU9250_MONITOR_I2C
