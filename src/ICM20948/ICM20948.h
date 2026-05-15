/*
ICM20948.h
Patrick Andrasena T.
FCU Madgwick Control Filter Research Platform
Last Updated : 2026-05-14
Version      : 1.0
pat271203@gmail.com

Attempt to integrate the use of 9DOF IMU ICM-20948 into DRehmFlight's stacks
since MPU9250 for 9DOF solutions are saturated by counterfeit breakout boards
and non working magnetometer chips.

    #define USE_MPU9250_MONITOR_I2C   →  MPU9250  mpu9250_monitor(Wire1, 0x69)
    #define USE_ICM20948_MONITOR_I2C  →  ICM20948 icm20948_monitor(Wire1)
 
  All getter methods return the same physical units as the MPU9250
  library (m/s² for accel, rad/s for gyro, µT for magnetometer) so
  that getIMUdataMonitor() in imuMonitor.ino requires no changes.
 
  I²C only — the ICM-20948 SPI path is not implemented here because
  the monitor IMU is wired to Wire1 (SDA1/pin 16, SCL1/pin 17).
 
  ICM-20948 internal architecture notes
  ──────────────────────────────────────
  • Accel + Gyro live in the main die (Bank 0 / Bank 2 registers).
  • Magnetometer (AK09916) is a separate die connected via the
    ICM-20948's internal I²C master on Auxiliary I²C pins.
  • The user bank is selected by writing BANK_SEL (reg 0x7F).
  • All user-facing reads go through Bank 0 after configuration.
 
  Register map references
  ───────────────────────
  [DS] TDK InvenSense ICM-20948 Datasheet Rev 1.3
  [RM] ICM-20948 Register Map and Descriptions Rev 1.0

Implementations inspired based on MPU9250 library by:
brian.taylor@bolderflight.com
http://www.bolderflight.com

*/

#ifndef ICM20948_h
#define ICM20948_h
 
#include "Arduino.h"
#include "Wire.h"
 
// ─── I²C addresses ────────────────────────────────────────────────────────────
// AD0 low  (default, SDO/AD0 pin floating or tied to GND) → 0x68
// AD0 high (SDO/AD0 pin tied to VDD)                      → 0x69
#define ICM20948_I2C_ADDR_LOW   0x68
#define ICM20948_I2C_ADDR_HIGH  0x69
 
// ─── WHO_AM_I expected value ───────────────────────────────────────────────────
#define ICM20948_WHO_AM_I_VAL   0xEA   // [DS] §14.1
 
// ─── AK09916 magnetometer I²C address (internal bus) ─────────────────────────
#define AK09916_I2C_ADDR        0x0C
 
// ─── AK09916 WHO_AM_I expected value ─────────────────────────────────────────
#define AK09916_WHO_AM_I_VAL    0x09   // [DS AK09916] §8.1  WIA2 register
 
 
// ══════════════════════════════════════════════════════════════════════════════
//  ICM20948 class
// ══════════════════════════════════════════════════════════════════════════════
 
class ICM20948 {
 public:
 
  // ── Range enums (match MPU9250 naming so imuMonitor.ino #if chains compile) ─
 
  enum GyroRange {
    GYRO_RANGE_250DPS  = 0,
    GYRO_RANGE_500DPS  = 1,
    GYRO_RANGE_1000DPS = 2,
    GYRO_RANGE_2000DPS = 3
  };
 
  enum AccelRange {
    ACCEL_RANGE_2G  = 0,
    ACCEL_RANGE_4G  = 1,
    ACCEL_RANGE_8G  = 2,
    ACCEL_RANGE_16G = 3
  };
 
  // DLPF bandwidth enum — kept for API symmetry; values map to ICM-20948
  // GYRO_CONFIG_1 / ACCEL_CONFIG DLPFCFG fields.
  enum DlpfBandwidth {
    DLPF_BANDWIDTH_229HZ = 0,  // Closest ICM-20948 equivalent to MPU9250 184 Hz
    DLPF_BANDWIDTH_119HZ = 1,
    DLPF_BANDWIDTH_51HZ  = 2,
    DLPF_BANDWIDTH_24HZ  = 3,
    DLPF_BANDWIDTH_12HZ  = 4,
    DLPF_BANDWIDTH_6HZ   = 5,
    DLPF_BANDWIDTH_361HZ = 6   // ~bypass (DLPF off)
  };
 
  // ── Constructor ────────────────────────────────────────────────────────────
  // bus     : reference to a TwoWire object (Wire, Wire1, …)
  // address : I²C address — default 0x68 (AD0 low)
  explicit ICM20948(TwoWire &bus, uint8_t address = ICM20948_I2C_ADDR_LOW);
 
  // ── Lifecycle ──────────────────────────────────────────────────────────────
  // begin() initialises the device, wakes it, configures default ranges
  // (±16 g, ±2000 °/s), enables the internal I²C master for the AK09916,
  // and performs a gyro bias calibration.
  // Returns  1 on success, negative error code on failure.
  int begin();
 
  // ── Range configuration ────────────────────────────────────────────────────
  int setAccelRange(AccelRange range);
  int setGyroRange(GyroRange range);
  int setDlpfBandwidth(DlpfBandwidth bandwidth);
 
  // setSrd — sample-rate divider (0 = 1125 Hz / (1+0) = 1125 Hz for gyro/accel).
  // Maps to the ICM-20948 GYRO_SMPLRT_DIV / ACCEL_SMPLRT_DIV registers.
  // Pass 0 for maximum rate (matches mpu9250_monitor.setSrd(0) in imuMonitor.ino).
  int setSrd(uint8_t srd);
 
  // ── Magnetometer calibration setters (same signature as MPU9250) ──────────
  // bias   : hard-iron offset in µT
  // scale  : soft-iron scale factor (dimensionless)
  void setMagCalX(float bias, float scale);
  void setMagCalY(float bias, float scale);
  void setMagCalZ(float bias, float scale);
 
  // ── Data acquisition ───────────────────────────────────────────────────────
  // readSensor() bursts all accel, gyro, temp, and magnetometer registers
  // in one I²C transaction (accel+gyro from Bank 0 ACCEL_XOUT_H, mag via
  // the external sensor data registers EXT_SLV_SENS_DATA_00).
  // Must be called before the getXxx() functions each loop iteration.
  int readSensor();
 
  // ── Getters — same units as MPU9250 ───────────────────────────────────────
  // Accelerometer : m/s²   (imuMonitor divides by 9.807 to get g)
  float getAccelX_mss();
  float getAccelY_mss();
  float getAccelZ_mss();
 
  // Gyroscope : rad/s   (imuMonitor multiplies by 57.29578 to get °/s)
  float getGyroX_rads();
  float getGyroY_rads();
  float getGyroZ_rads();
 
  // Magnetometer : µT   (bias and scale applied internally by setMagCalX/Y/Z)
  float getMagX_uT();
  float getMagY_uT();
  float getMagZ_uT();
 
  float getTemperature_C();
 
  // ── Calibration helpers ────────────────────────────────────────────────────
  // calibrateGyro() averages _numSamples readings at rest to estimate bias.
  // Called automatically inside begin() but can be called again by the user.
  int calibrateGyro();
 
  float getGyroBiasX_rads();
  float getGyroBiasY_rads();
  float getGyroBiasZ_rads();
  void  setGyroBiasX_rads(float bias);
  void  setGyroBiasY_rads(float bias);
  void  setGyroBiasZ_rads(float bias);
 
  // ── Magnetometer calibration getters (used by calibrateMag helper) ─────────
  float getMagBiasX_uT();
  float getMagScaleFactorX();
  float getMagBiasY_uT();
  float getMagScaleFactorY();
  float getMagBiasZ_uT();
  float getMagScaleFactorZ();
 
  // calibrateMag() — interactive hard/soft-iron calibration routine.
  // Spins until _maxCounts samples are collected then computes bias & scale.
  // Returns 1 on success (same API as MPU9250::calibrateMag()).
  int calibrateMag();
 
 private:
 
  // ── I²C handles ───────────────────────────────────────────────────────────
  TwoWire *_i2c;
  uint8_t  _address;
  size_t   _numBytes;                  // bytes received from last Wire read
 
  // ── I²C clock ─────────────────────────────────────────────────────────────
  // imuMonitor calls Wire1.setClock(400000) before begin(), so we honour
  // whatever clock was already set rather than forcing a specific value here.
  const uint32_t _i2cRate = 400000;   // fallback if Wire1 not pre-configured
 
  // ── Scaled output cache (written by readSensor, read by getters) ──────────
  float _ax, _ay, _az;   // m/s²
  float _gx, _gy, _gz;   // rad/s
  float _hx, _hy, _hz;   // µT  (after bias / scale)
  float _t;              // °C
 
  // ── Raw register counts (written by readSensor, used for calibration) ─────
  int16_t _axcounts, _aycounts, _azcounts;
  int16_t _gxcounts, _gycounts, _gzcounts;
  int16_t _hxcounts, _hycounts, _hzcounts;
  int16_t _tcounts;
 
  // ── Scale factors ─────────────────────────────────────────────────────────
  float _accelScale;          // (m/s²) / count
  float _gyroScale;           // (rad/s) / count
  float _magScaleX;           // µT / count  (from AK09916 sensitivity)
  float _magScaleY;
  float _magScaleZ;
  const float _tempScale  = 333.87f;
  const float _tempOffset = 21.0f;
 
  // ── Configuration cache ────────────────────────────────────────────────────
  AccelRange    _accelRange;
  GyroRange     _gyroRange;
  DlpfBandwidth _bandwidth;
  uint8_t       _srd;
 
  // ── Gyro bias (rad/s) — estimated by calibrateGyro() ─────────────────────
  size_t _numSamples = 100;
  float  _gxb = 0.0f, _gyb = 0.0f, _gzb = 0.0f;
  double _gxbD, _gybD, _gzbD;           // accumulator during calibration
 
  // ── Magnetometer calibration (user-supplied via setMagCalX/Y/Z) ───────────
  float _hxb = 0.0f, _hyb = 0.0f, _hzb = 0.0f;   // hard-iron bias  (µT)
  float _hxs = 1.0f, _hys = 1.0f, _hzs = 1.0f;   // soft-iron scale
 
  // ── Magnetometer self-calibration state (used by calibrateMag) ───────────
  uint16_t _maxCounts  = 1000;
  float    _deltaThresh = 0.3f;
  uint8_t  _coeff      = 8;
  uint16_t _counter;
  float    _framedelta, _delta;
  float    _hxfilt, _hyfilt, _hzfilt;
  float    _hxmax, _hymax, _hzmax;
  float    _hxmin, _hymin, _hzmin;
  float    _avgs;
 
  // ── I/O buffer ────────────────────────────────────────────────────────────
  uint8_t _buffer[24];   // enough for accel(6)+gyro(6)+temp(2)+mag(8)+status(2)
 
  // ── Physical constants ────────────────────────────────────────────────────
  const float _G    = 9.807f;
  const float _d2r  = 3.14159265359f / 180.0f;
 
  // ════════════════════════════════════════════════════════════════════════════
  //  ICM-20948 Register map  (Bank 0 unless noted)
  //  [RM] Rev 1.0
  // ════════════════════════════════════════════════════════════════════════════
 
  // ── Bank select (common to all banks) ─────────────────────────────────────
  static const uint8_t REG_BANK_SEL      = 0x7F;
 
  // ── Bank 0 ────────────────────────────────────────────────────────────────
  static const uint8_t WHO_AM_I          = 0x00;
  static const uint8_t USER_CTRL         = 0x03;
  static const uint8_t LP_CONFIG         = 0x05;
  static const uint8_t PWR_MGMT_1        = 0x06;
  static const uint8_t PWR_MGMT_2        = 0x07;
  static const uint8_t INT_PIN_CFG       = 0x0F;
  static const uint8_t INT_ENABLE        = 0x10;
  static const uint8_t INT_ENABLE_1      = 0x11;
  static const uint8_t INT_STATUS        = 0x19;
  static const uint8_t INT_STATUS_1      = 0x1A;
  static const uint8_t ACCEL_XOUT_H      = 0x2D;   // first of 12 bytes accel+gyro
  static const uint8_t TEMP_OUT_H        = 0x39;
  static const uint8_t EXT_SLV_SENS_DATA_00 = 0x3B; // mag data mirrored here
  static const uint8_t FIFO_EN_1         = 0x66;
  static const uint8_t FIFO_EN_2         = 0x67;
 
  // USER_CTRL bit masks
  static const uint8_t I2C_MST_EN        = 0x20;
  static const uint8_t FIFO_EN           = 0x40;
  static const uint8_t DMP_EN            = 0x80;
 
  // PWR_MGMT_1 fields
  static const uint8_t CLOCK_SEL_AUTO    = 0x01;   // CLKSEL=1 → auto best source
  static const uint8_t DEV_RESET         = 0x80;
  static const uint8_t SLEEP_BIT         = 0x40;
 
  // PWR_MGMT_2 — enable/disable accel + gyro axes
  static const uint8_t ALL_AXES_EN       = 0x00;   // enable all 6 axes
  static const uint8_t ACCEL_DIS         = 0x38;
  static const uint8_t GYRO_DIS          = 0x07;
 
  // ── Bank 2 ────────────────────────────────────────────────────────────────
  static const uint8_t GYRO_SMPLRT_DIV  = 0x00;   // Bank 2
  static const uint8_t GYRO_CONFIG_1    = 0x01;   // Bank 2 — GYRO_FS_SEL, GYRO_DLPFCFG
  static const uint8_t GYRO_CONFIG_2    = 0x02;   // Bank 2 — averaging
  static const uint8_t ACCEL_SMPLRT_DIV_1 = 0x10; // Bank 2 — MSB of 12-bit divider
  static const uint8_t ACCEL_SMPLRT_DIV_2 = 0x11; // Bank 2 — LSB
  static const uint8_t ACCEL_INTEL_CTRL = 0x12;   // Bank 2
  static const uint8_t ACCEL_WOM_THR    = 0x13;   // Bank 2
  static const uint8_t ACCEL_CONFIG     = 0x14;   // Bank 2 — ACCEL_FS_SEL, ACCEL_DLPFCFG
 
  // GYRO_CONFIG_1 / ACCEL_CONFIG FS_SEL masks
  static const uint8_t GYRO_FS_250DPS   = 0x00;   // GYRO_FS_SEL[1:0] = 00
  static const uint8_t GYRO_FS_500DPS   = 0x02;   //                  = 01
  static const uint8_t GYRO_FS_1000DPS  = 0x04;   //                  = 10
  static const uint8_t GYRO_FS_2000DPS  = 0x06;   //                  = 11
  static const uint8_t ACCEL_FS_2G      = 0x00;   // ACCEL_FS_SEL[1:0]= 00
  static const uint8_t ACCEL_FS_4G      = 0x02;   //                  = 01
  static const uint8_t ACCEL_FS_8G      = 0x04;   //                  = 10
  static const uint8_t ACCEL_FS_16G     = 0x06;   //                  = 11
 
  // DLPF enable bit (bit 0 of GYRO_CONFIG_1 / ACCEL_CONFIG)
  static const uint8_t DLPF_EN          = 0x01;
 
  // ── Bank 3 — Auxiliary I²C master (for AK09916) ───────────────────────────
  static const uint8_t I2C_MST_CTRL     = 0x01;   // Bank 3
  static const uint8_t I2C_MST_DELAY_CTRL = 0x02; // Bank 3
  static const uint8_t I2C_SLV0_ADDR    = 0x03;   // Bank 3
  static const uint8_t I2C_SLV0_REG     = 0x04;   // Bank 3
  static const uint8_t I2C_SLV0_CTRL    = 0x05;   // Bank 3
  static const uint8_t I2C_SLV0_DO      = 0x06;   // Bank 3
  static const uint8_t I2C_MST_CLK_400  = 0x07;   // I2C_MST_CLK field → ~400 kHz
 
  static const uint8_t I2C_SLV0_EN      = 0x80;   // EN bit in I2C_SLVx_CTRL
  static const uint8_t I2C_READ_FLAG    = 0x80;   // set in SLV0_ADDR for reads
 
  // ── AK09916 registers ─────────────────────────────────────────────────────
  static const uint8_t AK09916_WIA2     = 0x01;   // device ID (expect 0x09)
  static const uint8_t AK09916_ST1      = 0x10;   // status: DRDY bit
  static const uint8_t AK09916_HXL      = 0x11;   // first of 6 mag bytes
  static const uint8_t AK09916_ST2      = 0x18;   // must read to unlatch data
  static const uint8_t AK09916_CNTL2    = 0x31;   // mode control
  static const uint8_t AK09916_CNTL3    = 0x32;   // soft-reset
  // AK09916 CNTL2 mode values
  static const uint8_t AK09916_PWR_DOWN = 0x00;
  static const uint8_t AK09916_CONT_100HZ = 0x08; // continuous mode 4 (100 Hz)
  static const uint8_t AK09916_RESET    = 0x01;   // CNTL3 soft-reset
 
  // AK09916 sensitivity (fixed, no ASAX/ASAY/ASAZ correction needed for AK09916)
  // [DS AK09916] §8.3.4 — 0.15 µT/LSB at 16-bit
  static const float AK09916_MAG_SCALE; // defined in .cpp as 0.15f
 
  // ── Private helpers ────────────────────────────────────────────────────────
 
  // Bank-aware register access
  // selectBank() writes REG_BANK_SEL; kept private so callers use writeReg /
  // readRegs which handle the bank switch transparently.
  int  selectBank(uint8_t bank);
 
  // writeReg  — selects bank, writes one byte, returns 1 on success / -1 on NACK
  int  writeReg(uint8_t bank, uint8_t reg, uint8_t data);
 
  // readRegs  — selects bank, bursts count bytes into dest
  int  readRegs(uint8_t bank, uint8_t reg, uint8_t count, uint8_t *dest);
 
  // Auxiliary I²C master helpers (for AK09916 via internal bus)
  int  writeAK09916Reg(uint8_t reg, uint8_t data);
  int  readAK09916Regs(uint8_t reg, uint8_t count, uint8_t *dest);
 
  // whoAmI helpers
  uint8_t whoAmI();
  uint8_t whoAmIAK09916();
};
 
#endif  // ICM20948_h
 