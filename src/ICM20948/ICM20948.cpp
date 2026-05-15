/* 
ICM20948.cpp
Patrick Andrasena T.
FCU Madgwick Control Filter Research Platform
Last Updated : 2026-05-14
Version      : 1.0
pat271203@gmail.com

Attempt to integrate the use of 9DOF IMU ICM-20948 into DRehmFlight's stacks
since MPU9250 for 9DOF solutions are saturated by counterfeit breakout boards
and non working magnetometer chips.

Abstraction layer for the TDK InvenSense ICM-20948 9-DOF IMU.
Interface intentionally mirrors MPU9250.h so that imuMonitor.ino
can swap between both devices by changing one #define in quad.h:

  Implementation of the ICM20948 abstraction layer.
  See ICM20948.h for design notes and register map references.
 
  All getters return the same physical units as MPU9250.cpp so that
  imuMonitor.ino requires no code changes when swapping sensors:
    getAccelX/Y/Z_mss() → m/s²
    getGyroX/Y/Z_rads() → rad/s
    getMagX/Y/Z_uT()    → µT  (hard/soft-iron corrected)
 
  Register bank access pattern
  ─────────────────────────────
  The ICM-20948 has four user banks (0–3) selected by writing register
  0x7F on the chip.  Bank 0 contains WHO_AM_I, sensor data outputs, and
  EXT_SLV_SENS_DATA (the mirrored mag bytes).  Bank 2 contains accel/gyro
  configuration.  Bank 3 contains the I²C master for the AK09916.
 
  Every writeReg / readRegs call selects the target bank first, then
  immediately accesses the target register.  Because we are on a 400 kHz
  I²C bus this adds ~20 µs per call — acceptable at 2 kHz loop rate.

Implementations inspired based on MPU9250 library by:
brian.taylor@bolderflight.com
http://www.bolderflight.com

*/

#include "ICM20948.h"
#include "Arduino.h"
 
// ─── Static constant definition ───────────────────────────────────────────────
const float ICM20948::AK09916_MAG_SCALE = 0.15f;  // µT per LSB (16-bit mode)
 
// ════════════════════════════════════════════════════════════════════════════
//  Constructor
// ════════════════════════════════════════════════════════════════════════════
 
ICM20948::ICM20948(TwoWire &bus, uint8_t address)
    : _i2c(&bus), _address(address), _numBytes(0),
      _ax(0), _ay(0), _az(0),
      _gx(0), _gy(0), _gz(0),
      _hx(0), _hy(0), _hz(0), _t(0),
      _axcounts(0), _aycounts(0), _azcounts(0),
      _gxcounts(0), _gycounts(0), _gzcounts(0),
      _hxcounts(0), _hycounts(0), _hzcounts(0), _tcounts(0),
      _accelScale(0), _gyroScale(0),
      _magScaleX(AK09916_MAG_SCALE),
      _magScaleY(AK09916_MAG_SCALE),
      _magScaleZ(AK09916_MAG_SCALE),
      _accelRange(ACCEL_RANGE_16G), _gyroRange(GYRO_RANGE_2000DPS),
      _bandwidth(DLPF_BANDWIDTH_229HZ), _srd(0),
      _gxbD(0), _gybD(0), _gzbD(0),
      _counter(0), _framedelta(0), _delta(0),
      _hxfilt(0), _hyfilt(0), _hzfilt(0),
      _hxmax(0), _hymax(0), _hzmax(0),
      _hxmin(0), _hymin(0), _hzmin(0), _avgs(0) {}
 
// ════════════════════════════════════════════════════════════════════════════
//  begin() — initialise device
// ════════════════════════════════════════════════════════════════════════════
 
int ICM20948::begin() {
 
  // Start I²C if not already running; respect pre-configured clock.
  _i2c->begin();
  _i2c->setClock(_i2cRate);
 
  // ── 1. Soft-reset device ─────────────────────────────────────────────────
  if (writeReg(0, PWR_MGMT_1, DEV_RESET) < 0) return -1;
  delay(100);  // datasheet: ≥100 ms after reset
 
  // ── 2. Wake and select best clock source ─────────────────────────────────
  if (writeReg(0, PWR_MGMT_1, CLOCK_SEL_AUTO) < 0) return -2;
  delay(10);
 
  // ── 3. WHO_AM_I check ────────────────────────────────────────────────────
  if (whoAmI() != ICM20948_WHO_AM_I_VAL) return -3;
 
  // ── 4. Enable all axes ───────────────────────────────────────────────────
  if (writeReg(0, PWR_MGMT_2, ALL_AXES_EN) < 0) return -4;
 
  // ── 5. Default accel range ±16 g ─────────────────────────────────────────
  // ACCEL_CONFIG Bank 2: ACCEL_FS_SEL[1:0] | DLPF_EN | ACCEL_DLPFCFG[2:0]
  // Set FS = 16G, DLPF enabled, DLPFCFG = 0 (229 Hz 3dB bandwidth)
  if (writeReg(2, ACCEL_CONFIG, (uint8_t)(ACCEL_FS_16G | DLPF_EN | 0x00)) < 0) return -5;
  _accelScale = _G * 16.0f / 32767.5f;
  _accelRange = ACCEL_RANGE_16G;
 
  // ── 6. Default gyro range ±2000 °/s ──────────────────────────────────────
  // GYRO_CONFIG_1 Bank 2: GYRO_FS_SEL[1:0] | DLPF_EN | GYRO_DLPFCFG[2:0]
  if (writeReg(2, GYRO_CONFIG_1, (uint8_t)(GYRO_FS_2000DPS | DLPF_EN | 0x00)) < 0) return -6;
  _gyroScale = 2000.0f / 32767.5f * _d2r;
  _gyroRange = GYRO_RANGE_2000DPS;
 
  // ── 7. Maximum sample rate (divider = 0) ─────────────────────────────────
  // Gyro:  1125 Hz / (1 + GYRO_SMPLRT_DIV)  at ≤1kHz DLPF setting
  // Accel: 1125 Hz / (1 + ACCEL_SMPLRT_DIV) (12-bit divider, two registers)
  if (writeReg(2, GYRO_SMPLRT_DIV,    0x00) < 0) return -7;
  if (writeReg(2, ACCEL_SMPLRT_DIV_1, 0x00) < 0) return -8;
  if (writeReg(2, ACCEL_SMPLRT_DIV_2, 0x00) < 0) return -9;
  _srd = 0;
 
  // ── 8. Enable I²C master for AK09916 ─────────────────────────────────────
  if (writeReg(0, USER_CTRL, I2C_MST_EN) < 0) return -10;
  // I2C master clock: 0x07 → ~345.6 kHz (closest to 400 kHz available)
  if (writeReg(3, I2C_MST_CTRL, ICM20948::I2C_MST_CLK_400) < 0) return -11;
  delay(10);
 
  // ── 9. AK09916 soft-reset ────────────────────────────────────────────────
  if (writeAK09916Reg(AK09916_CNTL3, AK09916_RESET) < 0) return -12;
  delay(100);
 
  // ── 10. WHO_AM_I check for AK09916 ───────────────────────────────────────
  if (whoAmIAK09916() != AK09916_WHO_AM_I_VAL) return -13;
 
  // ── 11. Set AK09916 to continuous 100 Hz mode ────────────────────────────
  if (writeAK09916Reg(AK09916_CNTL2, AK09916_CONT_100HZ) < 0) return -14;
  delay(10);
 
  // ── 12. Configure I²C slave 0 to continuously read 9 bytes from AK09916 ──
  // (ST1[1] + HXL…HZH[6] + ST2[1] = 9 bytes starting at AK09916_ST1=0x10)
  // SLV0_ADDR: I2C_READ_FLAG | AK09916_I2C_ADDR (read mode)
  if (writeReg(3, I2C_SLV0_ADDR, (uint8_t)(I2C_READ_FLAG | AK09916_I2C_ADDR)) < 0) return -15;
  if (writeReg(3, I2C_SLV0_REG,  AK09916_ST1) < 0) return -16;
  // Read 9 bytes each master cycle, enable slave
  if (writeReg(3, I2C_SLV0_CTRL, (uint8_t)(I2C_SLV0_EN | 0x09)) < 0) return -17;
  delay(10);
 
  // ── 13. Gyro bias calibration ────────────────────────────────────────────
  if (calibrateGyro() < 0) return -18;
 
  return 1;  // success
}
 
// ════════════════════════════════════════════════════════════════════════════
//  Range configuration
// ════════════════════════════════════════════════════════════════════════════
 
int ICM20948::setAccelRange(AccelRange range) {
  // Preserve current DLPF settings; only update FS bits [2:1]
  uint8_t current;
  if (readRegs(2, ACCEL_CONFIG, 1, &current) < 0) return -1;
  current &= ~(0x06);  // clear FS_SEL bits [2:1]
  switch (range) {
    case ACCEL_RANGE_2G:  current |= ACCEL_FS_2G;  _accelScale = _G * 2.0f  / 32767.5f; break;
    case ACCEL_RANGE_4G:  current |= ACCEL_FS_4G;  _accelScale = _G * 4.0f  / 32767.5f; break;
    case ACCEL_RANGE_8G:  current |= ACCEL_FS_8G;  _accelScale = _G * 8.0f  / 32767.5f; break;
    case ACCEL_RANGE_16G: current |= ACCEL_FS_16G; _accelScale = _G * 16.0f / 32767.5f; break;
    default: return -2;
  }
  if (writeReg(2, ACCEL_CONFIG, current) < 0) return -3;
  _accelRange = range;
  return 1;
}
 
int ICM20948::setGyroRange(GyroRange range) {
  uint8_t current;
  if (readRegs(2, GYRO_CONFIG_1, 1, &current) < 0) return -1;
  current &= ~(0x06);  // clear FS_SEL bits [2:1]
  switch (range) {
    case GYRO_RANGE_250DPS:  current |= GYRO_FS_250DPS;  _gyroScale = 250.0f  / 32767.5f * _d2r; break;
    case GYRO_RANGE_500DPS:  current |= GYRO_FS_500DPS;  _gyroScale = 500.0f  / 32767.5f * _d2r; break;
    case GYRO_RANGE_1000DPS: current |= GYRO_FS_1000DPS; _gyroScale = 1000.0f / 32767.5f * _d2r; break;
    case GYRO_RANGE_2000DPS: current |= GYRO_FS_2000DPS; _gyroScale = 2000.0f / 32767.5f * _d2r; break;
    default: return -2;
  }
  if (writeReg(2, GYRO_CONFIG_1, current) < 0) return -3;
  _gyroRange = range;
  return 1;
}
 
int ICM20948::setDlpfBandwidth(DlpfBandwidth bandwidth) {
  // ICM-20948 DLPFCFG occupies bits [3:1] in GYRO_CONFIG_1 / ACCEL_CONFIG.
  // bit 0 is FCHOICE (DLPF enable).
  // The bandwidth enum values map directly to DLPFCFG field values.
  uint8_t gcfg, acfg;
  if (readRegs(2, GYRO_CONFIG_1, 1, &gcfg) < 0) return -1;
  if (readRegs(2, ACCEL_CONFIG,  1, &acfg) < 0) return -2;
  gcfg &= ~(0x0E);  // clear DLPFCFG[2:0] in bits [3:1]
  acfg &= ~(0x0E);
  uint8_t dlpfBits = ((uint8_t)bandwidth & 0x07) << 1;  // shift to bits [3:1]
  gcfg |= dlpfBits | DLPF_EN;
  acfg |= dlpfBits | DLPF_EN;
  if (writeReg(2, GYRO_CONFIG_1, gcfg) < 0) return -3;
  if (writeReg(2, ACCEL_CONFIG,  acfg) < 0) return -4;
  _bandwidth = bandwidth;
  return 1;
}
 
int ICM20948::setSrd(uint8_t srd) {
  // Gyro sample-rate divider: ODR = 1125 Hz / (1 + GYRO_SMPLRT_DIV) for DLPF≤1kHz mode
  if (writeReg(2, GYRO_SMPLRT_DIV, srd) < 0) return -1;
  // Accel uses a 12-bit divider split across two registers.
  // For symmetry with the MPU9250 setSrd(0) call, write srd into the LSB register.
  if (writeReg(2, ACCEL_SMPLRT_DIV_1, 0x00) < 0) return -2;
  if (writeReg(2, ACCEL_SMPLRT_DIV_2, srd)  < 0) return -3;
  _srd = srd;
  return 1;
}
 
// ════════════════════════════════════════════════════════════════════════════
//  Magnetometer calibration setters
// ════════════════════════════════════════════════════════════════════════════
 
void ICM20948::setMagCalX(float bias, float scale) { _hxb = bias; _hxs = scale; }
void ICM20948::setMagCalY(float bias, float scale) { _hyb = bias; _hys = scale; }
void ICM20948::setMagCalZ(float bias, float scale) { _hzb = bias; _hzs = scale; }
 
// ════════════════════════════════════════════════════════════════════════════
//  readSensor() — burst-read all sensor registers in one I²C transaction
// ════════════════════════════════════════════════════════════════════════════
 
int ICM20948::readSensor() {
 
  // ── 1. Read 12 bytes of accel (6) + gyro (6) from Bank 0 ACCEL_XOUT_H ───
  if (readRegs(0, ACCEL_XOUT_H, 12, _buffer) < 0) return -1;
 
  _axcounts = (int16_t)(((uint16_t)_buffer[0]  << 8) | _buffer[1]);
  _aycounts = (int16_t)(((uint16_t)_buffer[2]  << 8) | _buffer[3]);
  _azcounts = (int16_t)(((uint16_t)_buffer[4]  << 8) | _buffer[5]);
  _gxcounts = (int16_t)(((uint16_t)_buffer[6]  << 8) | _buffer[7]);
  _gycounts = (int16_t)(((uint16_t)_buffer[8]  << 8) | _buffer[9]);
  _gzcounts = (int16_t)(((uint16_t)_buffer[10] << 8) | _buffer[11]);
 
  // Scale and apply gyro bias
  _ax = _axcounts * _accelScale;
  _ay = _aycounts * _accelScale;
  _az = _azcounts * _accelScale;
  _gx = _gxcounts * _gyroScale - _gxb;
  _gy = _gycounts * _gyroScale - _gyb;
  _gz = _gzcounts * _gyroScale - _gzb;
 
  // ── 2. Read temperature (2 bytes) from Bank 0 TEMP_OUT_H ─────────────────
  if (readRegs(0, TEMP_OUT_H, 2, _buffer) < 0) return -2;
  _tcounts = (int16_t)(((uint16_t)_buffer[0] << 8) | _buffer[1]);
  _t = (float)_tcounts / _tempScale + _tempOffset;
 
  // ── 3. Read 9 bytes of mag data from EXT_SLV_SENS_DATA_00 ────────────────
  // Layout as configured in begin():  ST1(1) + HXL…HZH(6) + ST2(1) + pad(1)
  // byte 0 = ST1 (DRDY), bytes 1-6 = HXL HXH HYL HYH HZL HZH, byte 7 = ST2
  if (readRegs(0, EXT_SLV_SENS_DATA_00, 9, _buffer) < 0) return -3;
 
  // Check DRDY bit in ST1 (bit 0). If not set, mag data is stale but we
  // proceed with previous values rather than blocking — consistent with the
  // MPU9250 library which never blocks on DRDY.
  // Only unpack if DRDY is set to avoid latching stale data.
  if (_buffer[0] & 0x01) {
    _hxcounts = (int16_t)(((uint16_t)_buffer[2] << 8) | _buffer[1]);
    _hycounts = (int16_t)(((uint16_t)_buffer[4] << 8) | _buffer[3]);
    _hzcounts = (int16_t)(((uint16_t)_buffer[6] << 8) | _buffer[5]);
    // (buffer[7] = ST2 — must be consumed to unlatch; already read)
 
    // AK09916 sensitivity: 0.15 µT/LSB (fixed, no factory trim registers)
    // Apply hard/soft-iron correction supplied via setMagCalX/Y/Z
    _hx = (_hxcounts * AK09916_MAG_SCALE - _hxb) * _hxs;
    _hy = (_hycounts * AK09916_MAG_SCALE - _hyb) * _hys;
    _hz = (_hzcounts * AK09916_MAG_SCALE - _hzb) * _hzs;
  }
 
  return 1;
}
 
// ════════════════════════════════════════════════════════════════════════════
//  Getters — physical units matching MPU9250 library
// ════════════════════════════════════════════════════════════════════════════
 
float ICM20948::getAccelX_mss() { return _ax; }
float ICM20948::getAccelY_mss() { return _ay; }
float ICM20948::getAccelZ_mss() { return _az; }
float ICM20948::getGyroX_rads() { return _gx; }
float ICM20948::getGyroY_rads() { return _gy; }
float ICM20948::getGyroZ_rads() { return _gz; }
float ICM20948::getMagX_uT()    { return _hx; }
float ICM20948::getMagY_uT()    { return _hy; }
float ICM20948::getMagZ_uT()    { return _hz; }
float ICM20948::getTemperature_C() { return _t; }
 
// ════════════════════════════════════════════════════════════════════════════
//  Gyro calibration
// ════════════════════════════════════════════════════════════════════════════
 
int ICM20948::calibrateGyro() {
  _gxbD = 0; _gybD = 0; _gzbD = 0;
  for (size_t i = 0; i < _numSamples; i++) {
    if (readSensor() < 0) return -1;
    // readSensor subtracts _gxb/_gyb/_gzb, which are still 0 here, so
    // the raw rate read is _gx/_gy/_gz (no bias applied yet).
    // We must read the pre-bias-subtracted value from the raw getter.
    _gxbD += (_gxcounts * _gyroScale);
    _gybD += (_gycounts * _gyroScale);
    _gzbD += (_gzcounts * _gyroScale);
    delay(10);
  }
  _gxb = (float)(_gxbD / _numSamples);
  _gyb = (float)(_gybD / _numSamples);
  _gzb = (float)(_gzbD / _numSamples);
  return 1;
}
 
float ICM20948::getGyroBiasX_rads() { return _gxb; }
float ICM20948::getGyroBiasY_rads() { return _gyb; }
float ICM20948::getGyroBiasZ_rads() { return _gzb; }
void  ICM20948::setGyroBiasX_rads(float bias) { _gxb = bias; }
void  ICM20948::setGyroBiasY_rads(float bias) { _gyb = bias; }
void  ICM20948::setGyroBiasZ_rads(float bias) { _gzb = bias; }
 
// ════════════════════════════════════════════════════════════════════════════
//  Magnetometer calibration getters
// ════════════════════════════════════════════════════════════════════════════
 
float ICM20948::getMagBiasX_uT()      { return _hxb; }
float ICM20948::getMagScaleFactorX()  { return _hxs; }
float ICM20948::getMagBiasY_uT()      { return _hyb; }
float ICM20948::getMagScaleFactorY()  { return _hys; }
float ICM20948::getMagBiasZ_uT()      { return _hzb; }
float ICM20948::getMagScaleFactorZ()  { return _hzs; }
 
// ════════════════════════════════════════════════════════════════════════════
//  calibrateMag() — interactive hard/soft-iron estimation
//  Mirrors MPU9250::calibrateMag() algorithm and return convention.
// ════════════════════════════════════════════════════════════════════════════
 
int ICM20948::calibrateMag() {
  // Reset min/max trackers
  _hxmax = -1e9f; _hymax = -1e9f; _hzmax = -1e9f;
  _hxmin =  1e9f; _hymin =  1e9f; _hzmin =  1e9f;
  _hxfilt = 0; _hyfilt = 0; _hzfilt = 0;
  _counter = 0; _delta = 0;
 
  while (_counter < _maxCounts) {
    _framedelta = 0;
    readSensor();
    // Raw µT values before bias/scale (re-compute from counts)
    float hxRaw = _hxcounts * AK09916_MAG_SCALE;
    float hyRaw = _hycounts * AK09916_MAG_SCALE;
    float hzRaw = _hzcounts * AK09916_MAG_SCALE;
 
    _hxfilt = (_hxfilt * ((float)(_coeff - 1)) + hxRaw) / (float)_coeff;
    _hyfilt = (_hyfilt * ((float)(_coeff - 1)) + hyRaw) / (float)_coeff;
    _hzfilt = (_hzfilt * ((float)(_coeff - 1)) + hzRaw) / (float)_coeff;
 
    if (_hxfilt > _hxmax) { _framedelta += _hxfilt - _hxmax; _hxmax = _hxfilt; }
    if (_hyfilt > _hymax) { _framedelta += _hyfilt - _hymax; _hymax = _hyfilt; }
    if (_hzfilt > _hzmax) { _framedelta += _hzfilt - _hzmax; _hzmax = _hzfilt; }
    if (_hxfilt < _hxmin) { _framedelta += _hxmin - _hxfilt; _hxmin = _hxfilt; }
    if (_hyfilt < _hymin) { _framedelta += _hymin - _hyfilt; _hymin = _hyfilt; }
    if (_hzfilt < _hzmin) { _framedelta += _hzmin - _hzfilt; _hzmin = _hzfilt; }
 
    _delta += _framedelta;
    if (_delta > _deltaThresh) {
      _counter++;
      _delta = 0;
    }
    delay(10);
  }
 
  // Compute bias (hard-iron) and scale (soft-iron)
  _hxb = (_hxmax + _hxmin) / 2.0f;
  _hyb = (_hymax + _hymin) / 2.0f;
  _hzb = (_hzmax + _hzmin) / 2.0f;
 
  _avgs = ((_hxmax - _hxmin) + (_hymax - _hymin) + (_hzmax - _hzmin)) / 3.0f;
  _hxs  = _avgs / (_hxmax - _hxmin);
  _hys  = _avgs / (_hymax - _hymin);
  _hzs  = _avgs / (_hzmax - _hzmin);
 
  return 1;
}
 
// ════════════════════════════════════════════════════════════════════════════
//  Private helpers — bank-aware I²C access
// ════════════════════════════════════════════════════════════════════════════
 
int ICM20948::selectBank(uint8_t bank) {
  // REG_BANK_SEL = 0x7F.  BANK_SEL field is bits [5:4].
  _i2c->beginTransmission(_address);
  _i2c->write(REG_BANK_SEL);
  _i2c->write((uint8_t)(bank << 4));
  return (_i2c->endTransmission() == 0) ? 1 : -1;
}
 
int ICM20948::writeReg(uint8_t bank, uint8_t reg, uint8_t data) {
  if (selectBank(bank) < 0) return -1;
  _i2c->beginTransmission(_address);
  _i2c->write(reg);
  _i2c->write(data);
  return (_i2c->endTransmission() == 0) ? 1 : -1;
}
 
int ICM20948::readRegs(uint8_t bank, uint8_t reg, uint8_t count, uint8_t *dest) {
  if (selectBank(bank) < 0) return -1;
  _i2c->beginTransmission(_address);
  _i2c->write(reg);
  if (_i2c->endTransmission(false) != 0) return -2;   // repeated start
  _numBytes = _i2c->requestFrom((int)_address, (int)count);
  if (_numBytes != count) return -3;
  for (uint8_t i = 0; i < count; i++) {
    dest[i] = _i2c->read();
  }
  return 1;
}
 
// ── Auxiliary I²C master (AK09916 via internal bus) ──────────────────────────
 
int ICM20948::writeAK09916Reg(uint8_t reg, uint8_t data) {
  // Configure slave 0 for a single-byte write to AK09916
  // SLV0_ADDR: AK09916 address WITHOUT read flag (write direction)
  if (writeReg(3, I2C_SLV0_ADDR, AK09916_I2C_ADDR) < 0) return -1;
  if (writeReg(3, I2C_SLV0_REG,  reg)               < 0) return -2;
  if (writeReg(3, I2C_SLV0_DO,   data)               < 0) return -3;
  // Enable slave 0 to perform a 1-byte write this master cycle
  if (writeReg(3, I2C_SLV0_CTRL, (uint8_t)(I2C_SLV0_EN | 0x01)) < 0) return -4;
  delay(5);  // allow master cycle to complete
  return 1;
}
 
int ICM20948::readAK09916Regs(uint8_t reg, uint8_t count, uint8_t *dest) {
  // Re-configure slave 0 to read 'count' bytes from AK09916 starting at 'reg'
  if (writeReg(3, I2C_SLV0_ADDR, (uint8_t)(I2C_READ_FLAG | AK09916_I2C_ADDR)) < 0) return -1;
  if (writeReg(3, I2C_SLV0_REG,  reg)                                          < 0) return -2;
  if (writeReg(3, I2C_SLV0_CTRL, (uint8_t)(I2C_SLV0_EN | (count & 0x0F)))     < 0) return -3;
  delay(5);  // allow master cycle to latch data into EXT_SLV_SENS_DATA_00
  // Now read back from EXT_SLV_SENS_DATA_00 in Bank 0
  return readRegs(0, EXT_SLV_SENS_DATA_00, count, dest);
}
 
// ── WHO_AM_I helpers ──────────────────────────────────────────────────────────
 
uint8_t ICM20948::whoAmI() {
  uint8_t who = 0x00;
  readRegs(0, WHO_AM_I, 1, &who);
  return who;
}
 
uint8_t ICM20948::whoAmIAK09916() {
  uint8_t who = 0x00;
  readAK09916Regs(AK09916_WIA2, 1, &who);
  return who;
}
 