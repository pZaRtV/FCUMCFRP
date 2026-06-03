# Teensy 4.0 Resource Usage Analysis — FCUMCFRP_B_1.4.6L

**Platform**: ARM Cortex-M7 @ 600 MHz, single-precision FPU  
**Flash**: 2048 KB (Teensy 4.0) / 8192 KB (Teensy 4.1)  
**RAM**: 512 KB DTCM + 512 KB OCRAM  

---

## 1. `invSqrt()` Performance Analysis

### Call frequency per loop iteration (2 kHz)

| Caller | Calls per loop | Context |
|--------|---------------|---------|
| **Madgwick 9DOF** (main, if MPU9250/ICM20948) | 4 | accel norm, mag norm, step norm, quat norm |
| **Madgwick6DOF** (main, if MPU6050) | 3 | accel norm, step norm, quat norm |
| **MadgwickMonitor 9DOF** (imuMonitor) | 4 | same as above, but for monitor |
| **MadgwickMonitor 6DOF** (imuMonitor) | 3 | fallback if no mag |

**Current config** (MPU6050 main + ICM20948 monitor):
- Main Madgwick6DOF: **3 calls/loop**  
- Monitor Madgwick9DOF: **4 calls/loop** (every 5ms = every 10th loop)
- **Total**: ~3.4 calls/loop average

### Cycle comparison per `invSqrt` call

| Implementation | Cycles | Accuracy | Notes |
|---------------|--------|----------|-------|
| `1.0/sqrtf(x)` **(current)** | ~50-80 | IEEE exact | ⚠ `1.0` is double → software double division! |
| `1.0f/sqrtf(x)` **(recommended)** | ~28 | IEEE exact | Hardware VSQRT.F32 + VDIV.F32 |
| Quake fast invSqrt (1 iteration) | ~15-18 | ~0.175% error | Integer trick + 1 Newton-Raphson |
| Quake fast invSqrt (2 iterations) | ~20-24 | ~0.0001% error | Integer trick + 2 Newton-Raphson |
| ARM VRSQRTE + VRSQRTS | ~8-12 | ~0.4% → refined | Compiler may auto-generate with `-ffast-math` |

### Recommendation

**Use `1.0f/sqrtf(x)`**. Here's why:

1. **The double promotion is the real bug** — `1.0` forces software double emulation (~50-80 cycles)
2. **`1.0f/sqrtf(x)` uses pure hardware** — VSQRT.F32 (14 cycles) + VDIV.F32 (14 cycles) = ~28 cycles
3. **Quake trick saves ~10 cycles** but introduces approximation error into quaternion normalization
4. **Budget impact**: 3.4 calls × 28 cycles = ~95 cycles/loop — that's **0.03%** of the 300,000-cycle budget
5. **For a research platform**, IEEE-exact math is preferable over shaving 10 cycles

The comment in the code is correct — Teensy *is* fast enough. Just fix the `1.0` → `1.0f`.

---

## 2. CPU Cycle Budget (per 2 kHz loop iteration)

**Available**: 600 MHz / 2000 Hz = **300,000 cycles per loop**

| Function | Est. Cycles | % Budget | Frequency |
|----------|------------|----------|-----------|
| `getIMUdata()` — I2C read + LP filter | ~15,000 | 5.0% | Every loop |
| `Madgwick6DOF()` — main filter | ~2,000 | 0.7% | Every loop |
| `controlANGLE2()` — PID (cascaded) | ~800 | 0.3% | Every loop |
| `controlMixer()` | ~200 | 0.1% | Every loop |
| `scaleCommands()` | ~150 | 0.05% | Every loop |
| `getCommands()` — IBUS read | ~500 | 0.2% | Every loop |
| `failSafe()` | ~100 | 0.03% | Every loop |
| `throttleCut()` | ~50 | 0.02% | Every loop |
| Motor PWM output (6 channels) | ~300 | 0.1% | Every loop |
| Servo output (4 channels) | ~200 | 0.07% | Every loop |
| **Monitor IMU read** (Wire1 I2C) | ~12,000 | 4.0% | Every loop |
| **MadgwickMonitor9DOF** | ~2,500 | 0.8% | Every loop |
| **detectEvents()** | ~300 | 0.1% | Every loop |
| **UDP packet build + send** | ~5,000 | 1.7% | Every 10th loop (5ms) |
| `loopBlink()` | ~20 | 0.01% | Every loop |
| `loopRate()` — busy-wait | remainder | — | Every loop |

### Estimated total: ~39,000 cycles (~13% of budget)

**Headroom: ~87%** — the Teensy 4.0 is barely breaking a sweat. The `loopRate(2000)` busy-wait at the end of each loop is consuming the vast majority of CPU time doing nothing.

> [!NOTE]
> The I2C reads (main + monitor) are the dominant cost at ~9% combined. If you ever need more headroom, DMA I2C transfers would be the biggest win — not invSqrt optimization.

---

## 3. RAM Usage Estimate

### Global Variables (DTCM)

| Category | Variables | Est. Bytes |
|----------|----------|------------|
| IMU readings (main) | AccX/Y/Z, GyroX/Y/Z, MagX/Y/Z | 36 |
| IMU readings (monitor) | Same set for monitor | 36 |
| Quaternion state (main) | q0-q3, roll/pitch/yaw_IMU | 28 |
| Quaternion state (monitor) | q0_mon-q3_mon, mon_roll/pitch/yaw | 28 |
| PID state | errors, integrals, derivatives (3 axes × ~8 vars) | 96 |
| Radio channels | channel_1-6_pwm + _prev + _raw + failsafe | 96 |
| Motor/servo commands | m1-6 + s1-7 (scaled + PWM) | 104 |
| Calibration errors | main + monitor (6 each) | 48 |
| Timing variables | dt, current_time, prev_time, etc. | 32 |
| PID gains | Kp/Ki/Kd × 3 axes × 2 modes | 72 |
| Filter coefficients | B_madgwick, B_accel, B_gyro, B_mag | 16 |
| UDP packet struct | IMUDataPacket | 117 |
| Ethernet objects | EthernetUDP + W5500 buffers | ~2,048 |
| IBUS object + buffers | ibus + ibusChannels | ~128 |
| PWMServo objects | 6 motors + 4 servos | ~200 |
| ICM20948 driver | _buffer, scale caches, cal state | ~256 |
| MPU6050 driver | I2Cdev buffers | ~128 |
| Event detection state | prev_* values, flags | 64 |
| **Stack** (estimated peak) | Madgwick local vars are deepest | ~2,000 |

### Estimated total RAM: ~5,500 bytes (~1.1% of 512 KB DTCM)

**Headroom: ~99%** — RAM is not a concern.

---

## 4. Flash Usage Estimate

| Component | Lines | Est. Flash (KB) |
|-----------|-------|----------------|
| FCUMCFRP_B_1.4.6L.ino | 2,463 | ~40 |
| imuMonitor.ino | 682 | ~12 |
| radioComm.ino | 206 | ~4 |
| MPU6050 library | 3,180 + headers | ~50 |
| ICM20948 library | 532 + header | ~12 |
| iBus library | 165 + header | ~4 |
| Ethernet library (W5500) | (system) | ~20 |
| PWMServo library | (system) | ~4 |
| Wire/SPI libraries | (system) | ~8 |
| Arduino core + startup | (system) | ~30 |

### Estimated total Flash: ~184 KB (~9% of 2048 KB)

**Headroom: ~91%** — Flash is not a concern.

> [!TIP]
> The MPU6050 library includes MotionApps DMP headers (~3 files, 1,937 lines) that are never used in this project. They get compiled but the linker likely strips unused code. If flash ever becomes tight, removing those headers would save ~30 KB.

---

## 5. Double-Promotion Hotspots to Fix

Beyond `invSqrt`, these double-literal assignments run at init (not in the hot loop), but for consistency:

| File | Line | Current | Fix |
|------|------|---------|-----|
| **invSqrt** | 2738 | `1.0/sqrtf(x)` | `1.0f/sqrtf(x)` |
| B_madgwick | 171 | `= 0.04;` | `= 0.04f;` |
| B_accel | 172 | `= 0.14;` | `= 0.14f;` |
| PID gains | 224-240 | `= 0.05;` etc. | Add `f` suffix |
| LP filter b | 1918 | `float b = 0.7;` | `= 0.7f;` |
| SBUS scale | 1853 | `float scale = 0.615;` | `= 0.615f;` |

> [!IMPORTANT]
> **Only `invSqrt` is in the hot loop** (2 kHz × 3-4 calls). All others are init-time or one-shot assignments where double promotion has zero performance impact. Fix `invSqrt` first; the rest are style consistency.

---

## Summary

| Resource | Used | Available | Utilization |
|----------|------|-----------|-------------|
| **CPU** | ~39K cycles/loop | 300K cycles/loop | **~13%** |
| **RAM** | ~5.5 KB | 512 KB | **~1.1%** |
| **Flash** | ~184 KB | 2048 KB | **~9%** |

The Teensy 4.0 has **massive headroom** for this project. The `1.0f/sqrtf(x)` fix is the only performance-relevant change — and even that saves ~70 cycles/loop (0.02% budget) by avoiding software double division. The real justification is correctness (staying in the hardware FPU path), not performance.
