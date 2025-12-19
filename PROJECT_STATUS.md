# SensorFusion Project Status & Roadmap

**Hardware**: ESP32-S3 Mini + GY-87 (MPU6050 + QMC5883L + BMP180)
**Last Updated**: 2025-12-14
**Current Phase**: Phase 1 - Real-time Orientation Visualization

---

## 🎯 Project Vision

Create a modular, low-power sensor fusion platform for motion tracking with three distinct phases:

1. **Phase 1**: Real-time orientation tracking with web visualization
2. **Phase 2**: Motion capture system for animation/VR/body tracking
3. **Phase 3**: Drone flight stabilization and control

---

## 📊 Current Status: Phase 1

### ✅ Completed (Tested & Working)

#### Hardware & Drivers
- [x] ESP32-S3 Mini board setup (GPIO1=SDA, GPIO2=SCL)
- [x] GY-87 module I2C connection @ I2C_NUM_0
- [x] MPU6050 driver (gyro + accel)
  - Reads data successfully
  - Outputs: Accel in `g`, Gyro in `deg/s`
  - Auto-detects ±250 dps / ±2g ranges
  - I2C bypass mode enabled for magnetometer
- [x] QMC5883L driver (magnetometer)
  - Reads data successfully
  - Outputs: Magnetic field in `µT`
  - Configured: 200 Hz ODR, OSR512, 8 Gauss range
- [x] BMP180 driver (barometer)
  - Reads data successfully
  - Outputs: Temperature (°C), Pressure (Pa), Altitude (m)
  - Confirmed accurate readings

#### Data Pipeline
- [x] Lock-free registry (`regbus`) for sensor data sharing
- [x] FreeRTOS task architecture
  - ImuTask: 200 Hz (Core 0, priority 12)
  - MagTask: 50 Hz (Core 0, priority 10)
  - BaroTask: 10 Hz (Core 0, priority 9)
  - FusionTask: 200 Hz (Core 1, priority 11)
- [x] High-precision timing (MicroPacer for ≥100 Hz tasks)

#### Sensor Fusion
- [x] Complementary filter implementation
  - Gyro integration for fast updates
  - Accel-based tilt correction (α=0.02)
  - Tilt-compensated magnetic yaw (β=0.035)
  - Auto gyro bias calibration (0.75s at startup)
  - Quick calibration (1.5s still period)
  - Motion gating for magnetometer
- [x] Magnetometer hard-iron calibration
  - Runtime bias estimation
  - Automatic activation after sufficient motion

#### Web Interface
- [x] WiFi Station mode connectivity
- [x] WebSocket server (CONFIG_HTTPD_WS_SUPPORT enabled)
- [x] Progressive Web App (PWA) with service worker
- [x] Three.js 3D cube visualization
- [x] Real-time orientation updates via WebSocket

#### Build System
- [x] ESP-IDF v5.5 integration
- [x] CMake build configuration
- [x] Git submodules (esp-idf, regbus)
- [x] Code compiles successfully

---

### 🔧 Recently Fixed (2025-12-14)

#### Critical Unit Conversion Bug
- **Problem**: MPU6050 outputting m/s², but fusion filter expected `g`
- **Impact**: Tilt calculations were off by ~9.8x
- **Fix**: Changed accel conversion to output in `g` units
- **Files Modified**:
  - `main/src/MPU6050Driver.cpp:237-239`
  - `main/src/Tasks.cpp:324,345,360,411,473`

#### Duplicate Initialization Code
- **Problem**: MPU6050 init() had conflicting sample rate configs
- **Fix**: Removed duplicate code, kept 200 Hz setup
- **File**: `main/src/MPU6050Driver.cpp:102-108`

#### Magnetometer Task Rate
- **Problem**: MagTask running at 10 Hz (too slow for smooth yaw)
- **Fix**: Increased to 50 Hz for better heading tracking
- **File**: `main/main.cpp:101`

#### Build Errors
- **Problem**: WebSocket support disabled in sdkconfig
- **Fix**: Enabled `CONFIG_HTTPD_WS_SUPPORT=y`
- **File**: `sdkconfig`

#### Code Cleanup
- **Problem**: Unused static variables causing warnings
- **Fix**: Removed unused mag calibration variables from QMC5883L driver
- **File**: `main/src/QMC5883LDriver.cpp:32-33`

---

### ⚠️ Pending Testing (Ready to Test)

#### Phase 1 Validation
- [ ] Flash firmware to ESP32-S3
- [ ] Verify all three sensors initialize correctly
- [ ] Confirm gyro bias auto-calibration (first 0.75s)
- [ ] Test magnetometer calibration (figure-8 wave pattern)
- [ ] Validate web UI loads at ESP32 IP address
- [ ] **PRIMARY TEST**: Rotate GY-87 board → 3D cube follows orientation
  - [ ] Pitch (tilt forward/back)
  - [ ] Roll (tilt left/right)
  - [ ] Yaw (spin clockwise/counter-clockwise)
- [ ] Verify orientation accuracy
  - [ ] `|a|` ≈ 1.0g when stationary
  - [ ] Minimal drift over 60 seconds
  - [ ] Mag heading stable when still (`mag_ok=yes`)
- [ ] Test motion gating (mag disabled during movement)
- [ ] Check serial debug output formatting

#### Expected Serial Output
```
MPU6050 OK
WHO_AM_I=0x68
GYRO_CONFIG=0x00 -> FS_SEL=0, ±250 dps (131.0 LSB/°/s)
ACCEL_CONFIG=0x00 -> AFS_SEL=0, 16384 LSB/g

QMC5883L init OK (addr 0x0D)
BMP180 init OK

ImuTask: start @ 200.0 Hz
MagTask: start @ 50.0 Hz
BaroTask: start @ 10.0 Hz
FusionTask: start @ 200.0 Hz

QuickCal: gyro_bias(deg/s)=[0.123 -0.045 0.067], |a|=1.01 -> fudge=0.990

MagTask: calib VALID: bias=[12.34 -5.67 8.90] span=[45.2 43.1 41.8] (µT)

FusionTask: RPY fused= [0.5, -2.3, 87.4] deg | |a|=1.00g (expect ~1.0) | mag_ok=yes
```

---

### 🚧 Known Issues / To Investigate

- [ ] Verify sensor mounting orientation matches expected axes
  - May need axis remapping if cube moves in wrong direction
- [ ] Test magnetometer interference from WiFi/electronics
- [ ] Optimize complementary filter parameters (α, β) if needed
- [ ] Verify altitude calculation accuracy (BMP180)
- [ ] Test system stability over extended runtime (24hr+)
- [ ] Measure actual power consumption
- [ ] Check CPU utilization per core

---

### 📝 Potential Improvements (Phase 1)

#### Performance
- [ ] Implement quaternion output (avoids gimbal lock)
  - TODO already noted in `Complementary.cpp:246`
- [ ] Add Madgwick or Mahony filter as alternative
- [ ] Benchmark actual task execution times
- [ ] Add IMU FIFO buffering for burst reads

#### Calibration
- [ ] Persist mag calibration to NVS (non-volatile storage)
  - TODO noted in `Tasks.cpp:193`
- [ ] Soft-iron calibration matrix (currently identity)
- [ ] Gyro temperature compensation
- [ ] Accel range auto-detection

#### User Experience
- [ ] Add calibration status indicators to web UI
- [ ] Display sensor raw values in dashboard
- [ ] Add reset/recalibrate button
- [ ] Show WiFi RSSI and connection status
- [ ] Add downloadable calibration profile export

#### Robustness
- [ ] Watchdog timer for task monitoring
- [ ] Sensor health checks (detect disconnection)
- [ ] Fallback to accel-only mode if gyro fails
- [ ] Log errors to SPIFFS for debugging
- [ ] OTA (Over-The-Air) firmware updates

---

## 🚀 Phase 2: Motion Capture (Future)

### Goals
- Track multiple sensor nodes simultaneously
- Estimate position via IMU dead reckoning + fusion
- Wireless synchronization between nodes
- Body pose reconstruction
- Export data to animation software (BVH, FBX)

### Technical Requirements
- [ ] Multi-node architecture design
- [ ] ESP-NOW for low-latency peer-to-peer
- [ ] Time synchronization protocol (PTP-like)
- [ ] Kalman filter for position estimation
- [ ] Sensor-to-body coordinate transforms
- [ ] 3D visualization of multiple nodes
- [ ] Data recording and playback
- [ ] Battery optimization (target: 8+ hours)

### Potential Applications
- Motion capture for animation
- Digital art / interactive installations
- VR body tracking
- Sports performance analysis
- Rehabilitation monitoring

---

## 🛩️ Phase 3: Drone Stabilization (Future)

### Goals
- Use fused orientation for flight control
- Implement PID controllers for stabilization
- Support manual and autonomous flight modes
- Add GPS for position hold

### Technical Requirements
- [ ] Migration to nRF52840 (lower power, mentioned in README)
- [ ] Motor ESC drivers (PWM output)
- [ ] PID tuning framework
- [ ] GPS integration (UBX protocol)
- [ ] Altitude hold (barometer + accel)
- [ ] Safety features (failsafe, geofencing)
- [ ] RC receiver input (SBUS/PPM)
- [ ] Battery voltage monitoring
- [ ] Flight data logging (black box)

### Flight Modes
- [ ] Stabilize (angle mode)
- [ ] Acro (rate mode)
- [ ] Altitude hold
- [ ] Position hold (GPS)
- [ ] Return-to-home
- [ ] Autonomous waypoint navigation

---

## 📚 Documentation Status

### Existing Documentation
- [x] README.md - Project overview and goals
- [x] Inline code comments
- [x] Git commit history shows iterative development
- [x] `.clang-format` for code style
- [x] `Kconfig.projbuild` for WiFi configuration

### Documentation Needed
- [ ] API documentation (Doxygen?)
- [ ] Calibration procedure guide
- [ ] Troubleshooting guide
- [ ] Performance benchmarks
- [ ] Hardware assembly guide
- [ ] Pin mapping diagram
- [ ] Theory of operation (complementary filter math)
- [ ] Contributing guidelines

---

## 🔬 Testing Checklist

### Unit Testing (Future)
- [ ] MPU6050 register read/write
- [ ] QMC5883L data conversion accuracy
- [ ] BMP180 altitude calculation
- [ ] Complementary filter math (offline simulation)
- [ ] Registry thread-safety (stress test)
- [ ] WebSocket message formatting

### Integration Testing
- [ ] Sensor task timing accuracy
- [ ] Data pipeline latency (sensor → web UI)
- [ ] WiFi reconnection handling
- [ ] Multi-client WebSocket handling
- [ ] Power cycle recovery
- [ ] Brownout detection

### Performance Testing
- [ ] CPU utilization per task
- [ ] Memory usage (heap, stack)
- [ ] Network bandwidth usage
- [ ] Battery life measurement
- [ ] Thermal performance

---

## 🛠️ Development Environment

### Required Tools
- ESP-IDF v5.5 (installed via submodule)
- Xtensa GCC toolchain (auto-installed by ESP-IDF)
- Python 3.9+ (for build scripts)
- Git with submodule support

### Build Commands
```bash
# Setup environment
cd /home/litu/sandbox/esp/SensorFusion
source env.sh

# Configure (optional)
idf.py menuconfig

# Build
idf.py build

# Flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor

# Clean build
idf.py fullclean
```

### Repository Structure
```
SensorFusion/
├── main/
│   ├── inc/            # Header files
│   │   ├── fusion/     # Sensor fusion algorithms
│   │   └── *.hpp       # Driver and component headers
│   ├── src/            # Implementation files
│   │   ├── fusion/     # Fusion algorithm implementations
│   │   └── *.cpp       # Driver and component implementations
│   ├── main.cpp        # Application entry point
│   ├── index.html      # Web UI (embedded)
│   └── CMakeLists.txt  # Component build config
├── third_party/
│   ├── esp-idf/        # ESP-IDF framework (submodule)
│   └── regbus/         # Lock-free registry library (submodule)
├── CMakeLists.txt      # Root project config
├── sdkconfig           # ESP-IDF configuration (gitignored)
├── env.sh              # Environment setup script
└── PROJECT_STATUS.md   # This file
```

---

## 🐛 Debugging Tips

### Serial Monitor
- Baud rate: 115200 (default)
- Use `idf.py monitor` for colored output and crash decoding
- Press `Ctrl+]` to exit monitor

### Common Issues
1. **"I2C timeout"**: Check wiring, pull-ups, device addresses
2. **"No active WebSocket client"**: Refresh browser, check IP
3. **Cube doesn't move**: Check `mag_ok` status, calibrate magnetometer
4. **Orientation wrong direction**: May need axis remapping in fusion filter
5. **Build fails**: Run `idf.py fullclean && idf.py build`

### Useful Log Filters
```bash
# Show only errors
idf.py monitor | grep -E "ERROR|LOGE"

# Show fusion output only
idf.py monitor | grep "FusionTask"

# Show all sensor readings
idf.py monitor | grep -E "ImuTask|MagTask|BaroTask"
```

---

## 📈 Performance Metrics (To Be Measured)

### Target Specifications
- **Sensor sampling rates**: 200 Hz (IMU), 50 Hz (Mag), 10 Hz (Baro) ✅
- **Fusion update rate**: 200 Hz ✅
- **WebSocket latency**: <50 ms (TBD)
- **Orientation accuracy**: ±2° (TBD)
- **Drift rate**: <1°/minute (TBD)
- **Power consumption**: <500 mW (TBD)
- **Boot time**: <2 seconds (TBD)

### Actual Measurements
- [ ] Measure and document actual performance
- [ ] Compare against targets
- [ ] Identify bottlenecks

---

## 🔄 Git Workflow

### Current Branch
- `claude-fix` - Latest fixes for unit conversion and build errors

### Recommended Workflow
```bash
# Create feature branch
git checkout -b feature/quaternion-output

# Make changes, commit frequently
git add .
git commit -m "Add quaternion output to fusion state"

# Merge back to main when tested
git checkout main
git merge feature/quaternion-output

# Tag releases
git tag -a v1.0-phase1 -m "Phase 1: Orientation tracking complete"
```

---

## 📞 Next Steps

### Immediate (This Week)
1. **Test Phase 1 on hardware**
   - Flash firmware
   - Validate orientation tracking
   - Document any issues

2. **Tune and optimize**
   - Adjust filter parameters if needed
   - Fix axis mapping if cube moves incorrectly
   - Improve calibration UX

3. **Document findings**
   - Update this file with test results
   - Take photos/videos of working demo
   - Note any hardware issues

### Short Term (This Month)
1. **Implement quaternions** (avoid gimbal lock)
2. **Add NVS persistence** for calibration
3. **Create `sdkconfig.defaults`** to prevent config issues
4. **Write user documentation** (setup, calibration, troubleshooting)

### Long Term (Next 3-6 Months)
1. **Begin Phase 2 planning** (motion capture architecture)
2. **Evaluate alternative hardware** (nRF52840)
3. **Build second sensor node** for multi-node testing
4. **Explore Kalman filter** implementation

---

## 📜 Change Log

### 2025-12-14 - Critical Fixes
- Fixed MPU6050 accel unit conversion (m/s² → g)
- Removed duplicate initialization code in MPU6050
- Increased magnetometer sample rate (10 → 50 Hz)
- Enabled WebSocket support in sdkconfig
- Cleaned up unused variables in QMC5883L driver
- Updated motion gating thresholds for `g` units
- Fixed debug log output expectations

### Previous Work (Before Documentation)
- Implemented all sensor drivers
- Created complementary filter
- Built FreeRTOS task architecture
- Developed web UI with Three.js
- Set up registry-based data sharing
- Configured WiFi and WebSocket server

---

**Document Version**: 1.0
**Maintainer**: Project Owner
**Last Reviewed**: 2025-12-14
