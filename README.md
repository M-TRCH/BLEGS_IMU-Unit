# IMU Unit for Quadruped Robot Balance Control

High-performance IMU unit with binary protocol streaming @ 100 Hz, compatible with Motor Protocol v1.2

[![PlatformIO](https://img.shields.io/badge/PlatformIO-STM32-orange)](https://platformio.org/)
[![Sensor](https://img.shields.io/badge/Sensor-BNO055-blue)](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)
[![Protocol](https://img.shields.io/badge/Protocol-Binary%20CRC16-green)](docs/IMU_DOCUMENTATION.md)

---

## ⚡ Quick Start

### 1. Hardware Setup

**Required Components:**
- STM32F103C8 (Blue Pill)
- BNO055 IMU Sensor
- USB-Serial adapter

**Wiring:**
```
STM32F103C8          BNO055
    PB9    ───────   SDA
    PB8    ───────   SCL
    3.3V   ───────   VIN
    GND    ───────   GND

UART (to Computer):
    PA9    ───────   TX (to USB-Serial RX)
    PA10   ───────   RX (to USB-Serial TX)
```

### 2. Flash Firmware

```bash
# Install PlatformIO CLI (if not installed)
pip install platformio

# Build and upload
cd BLEGS_IMU-Unit
pio run --target upload
```

### 3. Test Connection

```bash
# Install Python dependencies
pip install pyserial

# Edit COM port in test_imu.py (line 191)
PORT = 'COM44'  # Change to your port

# Run test client
python tools/test_imu.py
```

**Expected Output:**
```
[00123] R:   0.00° P:   0.00° Y:   0.00° | 100Hz | Cal:✓ Err:0 Gap:0
```

---

## 📋 Features

### Core Features
- ✅ **100 Hz streaming** - Real-time orientation data @ 10ms interval
- ✅ **Binary protocol** - Efficient CRC16-validated packets
- ✅ **Set Zero command** - Software calibration for any orientation
- ✅ **Cross-platform** - Windows, Linux, macOS support
- ✅ **Error tracking** - CRC errors and sequence gap detection
- ✅ **Auto recovery** - Robust packet resync and timeout handling

### Performance
- **Bandwidth**: 1.39% @ 921,600 baud (1,600 bytes/sec)
- **CPU Load**: ~17% (83% available for future features)
- **Latency**: < 1.7 ms per packet transmission
- **Precision**: 0.01° (degrees × 100 in int16)

### Recent Improvements (v1.1)
- 🔧 Fixed critical yaw overflow bug (int16 max exceeded at 327.67°)
- 🔧 Fixed Set Zero normalization for all axes
- 🔧 Fixed CRC packet loss for zero-payload commands
- 🔧 Added buffer overflow protection
- 🎯 Improved packet resync performance (16 bytes/cycle)

---

## 📦 Project Structure

```
BLEGS_IMU-Unit/
├── src/
│   ├── main.cpp              # Main firmware (100 Hz loop)
│   └── protocol.cpp          # Binary protocol implementation
├── include/
│   └── protocol.h            # Protocol definitions
├── tools/
│   └── test_imu.py           # Python test client (cross-platform)
├── docs/
│   └── IMU_DOCUMENTATION.md  # Full protocol documentation
├── platformio.ini            # PlatformIO configuration
└── README.md                 # This file
```

---

## 🎮 Usage

### Python Client Commands

```bash
z           → Set Zero (current orientation as reference)
Ctrl+C      → Exit
```

**Note**: On Windows/Unix, just press 'z' (no Enter needed) - the client uses non-blocking keyboard input.

### Display Metrics

```
[00123] R:   0.00° P:   0.00° Y:   0.00° | 100Hz | Cal:✓ Err:0 Gap:0
        │     │       │       │       │      │      │     └─ Sequence gaps
        │     │       │       │       │      │      └─ CRC errors
        │     │       │       │       │      └─ Calibration status
        │     │       │       │       └─ Actual packet rate
        │     │       │       └─ Yaw angle
        │     │       └─ Pitch angle
        │     └─ Roll angle
        └─ Sequence number (0-65535)
```

---

## 🔧 Integration Example

### Python API

```python
import serial
from tools.test_imu import receive_packet, parse_imu_data, send_set_zero

# Connect to IMU
port = serial.Serial('COM44', 921600)
time.sleep(0.5)
port.reset_input_buffer()

# Set zero point
send_set_zero(port)

# Read data loop
while True:
    result = receive_packet(port, timeout=0.015)
    if result:
        pkt_type, payload = result
        if pkt_type == 0x85:  # FB_IMU_DATA
            data = parse_imu_data(payload)
            
            # Use orientation data
            roll = data['roll']     # -180.00 to +180.00
            pitch = data['pitch']   # -180.00 to +180.00
            yaw = data['yaw']       # -180.00 to +180.00
            
            # Your control logic here
            balance_robot(roll, pitch)
```

---

## 📊 Protocol Overview

### Packet Types

| Type | Direction | Name | Size | Frequency | Description |
|------|-----------|------|------|-----------|-------------|
| 0x85 | IMU → PC | FB_IMU_DATA | 16 bytes | 100 Hz | Euler angles (R,P,Y) |
| 0x87 | IMU → PC | FB_IMU_CALIBRATION | 11 bytes | 5 sec | Sensor calibration status |
| 0x06 | PC → IMU | CMD_SET_ZERO | 6 bytes | On demand | Set zero reference |

### Packet Structure

```
┌─────┬─────┬──────┬────────┬─────────┬─────┐
│ FE  │ EE  │ Type │ Length │ Payload │ CRC │
│ (1) │ (1) │ (1)  │  (1)   │  (var)  │ (2) │
└─────┴─────┴──────┴────────┴─────────┴─────┘
```

**Full documentation**: [docs/IMU_DOCUMENTATION.md](docs/IMU_DOCUMENTATION.md)

---

## 🛠️ Troubleshooting

### No data received?

1. Check COM port in `test_imu.py` (line 191)
2. Verify baud rate: **921,600** (both firmware and Python)
3. Reset MCU (press reset button or re-upload)
4. Check wiring (especially I2C connections)

### Set Zero not working?

1. **Update to v1.1+ firmware** (contains critical fixes)
2. Wait for "✓" calibration indicator
3. Verify acknowledgment message appears
4. Test all axes rotate ±180° correctly

### CRC errors (Err > 0)?

1. Use high-quality USB cable (shorter is better)
2. Reduce USB latency timer to 1ms (Device Manager → Advanced)
3. Check for electromagnetic interference
4. Verify baud rate matches exactly

### Low packet rate (< 100 Hz)?

1. Close other programs using CPU
2. Check Python timeout = 0.015s
3. Verify I2C clock speed = 400 kHz

---

## 📚 Documentation

- **Full Protocol Documentation**: [docs/IMU_DOCUMENTATION.md](docs/IMU_DOCUMENTATION.md)
- **BNO055 Datasheet**: [Bosch Sensortec](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)
- **PlatformIO STM32**: [Platform Docs](https://docs.platformio.org/en/latest/platforms/ststm32.html)

---

## 🔄 Version History

### v1.1 (2026-02-10) - **Current**
- ✅ Fixed yaw overflow bug (int16 max exceeded)
- ✅ Fixed Set Zero normalization for all axes
- ✅ Fixed CRC packet loss for CMD_SET_ZERO
- ✅ Added buffer overflow protection
- ✅ Cross-platform keyboard support (Windows/Linux/Mac)
- ✅ Error tracking: CRC errors + sequence gaps
- ✅ Improved packet resync (16 bytes/cycle)

### v1.0 (2025-12-21)
- Initial release
- 100 Hz binary streaming
- CRC16 validation
- Basic Set Zero command

---

## 📝 License

This project is part of the BLEGS Quadruped Robot Control System.

**Author**: M-TRCH  
**Date**: 2026-02-10  
**Compatible with**: Motor Protocol v1.2

---

## 🤝 Contributing

For bug reports or feature requests, please document:
1. Firmware version (check `main.cpp` header)
2. Error messages or unexpected behavior
3. Test conditions (orientation, calibration status)
4. Serial logs if available

---

## ⚠️ Important Notes

- Always wait for full calibration (Cal:✓) before using
- Set Zero offset persists until power cycle
- All angles normalized to ±180° range
- Recommended to Set Zero with robot on level surface
- BNO055 requires calibration dance (see [docs](docs/IMU_DOCUMENTATION.md#calibration-tips))

---

**Ready to integrate?** Start with [Quick Start](#-quick-start) or read the [Full Documentation](docs/IMU_DOCUMENTATION.md)
