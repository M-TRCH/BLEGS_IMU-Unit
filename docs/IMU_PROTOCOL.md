# IMU Unit - Binary Protocol Documentation

## ภาพรวม (Overview)

IMU Unit ใช้ **Binary Protocol** ที่เข้ากันได้กับ Motor Protocol v1.2 สำหรับการสื่อสารความเร็วสูงกับคอมพิวเตอร์หลัก โดยใช้:
- **UART @ 921,600 baud** - High-speed serial communication
- **One-way streaming @ 100 Hz** - IMU ส่งข้อมูลไปยังคอมพิวเตอร์อย่างต่อเนื่อง
- **CRC16 validation** - ตรวจสอบความถูกต้องของข้อมูล
- **Bidirectional commands** - รองรับคำสั่งจากคอมพิวเตอร์ (เช่น Set Zero)

---

## 🔧 Hardware Configuration

### IMU Sensor
- **Sensor**: BNO055 (9-DOF Absolute Orientation)
- **I2C Address**: 0x28
- **Sample Rate**: 100 Hz (fusion data)
- **Data Output**: Euler angles (Roll, Pitch, Yaw)

### Microcontroller Pinout (STM32)
```cpp
Serial (UART):
  - RX: PA10
  - TX: PA9
  
I2C (BNO055):
  - SDA: PB9
  - SCL: PB8
```

---

## 📡 Protocol Structure

### Packet Format

```
┌──────────┬──────────┬─────────────┬─────────────┬─────────┬─────────┐
│ Header 1 │ Header 2 │ Packet Type │ Payload Len │ Payload │  CRC16  │
│  (0xFE)  │  (0xEE)  │   (1 byte)  │   (1 byte)  │  (var)  │ (2 byte)│
└──────────┴──────────┴─────────────┴─────────────┴─────────┴─────────┘
```

### Protocol Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `PROTOCOL_HEADER1` | 0xFE | First header byte |
| `PROTOCOL_HEADER2` | 0xEE | Second header byte |
| `IMU_UNIT_ID` | 0xFF | Reserved ID for IMU unit |
| `BAUD_RATE` | 921600 | High-speed UART |

---

## 📦 Packet Types

### Commands (PC → IMU)

| ID | Name | Description | Payload |
|----|------|-------------|---------|
| `0x06` | CMD_SET_ZERO | ตั้งค่าจุดศูนย์ (Zero Calibration) | ไม่มี (0 bytes) |

### Feedback (IMU → PC)

| ID | Name | Description | Payload Size | Frequency |
|----|------|-------------|--------------|-----------|
| `0x85` | FB_IMU_DATA | Euler angles streaming | 10 bytes | 100 Hz |
| `0x87` | FB_IMU_CALIBRATION | Calibration status | 5 bytes | Every 5 sec |

---

## 📨 Packet Details

### FB_IMU_DATA (0x85) - Main Data Stream

**Frequency**: 100 Hz (every 10ms)

**Payload Structure** (10 bytes):
```
┌──────────┬──────┬───────┬─────┬──────────┬────────┐
│ Unit ID  │ Roll │ Pitch │ Yaw │ Sequence │ Status │
│ (uint8)  │(int16)│(int16)│(int16)│ (uint16) │(uint8) │
│  1 byte  │2 byte│2 byte │2 byte│  2 bytes │1 byte  │
└──────────┴──────┴───────┴─────┴──────────┴────────┘
```

**Data Format**:
- **Roll, Pitch, Yaw**: Signed 16-bit integer (degrees × 100)
  - Range: -18000 to +18000 (= -180.00° to +180.00°)
  - Precision: 0.01 degree
- **Sequence**: Packet counter (0-65535, wraps around)
- **Status**: Calibration flags (bitfield)

**Example Packet**:
```
FE EE 85 0A FF 1194 F38C 0000 0123 0F [CRC_H] [CRC_L]
         │  │  │    │    │    │    │
         │  │  │    │    │    │    └─ Status: 0x0F (all calibrated)
         │  │  │    │    │    └────── Sequence: 0x0123 (291)
         │  │  │    │    └─────────── Yaw: 0.00°
         │  │  │    └──────────────── Pitch: -10.00° (0xF38C)
         │  │  └───────────────────── Roll: 45.00° (0x1194)
         │  └──────────────────────── Unit ID: 0xFF
         └─────────────────────────── Payload length: 10 bytes
```

**Status Flags** (bitfield):
```
Bit 0 (0x01): IMU_STATUS_CALIBRATED    - ระบบ calibrated ครบ
Bit 1 (0x02): IMU_STATUS_GYRO_CAL      - Gyroscope calibrated
Bit 2 (0x04): IMU_STATUS_ACCEL_CAL     - Accelerometer calibrated
Bit 3 (0x08): IMU_STATUS_MAG_CAL       - Magnetometer calibrated
Bit 7 (0x80): IMU_STATUS_ERROR         - เกิด error
```

---

### FB_IMU_CALIBRATION (0x87) - Calibration Status

**Frequency**: Every 5 seconds

**Payload Structure** (5 bytes):
```
┌──────────┬────────┬─────────┬───────────┬─────────┐
│ Unit ID  │ System │  Gyro   │   Accel   │   Mag   │
│ (uint8)  │ (uint8)│ (uint8) │  (uint8)  │ (uint8) │
└──────────┴────────┴─────────┴───────────┴─────────┘
```

**Calibration Levels**: 0 (not calibrated) to 3 (fully calibrated)

**Example**:
```
FE EE 87 05 FF 03 03 03 03 [CRC_H] [CRC_L]
         │  │  │  │  │  └─ Magnetometer: 3/3
         │  │  │  │  └──── Accelerometer: 3/3
         │  │  │  └─────── Gyroscope: 3/3
         │  │  └────────── System: 3/3
         │  └───────────── Unit ID: 0xFF
         └──────────────── Payload length: 5 bytes
```

---

### CMD_SET_ZERO (0x06) - Zero Calibration

**Direction**: PC → IMU  
**Payload**: None (0 bytes)

**Purpose**: ตั้งค่าตำแหน่งปัจจุบันเป็นจุดศูนย์อ้างอิง

**Behavior**:
1. IMU อ่านค่า Euler angles ปัจจุบัน
2. เก็บค่าเป็น offset (roll_offset, pitch_offset, yaw_offset)
3. ข้อมูลที่ส่งต่อจากนั้นจะถูกลบ offset
4. ส่ง FB_IMU_CALIBRATION กลับเป็น acknowledgment

**Example Packet**:
```
FE EE 06 00 [CRC_H] [CRC_L]
         │  └─ No payload
         └──── CMD_SET_ZERO
```

**Response**: IMU ส่ง FB_IMU_CALIBRATION packet กลับทันที

---

## 🧪 การใช้งาน (Usage)

### 1. Python Client

#### Installation
```bash
pip install pyserial
```

#### Basic Usage
```bash
# แก้ไข PORT ใน test_imu.py
PORT = 'COM44'  # เปลี่ยนเป็น port ของคุณ

# รันโปรแกรม
python tools/test_imu.py
```

#### Commands
```
z + Enter    → ตั้งค่า Zero (Set current orientation as zero)
Ctrl+C       → ออกจากโปรแกรม
```

#### Output Example
```
============================================================
IMU Unit - Binary Protocol Test Client
============================================================
Commands:
  Press 'z' + Enter: Set current orientation as zero
  Press Ctrl+C: Exit
============================================================
Connected to COM44 @ 921600 baud

Receiving IMU data stream...
------------------------------------------------------------
[00123] R: 45.23° P:-12.45° Y:180.67° | 100Hz | Cal:✓
```

---

### 2. Python API Reference

#### Send SET_ZERO Command
```python
import serial
from test_imu import send_set_zero

port = serial.Serial('COM44', 921600)
send_set_zero(port)  # ตั้งค่า zero point
```

#### Receive IMU Data
```python
from test_imu import receive_packet, parse_imu_data

result = receive_packet(port, timeout=0.02)
if result:
    pkt_type, payload = result
    if pkt_type == 0x85:  # FB_IMU_DATA
        data = parse_imu_data(payload)
        print(f"Roll: {data['roll']:.2f}°")
        print(f"Pitch: {data['pitch']:.2f}°")
        print(f"Yaw: {data['yaw']:.2f}°")
```

#### Custom Integration Example
```python
import serial
import time
from test_imu import receive_packet, parse_imu_data

port = serial.Serial('COM44', 921600)
time.sleep(0.5)

while True:
    result = receive_packet(port, timeout=0.015)
    if result:
        pkt_type, payload = result
        if pkt_type == 0x85:
            data = parse_imu_data(payload)
            
            # ใช้ข้อมูล IMU ในระบบควบคุม
            roll = data['roll']
            pitch = data['pitch']
            
            # Control logic here...
            balance_robot(roll, pitch)
```

---

## ⚡ Performance Analysis

### Timing Breakdown (@ 100 Hz)

```
Per Cycle (10,000 µs):
┌─────────────────────────────────┬──────────┐
│ Operation                       │ Time     │
├─────────────────────────────────┼──────────┤
│ Get Euler data (I2C)           │ ~1,500 µs│
│ Data conversion (int16)        │    ~20 µs│
│ Send packet (16 bytes @ 921600)│   ~176 µs│
│ Calibration read (1/100 cyc)   │    ~20 µs│
│ Command check                  │    ~10 µs│
│ Idle time                      │ ~8,274 µs│
└─────────────────────────────────┴──────────┘

Bandwidth: 176 µs / 10,000 µs = 1.76%
CPU Load: ~17% (sensor reads + processing)
Margin: 83% available for future features
```

### Bandwidth Utilization

```
Data Rate: 100 packets/sec × 16 bytes = 1,600 bytes/sec
UART Speed: 921,600 baud = 115,200 bytes/sec (theoretical)
Utilization: 1,600 / 115,200 = 1.39%
```

### Packet Efficiency

| Metric | Value | Notes |
|--------|-------|-------|
| **Packet Size** | 16 bytes | Headers(4) + Payload(10) + CRC(2) |
| **Payload Ratio** | 62.5% | 10/16 bytes |
| **TX Time** | 176 µs | @ 921,600 baud |
| **Overhead** | 1.76% | of 10ms cycle |

---

## 🔍 CRC16 Calculation

### Algorithm
- **Polynomial**: 0x1021 (CRC-CCITT)
- **Initial value**: 0xFFFF
- **Input**: Packet Type + Payload Length + Payload

### Python Implementation
```python
def calculate_crc16(data):
    crc = 0xFFFF
    
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    
    return crc
```

### C++ Implementation
```cpp
uint16_t calculateCRC16(const uint8_t* data, uint8_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (uint8_t i = 0; i < length; i++)
    {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    
    return crc;
}
```

---

## 🛠️ Troubleshooting

### ปัญหา: ไม่ได้รับข้อมูล

**Symptoms**: Python client ไม่แสดงข้อมูล

**Solutions**:
1. ตรวจสอบ COM port
   ```python
   PORT = 'COM44'  # ตรวจสอบใน Device Manager
   ```

2. ตรวจสอบ baud rate
   ```python
   BAUD_RATE = 921600  # ต้องตรงกับ MCU
   ```

3. ตรวจสอบสาย USB
   - ใช้สาย USB ที่ดี (รองรับ high-speed)
   - ลองเสียบ port อื่น

4. Reset MCU
   - กด reset button บน board
   - หรือ upload โค้ดใหม่

---

### ปัญหา: CRC Failed

**Symptoms**: เห็น error message "CRC mismatch"

**Causes**:
- Noise on UART line
- Baud rate mismatch
- USB latency

**Solutions**:
1. ตรวจสอบ baud rate (MCU และ PC ต้องเท่ากัน)
2. ลด USB Latency Timer เป็น 1ms:
   - Device Manager → Ports → Properties → Advanced
3. ใช้สาย USB ที่สั้นและมีคุณภาพดี
4. เพิ่ม pull-up resistor บน TX/RX line

---

### ปัญหา: Packet Rate ต่ำกว่า 100 Hz

**Symptoms**: แสดง "95 Hz" หรือต่ำกว่า แทนที่จะเป็น "100 Hz"

**Causes**:
- Python timeout สูงเกินไป
- CPU overload
- I2C communication slow

**Solutions**:
1. ตรวจสอบ timeout ใน Python:
   ```python
   receive_packet(port, timeout=0.015)  # ควรเป็น 15ms
   ```

2. ตรวจสอบ CPU usage บน PC
   - ปิดโปรแกรมอื่นที่ไม่จำเป็น

3. ตรวจสอบ I2C clock speed (MCU):
   ```cpp
   Wire.setClock(400000);  // 400 kHz (Fast Mode)
   ```

---

### ปัญหา: Zero Calibration ไม่ทำงาน

**Symptoms**: กด 'z' แล้วค่ายังไม่เป็น 0

**Solutions**:
1. ตรวจสอบว่าเห็น message "[CMD] Sending SET_ZERO command..."
2. รอ acknowledgment packet (CALIBRATION)
3. ตรวจสอบว่า MCU ได้รับคำสั่ง:
   ```cpp
   // ใน main.cpp ควรมี code นี้
   if (packet_type == CMD_SET_ZERO) {
       // Set zero offset...
   }
   ```

---

## 📊 Data Flow Diagram

```
┌─────────────┐                              ┌─────────────┐
│   BNO055    │                              │  Computer   │
│  (Sensor)   │                              │   (Python)  │
└──────┬──────┘                              └──────┬──────┘
       │                                            │
       │ I2C @ 400kHz                               │
       │ Read Euler angles                          │
       │                                            │
       ▼                                            │
┌─────────────┐                                     │
│     MCU     │          UART @ 921600 baud         │
│  (STM32)    │────────────────────────────────────▶│
│             │     FB_IMU_DATA (100 Hz)           │
│             │                                     │
│             │◀────────────────────────────────────│
│             │     CMD_SET_ZERO (on demand)       │
└─────────────┘                                     │
       │                                            │
       │ Every 5 seconds                            │
       │────────────────────────────────────────────▶
       │     FB_IMU_CALIBRATION                     │
                                                    ▼
                                           ┌─────────────────┐
                                           │  Robot Control  │
                                           │     System      │
                                           └─────────────────┘
```

---

## 🚀 Advanced Features

### 1. Multiple IMU Units

สามารถใช้ IMU หลายตัวพร้อมกัน โดยเชื่อมต่อผ่าน UART แยก:

```python
# Open multiple ports
imu1 = serial.Serial('COM10', 921600)
imu2 = serial.Serial('COM11', 921600)

# Read from both
data1 = get_imu_data(imu1)
data2 = get_imu_data(imu2)
```

### 2. Data Logging

บันทึกข้อมูล IMU ลงไฟล์:

```python
import csv
import time

with open('imu_log.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['Timestamp', 'Roll', 'Pitch', 'Yaw', 'Sequence'])
    
    while True:
        result = receive_packet(port)
        if result and result[0] == 0x85:
            data = parse_imu_data(result[1])
            timestamp = time.time()
            writer.writerow([
                timestamp,
                data['roll'],
                data['pitch'],
                data['yaw'],
                data['sequence']
            ])
```

### 3. Real-time Visualization

แสดงผลแบบ real-time ด้วย matplotlib:

```python
import matplotlib.pyplot as plt
from collections import deque

# Setup plot
fig, ax = plt.subplots()
max_points = 100
roll_data = deque(maxlen=max_points)
pitch_data = deque(maxlen=max_points)
yaw_data = deque(maxlen=max_points)

while True:
    result = receive_packet(port)
    if result and result[0] == 0x85:
        data = parse_imu_data(result[1])
        
        roll_data.append(data['roll'])
        pitch_data.append(data['pitch'])
        yaw_data.append(data['yaw'])
        
        ax.clear()
        ax.plot(roll_data, label='Roll')
        ax.plot(pitch_data, label='Pitch')
        ax.plot(yaw_data, label='Yaw')
        ax.legend()
        plt.pause(0.01)
```

---

## 📝 Notes

### Coordinate System

BNO055 Orientation Convention:
```
Roll (Y-axis):  Rotation around Y-axis (left/right tilt)
Pitch (Z-axis): Rotation around Z-axis (forward/backward tilt)
Yaw (X-axis):   Rotation around X-axis (heading/compass)
```

### Zero Calibration Behavior

- Zero offset ใช้ได้จนกว่าจะ power cycle MCU
- Yaw มี wraparound handling (-180° ถึง +180°)
- Roll และ Pitch ไม่มี wraparound (linear)

### Calibration Tips

BNO055 ต้องการ calibration ก่อนใช้งาน:
1. **Gyroscope**: วาง IMU นิ่ง ๆ 2-3 วินาที
2. **Accelerometer**: เคลื่อนไหว IMU ช้า ๆ ในทุกแกน
3. **Magnetometer**: เคลื่อนไหวเป็นรูป 8 ในอากาศ

---

## 📚 References

- [BNO055 Datasheet](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)
- [Adafruit BNO055 Library](https://github.com/adafruit/Adafruit_BNO055)
- [CRC-CCITT Standard](https://en.wikipedia.org/wiki/Cyclic_redundancy_check)

---

**Document Version**: 1.0  
**Last Updated**: 2025-12-21  
**Author**: M-TRCH  
**Compatible with**: Motor Protocol v1.2
