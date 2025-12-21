#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

// ============================================================
// Binary Protocol for IMU Unit (Compatible with Motor Protocol)
// ============================================================

// Protocol Constants
#define PROTOCOL_HEADER1        0xFE
#define PROTOCOL_HEADER2        0xEE
#define PROTOCOL_MAX_PAYLOAD    32

// Packet Types (IMU Specific)
#define FB_IMU_DATA             0x85    // IMU streaming data (Euler only)
#define FB_IMU_RAW              0x86    // Raw sensor data (Accel + Gyro + Mag)
#define FB_IMU_CALIBRATION      0x87    // Calibration status
#define CMD_IMU_CONFIG          0x05    // Configure IMU settings (future)
#define CMD_SET_ZERO            0x06    // Set current orientation as zero point

// IMU Unit ID (different from motor IDs)
#define IMU_UNIT_ID             0xFF    // Reserved ID for IMU unit

// Packet Structure
typedef struct {
    uint8_t header1;            // 0xFE
    uint8_t header2;            // 0xEE
    uint8_t packet_type;        // Packet type
    uint8_t payload_length;     // Length of payload
    uint8_t payload[PROTOCOL_MAX_PAYLOAD];
    uint16_t crc;               // CRC16 checksum
} __attribute__((packed)) BinaryPacket;

// IMU Data Payload (Packet Type 0x85)
// Total: 10 bytes (reduced from 16 - removed gyro data)
typedef struct {
    uint8_t unit_id;            // IMU Unit ID (0xFF)
    int16_t roll;               // Roll angle (degrees * 100)
    int16_t pitch;              // Pitch angle (degrees * 100)
    int16_t yaw;                // Yaw angle (degrees * 100)
    uint16_t sequence;          // Sequence number for packet tracking
    uint8_t status;             // Status flags
} __attribute__((packed)) IMUDataPayload;

// IMU Status Flags
#define IMU_STATUS_CALIBRATED       0x01    // Sensor is calibrated
#define IMU_STATUS_GYRO_CAL         0x02    // Gyroscope calibrated
#define IMU_STATUS_ACCEL_CAL        0x04    // Accelerometer calibrated
#define IMU_STATUS_MAG_CAL          0x08    // Magnetometer calibrated
#define IMU_STATUS_ERROR            0x80    // Sensor error

// Raw Sensor Data Payload (Packet Type 0x86)
// Total: 20 bytes
typedef struct {
    uint8_t unit_id;            // IMU Unit ID (0xFF)
    int16_t accel_x;            // Acceleration X (m/s² * 100)
    int16_t accel_y;            // Acceleration Y (m/s² * 100)
    int16_t accel_z;            // Acceleration Z (m/s² * 100)
    int16_t gyro_x;             // Angular velocity X (deg/s * 100)
    int16_t gyro_y;             // Angular velocity Y (deg/s * 100)
    int16_t gyro_z;             // Angular velocity Z (deg/s * 100)
    int16_t mag_x;              // Magnetic field X (µT * 10)
    int16_t mag_y;              // Magnetic field Y (µT * 10)
    int16_t mag_z;              // Magnetic field Z (µT * 10)
    uint16_t sequence;          // Sequence number
} __attribute__((packed)) IMURawPayload;

// Calibration Status Payload (Packet Type 0x87)
// Total: 5 bytes
typedef struct {
    uint8_t unit_id;            // IMU Unit ID (0xFF)
    uint8_t system_cal;         // System calibration (0-3)
    uint8_t gyro_cal;           // Gyroscope calibration (0-3)
    uint8_t accel_cal;          // Accelerometer calibration (0-3)
    uint8_t mag_cal;            // Magnetometer calibration (0-3)
} __attribute__((packed)) IMUCalibrationPayload;

// Set Zero Command Payload (Packet Type 0x06)
// Total: 0 bytes (no payload, command only)
typedef struct {
    // Empty - just triggers zero calibration
} __attribute__((packed)) SetZeroCommandPayload;

// Function Declarations
void protocolInit();
uint16_t calculateCRC16(const uint8_t* data, uint8_t length);
void sendIMUData(Stream& serial, const IMUDataPayload* data);
void sendIMURaw(Stream& serial, const IMURawPayload* data);
void sendIMUCalibration(Stream& serial, const IMUCalibrationPayload* data);
bool isBinaryPacketAvailable(Stream& serial);
bool receivePacket(Stream& serial, uint8_t* packet_type, uint8_t* payload, uint8_t* payload_length);

#endif // PROTOCOL_H
