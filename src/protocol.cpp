#include "protocol.h"

// ============================================================
// Binary Protocol Implementation for IMU Unit
// ============================================================

// CRC16-CCITT (Polynomial: 0x1021, Initial: 0xFFFF)
// Same as motor protocol for compatibility
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

// Initialize protocol
void protocolInit()
{
    // Nothing specific to initialize for now
    // Can be used for future configuration
}

// Send IMU Data Packet (0x85)
// Contains: Roll, Pitch, Yaw, Gyro X/Y/Z, Sequence
void sendIMUData(Stream& serial, const IMUDataPayload* data)
{
    uint8_t buffer[64];
    uint8_t idx = 0;
    
    // Headers
    buffer[idx++] = PROTOCOL_HEADER1;
    buffer[idx++] = PROTOCOL_HEADER2;
    buffer[idx++] = FB_IMU_DATA;
    buffer[idx++] = sizeof(IMUDataPayload);  // Payload length = 16
    
    // Payload
    buffer[idx++] = data->unit_id;
    
    // Roll (int16)
    buffer[idx++] = (data->roll >> 8) & 0xFF;
    buffer[idx++] = data->roll & 0xFF;
    
    // Pitch (int16)
    buffer[idx++] = (data->pitch >> 8) & 0xFF;
    buffer[idx++] = data->pitch & 0xFF;
    
    // Yaw (int16)
    buffer[idx++] = (data->yaw >> 8) & 0xFF;
    buffer[idx++] = data->yaw & 0xFF;
    
    // Gyro X (int16)
    buffer[idx++] = (data->gyro_x >> 8) & 0xFF;
    buffer[idx++] = data->gyro_x & 0xFF;
    
    // Gyro Y (int16)
    buffer[idx++] = (data->gyro_y >> 8) & 0xFF;
    buffer[idx++] = data->gyro_y & 0xFF;
    
    // Gyro Z (int16)
    buffer[idx++] = (data->gyro_z >> 8) & 0xFF;
    buffer[idx++] = data->gyro_z & 0xFF;
    
    // Sequence (uint16)
    buffer[idx++] = (data->sequence >> 8) & 0xFF;
    buffer[idx++] = data->sequence & 0xFF;
    
    // Status (uint8)
    buffer[idx++] = data->status;
    
    // Calculate CRC over packet type, length, and payload
    uint16_t crc = calculateCRC16(&buffer[2], idx - 2);
    
    // Append CRC (big-endian)
    buffer[idx++] = (crc >> 8) & 0xFF;
    buffer[idx++] = crc & 0xFF;
    
    // Send packet
    serial.write(buffer, idx);
}

// Send Raw Sensor Data Packet (0x86)
// Contains: Accel X/Y/Z, Gyro X/Y/Z, Mag X/Y/Z, Sequence
void sendIMURaw(Stream& serial, const IMURawPayload* data)
{
    uint8_t buffer[64];
    uint8_t idx = 0;
    
    // Headers
    buffer[idx++] = PROTOCOL_HEADER1;
    buffer[idx++] = PROTOCOL_HEADER2;
    buffer[idx++] = FB_IMU_RAW;
    buffer[idx++] = sizeof(IMURawPayload);  // Payload length = 20
    
    // Payload
    buffer[idx++] = data->unit_id;
    
    // Accel X/Y/Z
    buffer[idx++] = (data->accel_x >> 8) & 0xFF;
    buffer[idx++] = data->accel_x & 0xFF;
    buffer[idx++] = (data->accel_y >> 8) & 0xFF;
    buffer[idx++] = data->accel_y & 0xFF;
    buffer[idx++] = (data->accel_z >> 8) & 0xFF;
    buffer[idx++] = data->accel_z & 0xFF;
    
    // Gyro X/Y/Z
    buffer[idx++] = (data->gyro_x >> 8) & 0xFF;
    buffer[idx++] = data->gyro_x & 0xFF;
    buffer[idx++] = (data->gyro_y >> 8) & 0xFF;
    buffer[idx++] = data->gyro_y & 0xFF;
    buffer[idx++] = (data->gyro_z >> 8) & 0xFF;
    buffer[idx++] = data->gyro_z & 0xFF;
    
    // Mag X/Y/Z
    buffer[idx++] = (data->mag_x >> 8) & 0xFF;
    buffer[idx++] = data->mag_x & 0xFF;
    buffer[idx++] = (data->mag_y >> 8) & 0xFF;
    buffer[idx++] = data->mag_y & 0xFF;
    buffer[idx++] = (data->mag_z >> 8) & 0xFF;
    buffer[idx++] = data->mag_z & 0xFF;
    
    // Sequence
    buffer[idx++] = (data->sequence >> 8) & 0xFF;
    buffer[idx++] = data->sequence & 0xFF;
    
    // Calculate CRC
    uint16_t crc = calculateCRC16(&buffer[2], idx - 2);
    
    // Append CRC
    buffer[idx++] = (crc >> 8) & 0xFF;
    buffer[idx++] = crc & 0xFF;
    
    // Send packet
    serial.write(buffer, idx);
}

// Send Calibration Status Packet (0x87)
void sendIMUCalibration(Stream& serial, const IMUCalibrationPayload* data)
{
    uint8_t buffer[32];
    uint8_t idx = 0;
    
    // Headers
    buffer[idx++] = PROTOCOL_HEADER1;
    buffer[idx++] = PROTOCOL_HEADER2;
    buffer[idx++] = FB_IMU_CALIBRATION;
    buffer[idx++] = sizeof(IMUCalibrationPayload);  // Payload length = 5
    
    // Payload
    buffer[idx++] = data->unit_id;
    buffer[idx++] = data->system_cal;
    buffer[idx++] = data->gyro_cal;
    buffer[idx++] = data->accel_cal;
    buffer[idx++] = data->mag_cal;
    
    // Calculate CRC
    uint16_t crc = calculateCRC16(&buffer[2], idx - 2);
    
    // Append CRC
    buffer[idx++] = (crc >> 8) & 0xFF;
    buffer[idx++] = crc & 0xFF;
    
    // Send packet
    serial.write(buffer, idx);
}
