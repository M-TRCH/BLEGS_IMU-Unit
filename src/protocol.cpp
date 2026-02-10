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
// Contains: Roll, Pitch, Yaw, Sequence (Gyro removed for efficiency)
void sendIMUData(Stream& serial, const IMUDataPayload* data)
{
    uint8_t buffer[64];
    uint8_t idx = 0;
    
    // Headers
    buffer[idx++] = PROTOCOL_HEADER1;
    buffer[idx++] = PROTOCOL_HEADER2;
    buffer[idx++] = FB_IMU_DATA;
    buffer[idx++] = sizeof(IMUDataPayload);  // Payload length = 10
    
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

// Check if binary packet is available
bool isBinaryPacketAvailable(Stream& serial)
{
    // Drain invalid bytes to resync with packet header (limit to avoid blocking)
    uint8_t max_drain = 16;
    while (serial.available() >= 2 && max_drain > 0)
    {
        if (serial.peek() == PROTOCOL_HEADER1)
        {
            return true;
        }
        serial.read();  // Discard non-header byte
        max_drain--;
    }
    return false;
}

// Receive and parse binary packet
// Returns true if valid packet received
bool receivePacket(Stream& serial, uint8_t* packet_type, uint8_t* payload, uint8_t* payload_length)
{
    // Look for header
    if (serial.available() < 4) return false;  // Need at least header + type + length
    
    uint8_t header1 = serial.read();
    if (header1 != PROTOCOL_HEADER1) return false;
    
    uint8_t header2 = serial.read();
    if (header2 != PROTOCOL_HEADER2) return false;
    
    // Read packet type and length
    *packet_type = serial.read();
    *payload_length = serial.read();
    
    // Validate payload length to prevent buffer overflow
    if (*payload_length > PROTOCOL_MAX_PAYLOAD)
    {
        return false;
    }
    
    // Wait for payload + CRC with unified timeout
    uint8_t total_remaining = *payload_length + 2;  // payload + CRC(2 bytes)
    uint32_t start_time = millis();
    while (serial.available() < total_remaining)
    {
        if (millis() - start_time > 100)  // 100ms timeout
        {
            return false;
        }
    }
    
    // Read payload
    for (uint8_t i = 0; i < *payload_length; i++)
    {
        payload[i] = serial.read();
    }
    
    // Read CRC
    uint8_t crc_high = serial.read();
    uint8_t crc_low = serial.read();
    uint16_t received_crc = (crc_high << 8) | crc_low;
    
    // Verify CRC
    uint8_t crc_buffer[PROTOCOL_MAX_PAYLOAD + 2];
    crc_buffer[0] = *packet_type;
    crc_buffer[1] = *payload_length;
    for (uint8_t i = 0; i < *payload_length; i++)
    {
        crc_buffer[2 + i] = payload[i];
    }
    
    uint16_t calculated_crc = calculateCRC16(crc_buffer, 2 + *payload_length);
    
    return (received_crc == calculated_crc);
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
