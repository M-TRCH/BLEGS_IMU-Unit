#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "protocol.h"

/* High-performance 100 Hz streaming configuration */
const uint16_t TARGET_SAMPLE_RATE_HZ = 100;
const uint32_t TARGET_SAMPLE_INTERVAL_US = 1000000 / TARGET_SAMPLE_RATE_HZ;  // 10000 microseconds

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Protocol variables
uint16_t packet_sequence = 0;
uint32_t last_sample_time_us = 0;
uint32_t last_calibration_send = 0;
const uint32_t CALIBRATION_SEND_INTERVAL = 5000;  // Send calibration status every 5 seconds

// Zero calibration offsets (for Set Zero command)
float roll_offset = 0.0f;
float pitch_offset = 0.0f;
float yaw_offset = 0.0f;
bool zero_calibrated = false;

// Cache calibration status (update every 100 samples to reduce I2C overhead)
uint8_t cached_system_cal = 0, cached_gyro_cal = 0, cached_accel_cal = 0, cached_mag_cal = 0;
uint16_t calibration_read_counter = 0;
const uint16_t CALIBRATION_READ_INTERVAL = 100;  // Update calibration every 100 samples (1 second)

void printEvent(sensors_event_t* event)
{
    double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
    if (event->type == SENSOR_TYPE_ACCELEROMETER)
    {
        Serial.print("Accl:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else if (event->type == SENSOR_TYPE_ORIENTATION)
    {
        Serial.print("Orient:");
        x = event->orientation.x;
        y = event->orientation.y;
        z = event->orientation.z;
    }
    else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD)
    {
        Serial.print("Mag:");
        x = event->magnetic.x;
        y = event->magnetic.y;
        z = event->magnetic.z;
    }
    else if (event->type == SENSOR_TYPE_GYROSCOPE)
    {
        Serial.print("Gyro:");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    }
    else if (event->type == SENSOR_TYPE_ROTATION_VECTOR)
    {
        Serial.print("Rot:");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    }
    else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION)
    {
        Serial.print("Linear:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else if (event->type == SENSOR_TYPE_GRAVITY)
    {
        Serial.print("Gravity:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else
    {
        Serial.print("Unk:");
    }

    Serial.print("\tx= ");
    Serial.print(x);
    Serial.print(" |\ty= ");
    Serial.print(y);
    Serial.print(" |\tz= ");
    Serial.println(z);
}

void setup()
{
    Serial.setRx(PA10);
    Serial.setTx(PA9);
    Wire.setSDA(PB9);
    Wire.setSCL(PB8);
    
    Serial.begin(921600);  // High-speed UART (same as motor protocol)
    Wire.begin();
    while (!Serial) delay(10);  // wait for serial port to open!

    Serial.println("IMU Unit - Binary Protocol Streaming");
    Serial.println("Compatible with Motor Protocol v1.2");
    Serial.println("");

    // Initialize protocol
    protocolInit();

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    
    delay(1000);
    
    Serial.println("BNO055 initialized successfully");
    Serial.println("Starting binary streaming at 100 Hz...");
    Serial.println("");
    
    // Read initial calibration status
    bno.getCalibration(&cached_system_cal, &cached_gyro_cal, &cached_accel_cal, &cached_mag_cal);
    
    delay(500);
    
    // Initialize timing
    last_sample_time_us = micros();
}

void loop()
{
    // Check for incoming commands from computer
    if (isBinaryPacketAvailable(Serial))
    {
        uint8_t packet_type;
        uint8_t payload[PROTOCOL_MAX_PAYLOAD];
        uint8_t payload_length;
        
        if (receivePacket(Serial, &packet_type, payload, &payload_length))
        {
            if (packet_type == CMD_SET_ZERO)
            {
                // Set current orientation as zero point
                sensors_event_t current_orientation;
                bno.getEvent(&current_orientation, Adafruit_BNO055::VECTOR_EULER);
                
                roll_offset = current_orientation.orientation.y;
                pitch_offset = current_orientation.orientation.z;
                yaw_offset = current_orientation.orientation.x;
                zero_calibrated = true;
                
                // Send acknowledgment (send current calibration status)
                IMUCalibrationPayload cal_ack;
                cal_ack.unit_id = IMU_UNIT_ID;
                cal_ack.system_cal = cached_system_cal;
                cal_ack.gyro_cal = cached_gyro_cal;
                cal_ack.accel_cal = cached_accel_cal;
                cal_ack.mag_cal = cached_mag_cal;
                sendIMUCalibration(Serial, &cal_ack);
            }
        }
    }
    
    // Precise 100 Hz timing control
    uint32_t current_time_us = micros();
    uint32_t elapsed_us = current_time_us - last_sample_time_us;
    
    // Wait for next sample interval
    if (elapsed_us < TARGET_SAMPLE_INTERVAL_US)
    {
        return;  // Not time yet for next sample
    }
    
    last_sample_time_us = current_time_us;
    
    // Get Euler angles only (optimized: single I2C transaction)
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    
    // Prepare IMU data packet
    IMUDataPayload imu_data;
    imu_data.unit_id = IMU_UNIT_ID;
    
    // Convert to int16 (degrees * 100 for precision)
    // Apply zero offset if calibrated
    // BNO055 Orientation: x=Yaw, y=Roll, z=Pitch
    float roll_raw = orientationData.orientation.y;
    float pitch_raw = orientationData.orientation.z;
    float yaw_raw = orientationData.orientation.x;
    
    if (zero_calibrated)
    {
        roll_raw -= roll_offset;
        pitch_raw -= pitch_offset;
        yaw_raw -= yaw_offset;
        
        // Handle yaw wraparound (0-360 degrees)
        if (yaw_raw > 180.0f) yaw_raw -= 360.0f;
        if (yaw_raw < -180.0f) yaw_raw += 360.0f;
    }
    
    imu_data.roll = (int16_t)(roll_raw * 100.0f);
    imu_data.pitch = (int16_t)(pitch_raw * 100.0f);
    imu_data.yaw = (int16_t)(yaw_raw * 100.0f);
    
    // Sequence number for packet tracking
    imu_data.sequence = packet_sequence++;
    
    // Update calibration cache periodically (reduce I2C overhead)
    calibration_read_counter++;
    if (calibration_read_counter >= CALIBRATION_READ_INTERVAL)
    {
        bno.getCalibration(&cached_system_cal, &cached_gyro_cal, &cached_accel_cal, &cached_mag_cal);
        calibration_read_counter = 0;
    }
    
    // Set status flags using cached calibration data
    imu_data.status = 0;
    if (cached_system_cal == 3) imu_data.status |= IMU_STATUS_CALIBRATED;
    if (cached_gyro_cal == 3) imu_data.status |= IMU_STATUS_GYRO_CAL;
    if (cached_accel_cal == 3) imu_data.status |= IMU_STATUS_ACCEL_CAL;
    if (cached_mag_cal == 3) imu_data.status |= IMU_STATUS_MAG_CAL;
    
    // Send binary packet (non-blocking)
    sendIMUData(Serial, &imu_data);
    
    // Periodically send calibration status (every 5 seconds)
    uint32_t current_time_ms = millis();
    if (current_time_ms - last_calibration_send >= CALIBRATION_SEND_INTERVAL)
    {
        IMUCalibrationPayload cal_data;
        cal_data.unit_id = IMU_UNIT_ID;
        cal_data.system_cal = cached_system_cal;
        cal_data.gyro_cal = cached_gyro_cal;
        cal_data.accel_cal = cached_accel_cal;
        cal_data.mag_cal = cached_mag_cal;
        
        sendIMUCalibration(Serial, &cal_data);
        last_calibration_send = current_time_ms;
    }
}