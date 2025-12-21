#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

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
    
    Serial.begin(115200);
    Wire.begin();
    while (!Serial) delay(10);  // wait for serial port to open!

    Serial.println("Orientation Sensor Test");
    Serial.println("");

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    delay(1000);
}

void loop()
{
    // Get Euler angles for robot balance control
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    // Print Euler angles: Roll, Pitch, Yaw
    Serial.print("Roll: ");
    Serial.print(orientationData.orientation.y);
    Serial.print(" | Pitch: ");
    Serial.print(orientationData.orientation.z);
    Serial.print(" | Yaw: ");
    Serial.println(orientationData.orientation.x);

    delay(BNO055_SAMPLERATE_DELAY_MS);
}