//
// Created by Jasper Cusiel on 20/07/2026.
//

#include <imu.h>
#include <Adafruit_BNO055.h>

#define FUSION_RUNNING 5
#define SYSTEM_STARTED 0x0F
#define NO_ERROR 0

double heading_offset = 0; // initial heading [rad]

// imu interface object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

// Stores imu data, exposes data to other modules.
imu_data data;

void displaySensorStatus()
{
    // From Adafruit library documentation.
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

void displaySensorDetails()
{
    // From Adafruit library documentation.
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       ");
    Serial.println(sensor.name);
    Serial.print("Driver Ver:   ");
    Serial.println(sensor.version);
    Serial.print("Unique ID:    ");
    Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    ");
    Serial.print(sensor.max_value);
    Serial.println(" xxx");
    Serial.print("Min Value:    ");
    Serial.print(sensor.min_value);
    Serial.println(" xxx");
    Serial.print("Resolution:   ");
    Serial.print(sensor.resolution);
    Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

bool imu_init()
{
    // Start the imu and check that fusion is running and there are no issues.
    uint8_t sysStatus, selfTest, sysError;

    // Check sensor starts on I2C bus.
    if (!bno.begin())
    {
        Serial.println("IMU not started");
        return false;
    }
    // Set config
    bno.setMode(OPERATION_MODE_CONFIG);
    delay(50);

    bno.setMode(OPERATION_MODE_NDOF);
    delay(100);

    // Check sensor fusion started
    for (int i = 0; i < 20; i++)
    {
        // 2 seconds max
        bno.getSystemStatus(&sysStatus, &selfTest, &sysError);

        if (sysStatus == FUSION_RUNNING) break;

        delay(100);
    }
    displaySensorDetails();
    displaySensorStatus();
    Serial.printf("IMU   SYS: %d | SELF: 0x%02X | ERR: %d\n", sysStatus, selfTest, sysError);

    // Check no other errors occured during startup.
    if (sysError != NO_ERROR)
    {
        Serial.println("IMU system error");
        return false;
    }

    // Get first reading and set heading offset (if any, boots heading as zero)
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    heading_offset = (float)radians(euler.x());

    return true;
}

void imu_task()
{
    // Get raw data
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    double heading = heading_offset - radians(euler.x());
    // Wrap heading
    if (heading > PI)
        heading -= 2 * PI;

    if (heading < -PI)
        heading += 2 * PI;

    // Store heading
    data.heading = (float)heading;

    // linear acceleration (not currently used in EKF)
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    // get gyro data
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    // Store data
    data.gyro_z = radians(gyro.z()); // Default is degrees, radians used throughout this project
    data.accel_x = (float)accel.x();
    data.accel_y = (float)accel.y();
}

// Expose heading and gyro data for other modules.
float imu_get_heading()
{
    return data.heading;
}

float imu_get_gyro_z()
{
    return data.gyro_z;
}
