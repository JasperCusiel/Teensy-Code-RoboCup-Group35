#include "ToF-Sensors.h"
#include <Arduino.h>
#include <Wire.h>
#include <display.h>
#include <SparkFunSX1509.h>

// ST api libraries are written in C
extern "C" {
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
}

#include "stdint.h"

// This module samples a 'scan' from the tof array and derived from ST's 2D lidar example.

// IO expander controls TOF XSHUT pins
const byte SX1509_ADDRESS = 0x3F;
SX1509 io;
const uint8_t x_shut_pins[NumOfTOFSensors] = {5, 4, 3, 2, 1, 0}; // Ordering of sensors

// Stores scan data
static lidar_scan scan;

// ----- VL53L1X variables -----
constexpr uint16_t Dev_init = 0x29; // Default I2C address is 0x29

float LidarAngle[NumOfTOFSensors * NumOfZonesPerSensor];

uint16_t LidarDistance[NumOfTOFSensors * NumOfZonesPerSensor];
constexpr uint16_t Devs[6] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35};
uint16_t Distance;
uint8_t RangeStatus;

constexpr uint16_t zone_center[NumOfZonesPerSensor] = {239, 207, 175};
// Long distance mode supports timing budgets: 20, 33, 50, 100, 200, 500.
constexpr uint16_t TimingBudget = 50;
constexpr uint32_t InterMeasurementMs = (1000 + RangingFrequencyHz - 1) / RangingFrequencyHz;
char VL53L1X_BUFFER[60]; // Create a buffer to get data

VL53L1X_ERROR error = 0;
uint32_t idx = 0;
uint8_t Zone, Sensor, Timeout;

uint32_t TimeStart, TimeEnd, TotalTime, CurrentTime;
uint8_t Sensorcheck;

// Calibration accounts for the fact that each ROI points at different part of sensing cone.
// #define Calibrate
#ifdef Calibrate
int16_t OffsetCal[NumOfTOFSensors * NumOfZonesPerSensor] = {
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
};
#else
int16_t OffsetCal[NumOfTOFSensors * NumOfZonesPerSensor] = {
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
};

#endif

namespace
{
    // Calculate sector indices used for VFH
    void calculate_sector_indices()
    {
        for (int j = 0; j < NumOfTOFSensors * NumOfZonesPerSensor; j++)
        {
            float angle = FOV_MIN + j * (FOV_MAX - FOV_MIN) / ((NumOfTOFSensors * NumOfZonesPerSensor) - 1);

            int sector = (angle - FOV_MIN) / SECTOR_WIDTH;

            if (sector < 0)
            {
                sector = 0;
            }

            if (sector >= NUM_SECTORS)
            {
                sector = NUM_SECTORS - 1;
            }
            scan.sector_index[j] = sector;
        }
    }

    // Calculate the angles for each scan
    void lidar_init_angles()
    {
        float angle_step = (FOV_MAX - FOV_MIN) / (NUM_POINTS - 1);
        for (int i = 0; i < NUM_POINTS; i++)
        {
            scan.angles[i] = FOV_MIN + i * angle_step;
        }
    }
}

bool tof_init()
{
    // Start the tof array.

    // Turn off all sensors.
    io.begin(SX1509_ADDRESS);
    for (uint8_t i = 0; i < NumOfTOFSensors; i++)
    {
        io.pinMode(x_shut_pins[i], OUTPUT);
        io.digitalWrite(x_shut_pins[i], LOW);
    }

    // Calculate angles and sectors for VFH.
    calculate_sector_indices();
    lidar_init_angles();

    return ResetAndInitializeAllSensors(); // True if all sensors started correctly
}


void ResetAllSensors()
{
    // Disable/reset all sensors by driving their XSHUT pins low.
    for (uint8_t i = 0; i < NumOfTOFSensors; i++)
    {
        io.pinMode(x_shut_pins[i], OUTPUT);
        io.digitalWrite(x_shut_pins[i], LOW);
    }
}

void TurnOnSensor(uint8_t SensorNum)
{
    // Start specific sensor.
    io.digitalWrite(x_shut_pins[SensorNum], HIGH);
    Serial.printf("Starting Sensor %d\n", SensorNum);
}


bool ResetAndInitializeAllSensors()
{
    uint8_t initialize_error = 0;
    uint8_t boot_state = 0;
    int16_t Offset;

    Serial.println("Reset and initialize tof sensors");
    ResetAllSensors();
    delay(10); // Let sensors boot

    for (size_t i = 0; i < NumOfTOFSensors; i++)
    {
        TurnOnSensor(i);
        delay(5);
        initialize_error += VL53L1X_BootState(Dev_init, &boot_state);
        while (boot_state != 0x03)
        {
            delay(5);
            initialize_error += VL53L1X_BootState(Dev_init, &boot_state);
        }
        VL53L1X_SensorInit(Dev_init); // Initialize sensor
        initialize_error = VL53L1X_SetI2CAddress(Dev_init, (Devs[i] << 1)); // shift as it uses 7 bit addresses
    }
    Serial.print("All Chips booted\n");
    // Apply config
    for (const unsigned short sensor : Devs)
    {
        VL53L1X_SetDistanceMode(sensor, 2);
        VL53L1X_SetTimingBudgetInMs(sensor, TimingBudget);
        VL53L1X_SetInterMeasurementInMs(sensor, InterMeasurementMs);
        VL53L1X_SetROI(sensor, WidthOfSPADsPerZone, 6);
        VL53L1_WrByte(sensor, ROI_CONFIG__USER_ROI_CENTRE_SPAD, zone_center[0]);
        initialize_error = VL53L1X_GetOffset(sensor, &Offset);
        VL53L1X_SetOffset(sensor, (Offset + 40));
    }

    // Start each sensor
    for (const unsigned short sensor : Devs)
    {
        VL53L1X_StartRanging(sensor);
        delay(1);
    }

    if (initialize_error != 0)
    {
        Serial.print("Errors seen starting ToF\n");
        return false;
    }

    return true;
}

float OldAngle;
double SystemAngle;

void PlotPolarData(uint8_t sensor_num, uint8_t current_zone, uint16_t r)
{
    double corrected_distance = 0;

    if (r > 60000)
    {
        r = 0;
    }

    double part_zone_angle = (StartingZoneAngle + ZoneFOVChangePerStep * current_zone) - (HorizontalFOVofSensor / 2.0);

    SystemAngle = -80 + 20.0 * sensor_num + part_zone_angle;
    corrected_distance = pow(
        pow(RadarCircleRadius, 2) + pow(r, 2) - (2 * RadarCircleRadius * r * cos(
            (180 - part_zone_angle) / (180) * Pi)), 0.5);

    // We can't be less than mounting radius
    if (corrected_distance < RadarCircleRadius)
    {
        corrected_distance = RadarCircleRadius;
    }

    LidarAngle[sensor_num * NumOfZonesPerSensor + current_zone] = static_cast<float>(SystemAngle);
    LidarDistance[sensor_num * NumOfZonesPerSensor + current_zone] = static_cast<uint16_t>(corrected_distance);
}

void get_ToFCalibration()
{
    for (uint8_t Sensor = 0; Sensor < NumOfTOFSensors; Sensor++)
    {
        for (uint8_t Zone = 0; Zone < NumOfZonesPerSensor; Zone++)
        {
            VL53L1_WrByte(Devs[Sensor], ROI_CONFIG__USER_ROI_CENTRE_SPAD, zone_center[Zone]);
            delay(5);
            int16_t offset = 0;
            VL53L1X_CalibrateOffset(Devs[Sensor], 300, &offset);

            Serial.print(offset);
            if (Zone < NumOfZonesPerSensor - 1)
            {
                Serial.print(",");
            }
        }
        Serial.println(); // newline after each sensor's ROI values
    }
}


void get_tof_reading()
{
    error = 0;
    TimeStart = millis();
    Timeout = 0;
    for (Zone = 0; Zone < NumOfZonesPerSensor; Zone++)
    {
        for (Sensor = 0; Sensor < NumOfTOFSensors; Sensor++)
        {
            VL53L1_WrByte(Devs[Sensor], ROI_CONFIG__USER_ROI_CENTRE_SPAD, zone_center[Zone]);
        }
        idx = idx + 1;
        for (Sensor = 0; Sensor < NumOfTOFSensors; Sensor++)
        {
            error = VL53L1X_CheckForDataReady(Devs[Sensor], &Sensorcheck);
            while ((Sensorcheck == 0) && (Timeout == 0))
            {
                CurrentTime = millis();
                if (CurrentTime > (TimeStart + (NumOfZonesPerSensor + 1) * TimingBudget * 2))
                {
                    Timeout = 1;
                    Sensor = NumOfTOFSensors;
                    Zone = NumOfZonesPerSensor;
                }
                else
                {
                    error += VL53L1X_CheckForDataReady(Devs[Sensor], &Sensorcheck);
                }
            }
            if (Timeout == 0)
            {
                VL53L1_WrByte(Devs[Sensor], ROI_CONFIG__USER_ROI_CENTRE_SPAD, zone_center[Zone]);
                VL53L1X_ClearInterrupt(Devs[Sensor]);

                error += VL53L1X_GetDistance(Devs[Sensor], &Distance);
                error += VL53L1X_GetRangeStatus(Devs[Sensor], &RangeStatus);
                if ((RangeStatus == 0) || (RangeStatus == 7))
                {
                    if (Distance > 60000)
                    {
                        Distance = 0;
                        PlotPolarData(Sensor, Zone, 0);
                    }
                    else
                    {
                        Distance = Distance + OffsetCal[Sensor * NumOfZonesPerSensor + Zone];
                        if (Distance > 60000)
                        {
                            Distance = 0;
                        }
                        PlotPolarData(Sensor, Zone, Distance);
                    }
                }
                else
                {
                    PlotPolarData(Sensor, Zone, 4000);
                }
            }
        }
    }
    if (Timeout == 1)
    {
        ResetAndInitializeAllSensors();
        Timeout = 0;
        Serial.print("Reset Performed\n");
    }
    else
    {
        for (size_t n = 0; n < NumOfTOFSensors * NumOfZonesPerSensor; n++)
        {
            scan.ranges[n] = LidarDistance[n] / 1000.0f;
        }
    }
    if (error != 0)
    {
        Serial.print("Some Errors seen\n");
    }
}

lidar_scan* get_scan()
{
    return &scan;
}
