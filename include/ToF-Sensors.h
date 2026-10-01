#ifndef TOF_SENSORS_H
#define TOF_SENSORS_H

#include <lidar-config.h>
#include <stdint.h>

struct lidar_scan
{
    float ranges[NUM_POINTS];
    float angles[NUM_POINTS];
    uint8_t sector_index[NUM_POINTS];
};

bool tof_init();
void ResetAllSensors();
void TurnOnSensor(uint8_t SensorNum);
uint8_t ReadRegister8(uint8_t deviceAddress, uint8_t registerAddress);
void WriteRegister8(uint8_t deviceAddress, uint8_t registerAddress,
                    uint8_t dataByte);
bool ResetAndInitializeAllSensors();
void PlotPolarData(uint8_t sensor_num, uint8_t current_zone,
                   uint16_t r);
void get_ToFCalibration();
void get_tof_reading();
lidar_scan* get_scan();
// Incremented only when all sensors/zones have produced a complete scan.
uint32_t tof_get_scan_sequence();
uint32_t tof_get_scan_updated_ms();


// Right/left secondary sensor functions
bool tof_right_init();
uint16_t tof_right_read();
uint8_t tof_right_get_range_status();

bool tof_left_init();
uint16_t tof_left_read();
uint8_t tof_left_get_range_status();

#endif
