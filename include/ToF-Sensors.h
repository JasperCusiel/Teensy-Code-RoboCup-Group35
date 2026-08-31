#ifndef TOF_SENSORS_H
#define TOF_SENSORS_H

#include <lidar-config.h>
#include <stdint.h>

struct lidar_scan {
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
void PlotPolarData(uint8_t SensorNum, uint8_t CurrentZone, uint8_t NumOfZones,
                   uint16_t Distance);
void get_ToFCalibration();
void get_tof_reading();
void calculate_sector_indices();
lidar_scan* get_scan();
void lidar_init_angles();


#endif
