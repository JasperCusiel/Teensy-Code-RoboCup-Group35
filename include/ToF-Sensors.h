#ifndef TOF_SENSORS_H
#define TOF_SENSORS_H

#include <stdint.h>
#include <stdbool.h>

bool tof_init(void);
void ResetAllSensors(void);
void TurnOnSensor(uint8_t SensorNum);
uint8_t ReadRegister8(uint8_t deviceAddress, uint8_t registerAddress);
void WriteRegister8(uint8_t deviceAddress, uint8_t registerAddress,
                    uint8_t dataByte);
bool ResetAndInitializeAllSensors(void);
void PlotPolarData(uint8_t SensorNum, uint8_t CurrentZone, uint8_t NumOfZones,
                   uint16_t Distance);
void get_ToFCalibration();
void get_tof_reading(void);


#endif


