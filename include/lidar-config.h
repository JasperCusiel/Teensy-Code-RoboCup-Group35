//
// Created by Jasper Cusiel on 25/07/2026.
//

#ifndef ROBOCUP_LIDAR_CONFIG_H
#define ROBOCUP_LIDAR_CONFIG_H

#include <math.h>

#define RadarCircleRadius (180.0 / 2.0)
#define Pi 3.1415
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD 0x007F
#define NumOfTOFSensors 6
#define TotalWidthOfSPADS 16
#define WidthOfSPADsPerZone 4
#define NumOfSPADsShiftPerZone 1
#define HorizontalFOVofSensor 20
#define SingleSPADFOV (HorizontalFOVofSensor / TotalWidthOfSPADS)
#define NumOfZonesPerSensor (((TotalWidthOfSPADS - WidthOfSPADsPerZone) / NumOfSPADsShiftPerZone) + 1)
#define StartingZoneAngle (WidthOfSPADsPerZone / 2 * SingleSPADFOV)
#define ZoneFOVChangePerStep (SingleSPADFOV * NumOfSPADsShiftPerZone)

#define NUM_SECTORS 36
#define TOTAL_FOV (NumOfTOFSensors * HorizontalFOVofSensor * (M_PI/180.0f))

#define FOV_MIN (-TOTAL_FOV/2.0f)
#define FOV_MAX ( TOTAL_FOV/2.0f)

#define SECTOR_WIDTH ((FOV_MAX-FOV_MIN)/NUM_SECTORS)


#endif // ROBOCUP_LIDAR_CONFIG_H
