//
// Created by Jasper Cusiel on 25/07/2026.
//

#ifndef ROBOCUP_LIDAR_CONFIG_H
#define ROBOCUP_LIDAR_CONFIG_H

#include <math.h>
#include <wiring.h>

#define RadarCircleRadius (180.0 / 2.0)
#define Pi 3.1415
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD 0x007F
#define NumOfTOFSensors 6
#define TotalWidthOfSPADS 16
#define WidthOfSPADsPerZone 8
#define NumOfZonesPerSensor 3
#define NumOfSPADsShiftPerZone ((TotalWidthOfSPADS - WidthOfSPADsPerZone) / (NumOfZonesPerSensor - 1))
#define RangingFrequencyHz 15
#define TOF_ROI_PERIOD_US ((1000000UL + RangingFrequencyHz - 1) / RangingFrequencyHz)
#define TOF_FULL_SCAN_PERIOD_US ((TOF_ROI_PERIOD_US * NumOfZonesPerSensor) + 50000UL)
#define HorizontalFOVofSensor 20.0f
#define SingleSPADFOV (HorizontalFOVofSensor / TotalWidthOfSPADS)
#define StartingZoneAngle ((WidthOfSPADsPerZone / 2.0f) * SingleSPADFOV)
#define ZoneFOVChangePerStep (SingleSPADFOV * NumOfSPADsShiftPerZone)

#define NUM_SECTORS 18
#define TOTAL_FOV radians(NumOfTOFSensors * HorizontalFOVofSensor)

// Scan bearings are measured from robot forward (+Y), CCW-positive.
#define FOV_MIN (-TOTAL_FOV/2.0f) // Radians
#define FOV_MAX ( TOTAL_FOV/2.0f) // Radians

#define SECTOR_WIDTH ((FOV_MAX-FOV_MIN)/NUM_SECTORS)

#define NUM_POINTS (NumOfTOFSensors * NumOfZonesPerSensor)

// Sensor offsets
#define TOF_ARRAY_OFFSET_X 0.0f // [m]
#define TOF_ARRAY_OFFSET_Y 0.1f // [m]


#define MAX_TOF_RANGE 4.0f // [m]]

#endif // ROBOCUP_LIDAR_CONFIG_H
