#include "ToF-Sensors.h"
#include <Arduino.h>
#include <Wire.h>
#include <display.h>

#include <SparkFunSX1509.h>

extern "C" {
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
  }
#include "stdint.h"


// IO expander
const byte SX1509_ADDRESS = 0x3F;
SX1509 io; // Create an SX1509 object to be used throughout
const uint8_t xshutPins[NumOfTOFSensors] = {0, 1, 2, 3, 4, 5};

static lidar_scan scan;

/* ----- VL53L1X variables ----- */

uint16_t Dev_init = 0x29; /* I2C address of device 1 */
float LidarAngle[NumOfTOFSensors*NumOfZonesPerSensor];
uint16_t LidarDistance[NumOfTOFSensors*NumOfZonesPerSensor];
uint32_t TimeStamp[NumOfTOFSensors*NumOfZonesPerSensor];
uint16_t Devs[6] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35};
uint16_t Distance;
uint16_t SignalRate;
uint16_t SpadNb;
uint16_t AmbientRate;
uint16_t SignalPerSpad;
uint16_t AmbientPerSpad;
uint8_t RangeStatus;
uint16_t RangeCounter = 0;

uint16_t zone_center[] = {247, 239, 231, 223, 215, 207, 199, 191, 183,
                          175, 167, 159, 151, 247, 239, 231, 223, 215};
// Timing Budget Options:  15, 20, 33, 50, 100, 200, 500
uint16_t TimingBudget = 15;

uint16_t current_zone = 0;
char BigBuff[4000];
char VL53L1X_BUFFER[60]; /* Create a buffer to get data */

VL53L1X_ERROR error = 0;
uint32_t i = 0;
uint8_t Zone, Sensor, Timeout;

uint32_t TimeStart, TimeEnd, TotalTime, CurrentTime;
uint8_t Sensorcheck;

// #define Calibrate
#ifdef Calibrate
int16_t OffsetCal[NumOfTOFSensors * NumOfZonesPerSensor] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
#else
int16_t OffsetCal[NumOfTOFSensors * NumOfZonesPerSensor] = {
    -40, -44, -29, -24, -21, -18, -14, -15, -14, -13, -17, -22, -32, -28, -17,
    -21, -24, -11, -12, -12, -9,  -11, -26, -25, -36, -38, -36, -46, -26, -24,
    -20, -16, -17, -14, -11, -18, -17, -26, -37, -43, -31, -21, -25, -20, -8,
    -12, -12, -13, -20, -27, -26, -38, -51, -41, -36, -24, -22, -21, -19, -16,
    };

#endif

bool tof_init() {
  if (io.begin(SX1509_ADDRESS)) {
    Serial.println("SX1509 Port Expander Started");
  }

  for (uint8_t i = 0; i < NumOfTOFSensors; i++) {
    io.pinMode(xshutPins[i], OUTPUT);
    io.digitalWrite(xshutPins[i], LOW);
  }
  Serial.println("pins driven low");

  return ResetAndInitializeAllSensors();  // true if all sensors started correctly
}

void calculate_sector_indices() {
  for(int j=0;j<NumOfTOFSensors * NumOfZonesPerSensor;j++)
  {
    float angle = FOV_MIN + j*(FOV_MAX-FOV_MIN) /((NumOfTOFSensors * NumOfZonesPerSensor) -1);

    int sector = (angle - FOV_MIN)/SECTOR_WIDTH;

    if(sector < 0) {
      sector=0;
    }

    if(sector >= NUM_SECTORS) {
      sector=NUM_SECTORS-1;
    }
    scan.sector_index[j] = sector;
  }
}

void ResetAllSensors(void) {
  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < NumOfTOFSensors; i++) {
    io.pinMode(xshutPins[i], OUTPUT);
    io.digitalWrite(xshutPins[i], LOW);
  }
}

void TurnOnSensor(uint8_t SensorNum) {
  io.digitalWrite(xshutPins[SensorNum], HIGH);
  Serial.printf("Starting Sensor %d\n", SensorNum);
}
uint8_t dataRead;
uint8_t ReadRegister8(uint8_t deviceAddress, uint8_t registerAddress) {
  uint8_t data = 0;
  uint8_t error = VL53L1_RdByte(deviceAddress, registerAddress, &data);
  if (error != 0) {
    Serial.printf("ReadRegister8 failed. Error code: %d\n", error);
  }
  return data;
}

void WriteRegister8(uint8_t deviceAddress, uint8_t registerAddress,
                    uint8_t dataByte) {
  uint8_t error = VL53L1_WrByte(deviceAddress, registerAddress, dataByte);
  if (error != 0) {
    Serial.printf("WriteRegister8 failed. Error code: %d\n", error);
  }
}

bool ResetAndInitializeAllSensors(void) {
  Serial.println("reset and initalize");
  uint8_t i, Sensor, error = 0;
  uint8_t Bootstate = 0;
  int16_t Offset;
  ResetAllSensors();
  delay(10);
  for (i = 0; i < NumOfTOFSensors; i++) {
    TurnOnSensor(i);
    delay(5);
    error += VL53L1X_BootState(Dev_init, &Bootstate);
    while (Bootstate != 0x03)
    {
      delay(5);
      error += VL53L1X_BootState(Dev_init, &Bootstate);
    }
    VL53L1X_SensorInit(Dev_init); /* Initialize sensor  */
    error = VL53L1X_SetI2CAddress(
        Dev_init,
        (Devs[i] << 1)); /* Change i2c address Left is now 0x30 and Dev1 left
                            shift as it uses 7 bit addresses */
    dataRead = ReadRegister8(Devs[i], static_cast<uint8_t>(0x10f));
    dataRead = ReadRegister8(Devs[i], static_cast<uint8_t>(0x110));
  }
  Serial.print("All Chips booted\n");

  for (Sensor = 0; Sensor < NumOfTOFSensors; Sensor++) {
    VL53L1X_SetDistanceMode(Devs[Sensor], 1);
    VL53L1X_SetTimingBudgetInMs(Devs[Sensor], TimingBudget);
    VL53L1X_SetInterMeasurementInMs(Devs[Sensor], TimingBudget);
    VL53L1X_SetROI(Devs[Sensor], WidthOfSPADsPerZone, 6);
    WriteRegister8(Devs[Sensor], ROI_CONFIG__USER_ROI_CENTRE_SPAD,
                   zone_center[0] - 0);
    error = VL53L1X_GetOffset(Devs[Sensor], &Offset);
    VL53L1X_SetOffset(Devs[Sensor], Offset + 40);
  }
  for (Sensor = 0; Sensor < NumOfTOFSensors; Sensor++) {
    VL53L1X_StartRanging(Devs[Sensor]);
    delay(1);
  }
  if (error != 0) {
    Serial.print("Some Errors seen\n");
    return false;
  }

  return true;
}

float OldAngle;
double SystemAngle;
void PlotPolarData(uint8_t SensorNum, uint8_t CurrentZone, uint8_t NumOfZones,
                   uint16_t Distance) {
  double PartZoneAngle;

  float CorrectedDistance = 0;

  if (Distance > 60000) {
    Distance = 0;
  }
  PartZoneAngle = (StartingZoneAngle + ZoneFOVChangePerStep * CurrentZone) -
                  (HorizontalFOVofSensor / 2.0);
  SystemAngle = -80 + 20.0 * SensorNum + PartZoneAngle;
  CorrectedDistance = pow(pow(RadarCircleRadius, 2) + pow(Distance, 2) -
                              (2 * RadarCircleRadius * Distance *
                               cos((180 - PartZoneAngle) / (180) * Pi)),
                          0.5);
  if (CorrectedDistance < 55) {
    CorrectedDistance = 55;
  }
  LidarAngle[SensorNum * 13 + CurrentZone] = SystemAngle;
  LidarDistance[SensorNum * 13 + CurrentZone] = (uint16_t)CorrectedDistance;

}

void get_ToFCalibration() {
  for (uint8_t Sensor = 0; Sensor < NumOfTOFSensors; Sensor++) {
    for (uint8_t Zone = 0; Zone < NumOfZonesPerSensor; Zone++) {
      WriteRegister8(Devs[Sensor], ROI_CONFIG__USER_ROI_CENTRE_SPAD, zone_center[Zone + 1]);
      delay(5);
      int16_t offset = 0;
      VL53L1X_CalibrateOffset(Devs[Sensor], 300, &offset);

      Serial.print(offset);
      if (Zone < NumOfZonesPerSensor - 1) {
        Serial.print(",");
      }
    }
    Serial.println();  // newline after each sensor's 13 values
  }
}


void get_tof_reading() {
  error = 0;
  TimeStart = millis();
  Timeout = 0;
  for (Zone = 0; Zone < NumOfZonesPerSensor; Zone++) {
    for (Sensor = 0; Sensor < NumOfTOFSensors; Sensor++) {
      WriteRegister8(Devs[Sensor], ROI_CONFIG__USER_ROI_CENTRE_SPAD,
                     zone_center[Zone + 1] - 0);
    }
    i = i + 1;
    for (Sensor = 0; Sensor < NumOfTOFSensors; Sensor++) {
      error = VL53L1X_CheckForDataReady(Devs[Sensor], &Sensorcheck);
      while ((Sensorcheck == 0) && (Timeout == 0)) {
        delay(1);
        CurrentTime = millis();
        if (CurrentTime >
            (TimeStart + (NumOfZonesPerSensor + 1) * TimingBudget * 2)) {
          Timeout = 1;
          Sensor = NumOfTOFSensors;
          Zone = NumOfZonesPerSensor;
        } else {
          error += VL53L1X_CheckForDataReady(Devs[Sensor], &Sensorcheck);
        }
      }
      if (Timeout == 0) {
        WriteRegister8(Devs[Sensor], ROI_CONFIG__USER_ROI_CENTRE_SPAD,
                       zone_center[Zone + 1] - 0);
        TimeStamp[Sensor * 13 + Zone] = millis();
        VL53L1X_ClearInterrupt(Devs[Sensor]);

        error += VL53L1X_GetDistance(Devs[Sensor], &Distance);
        error += VL53L1X_GetRangeStatus(Devs[Sensor], &RangeStatus);
        if ((RangeStatus == 0) || (RangeStatus == 7)) {
          if (Distance > 60000) {
            Distance = 0;
            PlotPolarData(Sensor, Zone, 13, 0);
          } else {
            Distance = Distance + OffsetCal[Sensor * 13 + Zone];
            if (Distance > 60000) {
              Distance = 0;
            }
            PlotPolarData(Sensor, Zone, 13, Distance);
          }
        } else {
          PlotPolarData(Sensor, Zone, 13, 4000);
        }
      }
    }
  }
  if (Timeout == 1) {
    ResetAndInitializeAllSensors();
    Timeout = 0;
    Serial.print("Reset Performed\n");
  } else {
    delay(TimingBudget);
    TimeEnd = millis();
    TotalTime = (TimeEnd - TimeStart);
    // snprintf(BigBuff, sizeof(BigBuff), "Time: %ld\n", TotalTime);
    // Serial.print(BigBuff);

    #ifdef DEBUG
    // Distances
    Serial.print("D ");
    for (int i = 0; i < 78; i++)
      Serial.printf("%u ", LidarDistance[i]);
    Serial.print("\n");

    // Angles
    Serial.print("A ");
    for (int i = 0; i < 78; i++)
      Serial.printf("%.1f ", LidarAngle[i]);
    Serial.print("\n");
    #endif
    for (size_t n = 0; n < NumOfTOFSensors * NumOfZonesPerSensor; n++) {
      scan.ranges[n] = LidarDistance[n] / 1000.0f;
    }

  }
  if (error != 0) {
    Serial.print("Some Errors seen\n");
  }
}

lidar_scan* get_scan() {
  return &scan;
}
