#ifndef ROBOCUP_TELEMETRY_H
#define ROBOCUP_TELEMETRY_H

// Set to 0 to remove map telemetry from the firmware build.
#define MAP_TELEMETRY_ENABLED 1

void telemetry_init();

// Call periodically to transmit map data over serial.
void telemetry_map_task();

#endif // ROBOCUP_TELEMETRY_H
