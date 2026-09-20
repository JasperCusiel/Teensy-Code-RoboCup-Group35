//
// Created by Joe Elder on 20/09/2026
//

#ifndef ROBOCUP_WEIGHT_DROPOFF_H
#define ROBOCUP_WEIGHT_DROPOFF_H




typedef enum {
  DROPOFF_STATUS_IDLE,
  DROPOFF_STATUS_LOWER,
  DROPOFF_STATUS_UNLOAD,
} weight_dropoff_state_t;

void weight_dropoff_state_update();

#endif // ROBOCUP_WEIGHT_PICKUP_H
