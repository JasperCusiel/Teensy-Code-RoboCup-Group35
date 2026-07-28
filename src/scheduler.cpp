//
// Created by Jasper Cusiel on 28/07/2026.
//

/*
File : scheduler.c
Author : Jasper Cusiel(jcu71), Joe Elder(jel132)
Date : 28 Apr 2026
Descr : Functions to implement a scheduler that runs tasks at a specified frequency.
*/

#include "scheduler.h"
#include <core_pins.h>

void scheduler_init(task_t *tasks, uint8_t count) {
  uint32_t now = micros();
  for (uint8_t i = 0; i < count; i++) {
    tasks[i].next_run = now;
  }
}

void scheduler_run(task_t *tasks, uint8_t count) {
  uint32_t now = micros();

  for (uint8_t i = 0; i < count; i++) {
    if ((int32_t)(now - tasks[i].next_run) >= 0) {
      tasks[i].handler();

      // Maintain fixed frequency (no drift)
      tasks[i].next_run += tasks[i].period_us;
    }
  }
}

