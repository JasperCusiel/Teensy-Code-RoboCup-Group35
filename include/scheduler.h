//
// Created by Jasper Cusiel on 28/07/2026.
//

#ifndef ROBOCUP_SCHEDULER_H
#define ROBOCUP_SCHEDULER_H

/*
File : scheduler.h
Author : Jasper Cusiel(jcu71), Joe Elder(jel132)
Date : 28 Apr 2026
Descr : Functions to implement a scheduler that runs tasks at a specified frequency.
*/

#ifndef SCHEDULER_H
#define SCHEDULER_H
#include "stdint.h"

typedef struct {
  void (*handler)(void);  // Function to execute task
  uint32_t period_us;   // Period to run task in micro seconds
  uint32_t next_run;      // Tick value at which to next run the task
} task_t;
// Represents a single task


void scheduler_init(task_t *tasks, uint8_t count);
// Initializes all tasks to run immediately

void scheduler_run(task_t *tasks, uint8_t count);
// Run handler function if task is due to be run

#endif


#endif // ROBOCUP_SCHEDULER_H
