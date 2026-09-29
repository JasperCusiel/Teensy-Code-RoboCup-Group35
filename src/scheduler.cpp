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
#include <Arduino.h>

void scheduler_init(task_t* tasks, uint8_t count)
{
    uint32_t now = micros();
    for (uint8_t i = 0; i < count; i++)
    {
        tasks[i].next_run = now;
    }
}

void scheduler_run(task_t* tasks, uint8_t count)
{
    uint32_t now = micros();

    for (uint8_t i = 0; i < count; ++i)
    {
        if ((int32_t)(now - tasks[i].next_run) < 0)
        {
            continue;
        }

        const uint32_t started_us = micros();

        tasks[i].handler();

        const uint32_t finished_us = micros();
        const uint32_t duration_us = finished_us - started_us;

        // Temporary diagnostic. Print only if buffer space exists,
        // to reduce the chance that logging itself blocks control.
        if (duration_us > 5000 &&
            Serial.availableForWrite() >= 64)
        {
            Serial.printf(
                "SLOW task=%u duration=%lu us\n",
                (unsigned)i,
                (unsigned long)duration_us);
        }

        now = micros();

        uint32_t next_run =
            tasks[i].next_run + tasks[i].period_us;

        if ((int32_t)(now - next_run) >= 0)
        {
            next_run = now + tasks[i].period_us;
        }

        tasks[i].next_run = next_run;
    }
}
