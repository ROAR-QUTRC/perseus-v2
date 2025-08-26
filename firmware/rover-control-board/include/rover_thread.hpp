#pragma once
/*
Note: This file contains both the `thread` AND `timer` modules!
*/
// C lib
#include <stdint.h>

// core
#include <freertos/FreeRTOS.h>

#include "freertos/task.h"
#include "freertos/timers.h"

#define DELAY_MS(_x) (vTaskDelay(pdMS_TO_TICKS(_x)))

enum core_enum
{
    CORE_0 = 0,
    CORE_PRO = 0,
    CORE_1 = 1,
    CORE_APP = 1,
    CORE_ANY = tskNO_AFFINITY
};

enum task_priority
{
    PRIORITY_LOW = 0,
    PRIORITY_MED = 5,
    PRIORITY_HIGH = 10,
    PRIORITY_HIGHEST = 15
};

void threadCreate(void(task)(void*), const core_enum core = CORE_ANY, const task_priority priority = PRIORITY_MED,
                  const char* taskName = NULL, const size_t stackSize = CONFIG_TASK_STACK_SIZE);

TimerHandle_t timerCreate(void(callback)(TimerHandle_t), const uint32_t interval, const bool start = true, void* timerData = NULL, const char* timerName = NULL);
void timerDelete(TimerHandle_t timer);