#include "rover_thread.hpp"

void thread_create(void(task)(void*), const core_enum core, const task_priority priority,
                   const char* taskName, const size_t stackSize)
{
    xTaskCreatePinnedToCore(task, taskName, stackSize, NULL, priority, NULL, core);
}

TimerHandle_t timer_create(void(callback)(TimerHandle_t), const uint32_t interval, const bool start, void* timerData, const char* timerName)
{
    TimerHandle_t handle = xTimerCreate(timerName, pdMS_TO_TICKS(interval), pdTRUE, timerData, callback);
    if (handle && start)
        xTimerStart(handle, 0);
    return handle;
}
void timer_delete(TimerHandle_t timer)
{
    xTimerDelete(timer, 0);
}