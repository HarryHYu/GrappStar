// Host stub of the FreeRTOS calls kedge_dart.ino uses: mutex semaphores over std::timed_mutex, pinned tasks over
// std::thread, and the 1 ms tick (1 tick = 1 ms, so pdMS_TO_TICKS is the identity, as with CONFIG_FREERTOS_HZ=1000).
// A FreeRTOS mutex is not recursive, and neither is this one: a task that takes it twice deadlocks here too.
#pragma once
#include <stdint.h>

typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef uint32_t TickType_t;
typedef void (*TaskFunction_t)(void*);
typedef struct SimTask* TaskHandle_t;
typedef struct SimSemaphore* SemaphoreHandle_t;

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define pdFAIL 0
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFUL)
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portTICK_PERIOD_MS 1

SemaphoreHandle_t xSemaphoreCreateMutex();
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t ticks);
BaseType_t xSemaphoreGive(SemaphoreHandle_t s);

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char* name, uint32_t stackDepth, void* param,
                                   UBaseType_t priority, TaskHandle_t* handle, BaseType_t core);
TickType_t xTaskGetTickCount();
void vTaskDelay(TickType_t ticks);
void vTaskDelayUntil(TickType_t* previousWake, TickType_t increment);
