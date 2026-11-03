// Host stub of the ESP-IDF task watchdog (IDF 5 API, Arduino core 3). It does not reset anything, but it watches:
// a subscribed task that goes longer than timeout_ms without esp_task_wdt_reset() is reported on stderr and counted
// (see "wdt_trips" in /sim/state), so a stalled safety task shows up in the simulator.
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
typedef struct SimTask* TaskHandle_t;

typedef struct {
  uint32_t timeout_ms;
  uint32_t idle_core_mask;
  bool trigger_panic;
} esp_task_wdt_config_t;

esp_err_t esp_task_wdt_init(const esp_task_wdt_config_t* config);
esp_err_t esp_task_wdt_reconfigure(const esp_task_wdt_config_t* config);
esp_err_t esp_task_wdt_add(TaskHandle_t task);
esp_err_t esp_task_wdt_reset();
