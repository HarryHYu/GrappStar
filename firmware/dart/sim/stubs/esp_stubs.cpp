// Host implementation of the FreeRTOS, task watchdog and reset reason stubs.
#include <stdio.h>
#include <chrono>
#include <map>
#include <mutex>
#include <thread>
#include "freertos/FreeRTOS.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "sim_world.h"

// ---------------------------------------------------------------------------------------------------------------- FreeRTOS
struct SimSemaphore { std::timed_mutex m; };

SemaphoreHandle_t xSemaphoreCreateMutex() { return new SimSemaphore; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t ticks) {
  if (ticks == portMAX_DELAY) { s->m.lock(); return pdTRUE; }
  return s->m.try_lock_for(std::chrono::milliseconds(ticks)) ? pdTRUE : pdFALSE;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t s) { s->m.unlock(); return pdTRUE; }

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char*, uint32_t, void* param, UBaseType_t, TaskHandle_t* handle, BaseType_t) {
  std::thread(fn, param).detach();
  if (handle) *handle = nullptr;
  return pdPASS;
}
TickType_t xTaskGetTickCount() { return sim::nowMs(); }
void vTaskDelay(TickType_t ticks) { std::this_thread::sleep_for(std::chrono::milliseconds(ticks)); }
// sleep until the tick *previousWake + increment, then advance *previousWake by increment (no catch-up burst: like
// FreeRTOS, a wake time already in the past returns at once)
void vTaskDelayUntil(TickType_t* previousWake, TickType_t increment) {
  *previousWake += increment;
  int32_t ahead = (int32_t)(*previousWake - sim::nowMs());
  if (ahead > 0) {
    uint64_t targetUs = sim::nowUs() / 1000 * 1000 + (uint64_t)ahead * 1000;   // the start of that millisecond
    std::this_thread::sleep_for(std::chrono::microseconds(targetUs - sim::nowUs()));
  }
}

// ---------------------------------------------------------------------------------------------------------------- watchdog
namespace {
std::mutex wdtMu;
uint32_t wdtTimeoutMs = 5000;
std::map<std::thread::id, uint64_t> wdtLastUs;      // subscribed tasks and their last reset
bool wdtThreadRunning = false;

void wdtWatch() {
  for (;;) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::lock_guard<std::mutex> l(wdtMu);
    uint64_t now = sim::nowUs();
    for (auto& t : wdtLastUs) {
      if (now - t.second > (uint64_t)wdtTimeoutMs * 1000) {
        fprintf(stderr, "SIM: task watchdog would fire now (no reset for %u ms): the real dart would panic-reset\n", wdtTimeoutMs);
        sim::noteWatchdogTrip();
        t.second = now;
      }
    }
  }
}
}  // namespace

esp_err_t esp_task_wdt_init(const esp_task_wdt_config_t* c) { std::lock_guard<std::mutex> l(wdtMu); wdtTimeoutMs = c->timeout_ms; return ESP_OK; }
esp_err_t esp_task_wdt_reconfigure(const esp_task_wdt_config_t* c) { return esp_task_wdt_init(c); }
esp_err_t esp_task_wdt_add(TaskHandle_t task) {
  if (task) return ESP_FAIL;                         // only "the calling task" (nullptr) is supported
  std::lock_guard<std::mutex> l(wdtMu);
  wdtLastUs[std::this_thread::get_id()] = sim::nowUs();
  if (!wdtThreadRunning) { wdtThreadRunning = true; std::thread(wdtWatch).detach(); }
  return ESP_OK;
}
esp_err_t esp_task_wdt_reset() {
  std::lock_guard<std::mutex> l(wdtMu);
  auto it = wdtLastUs.find(std::this_thread::get_id());
  if (it == wdtLastUs.end()) return ESP_FAIL;
  it->second = sim::nowUs(); return ESP_OK;
}

// ---------------------------------------------------------------------------------------------------------------- reset reason
esp_reset_reason_t simResetReason = ESP_RST_POWERON;   // set by sim_main (--reset)
esp_reset_reason_t esp_reset_reason() { return simResetReason; }
