#pragma once
#include <Arduino.h>
#include "config.h"

class GloveApp {
 public:
  static inline volatile uint32_t sync_time = 0;
  static inline volatile TickType_t next_motor_cycle_start = 0;
  static inline volatile int32_t current_tick_drift = 0;

  static void IRAM_ATTR sync_isr();

  static void notifyMotorTask() {
    if (motor_task_handle_ != nullptr) {
      xTaskNotifyGive(motor_task_handle_);
    }
  }

  static void start(bool is_server);

 private:
  static inline TaskHandle_t ble_task_handle_ = nullptr;
  static inline TaskHandle_t motor_task_handle_ = nullptr;

  static void ble_server_task(void* parameter);
  static void ble_client_task(void* parameter);
  static void motor_server_task(void* parameter);
  static void motor_client_task(void* parameter);
};
