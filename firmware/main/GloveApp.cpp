#include "GloveApp.h"
#include "GloveHardware.h"
#include "GloveBluetooth.h"

void GloveApp::ble_server_task(void* parameter) {
  TickType_t last_wake_time = xTaskGetTickCount();

  ble_server.init();
  while (true) {
    vTaskDelayUntil(&last_wake_time, ble_update_frequency);
    ble_server.loop();
  }
}

void GloveApp::ble_client_task(void* parameter) {
  TickType_t last_wake_time = xTaskGetTickCount();

  ble_client.init();
  while (true) {
    ble_client.loop();
    vTaskDelayUntil(&last_wake_time, ble_update_frequency / 2);
  }
}

void GloveApp::motor_server_task(void* parameter) {
  pinMode(SYNC_PIN, INPUT);
  attachInterrupt(SYNC_PIN, GloveApp::sync_isr, RISING);
  int cycle_number = 0;

  TickType_t last_wake_time = xTaskGetTickCount();

  while (true) {
    glove.setCurrentPattern((cycle_number++ % CYCLES_PER_ROUND < ACTIVE_CYCLES_PER_ROUND) ? random(Glove::getNumPatterns()) : -1);
    GloveApp::next_motor_cycle_start = last_wake_time + FULL_CYCLE_LENGTH;
    vTaskDelayUntil(&last_wake_time, FULL_CYCLE_LENGTH);
    glove.runMotors(false);
  }
}

void GloveApp::motor_client_task(void* parameter) {
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  TickType_t last_wake_time = xTaskGetTickCount();
  vTaskDelayUntil(&last_wake_time, GloveApp::next_motor_cycle_start - last_wake_time);
  while (true) {
    glove.runMotors(true);
    vTaskDelayUntil(&last_wake_time, FULL_CYCLE_LENGTH);
  }
}

void GloveApp::start(bool is_server) {
  Serial.printf("Starting %s tasks: ", is_server ? "server" : "client");
  if (is_server) {
    xTaskCreate(GloveApp::motor_server_task, "Motor task", TASK_STACK_SIZE, NULL, 10, &GloveApp::motor_task_handle_);
    xTaskCreate(GloveApp::ble_server_task, "BLE task", TASK_STACK_SIZE, NULL, 10, &GloveApp::ble_task_handle_);
  } else {
    xTaskCreate(GloveApp::motor_client_task, "Motor task", TASK_STACK_SIZE, NULL, 10, &GloveApp::motor_task_handle_);
    xTaskCreate(GloveApp::ble_client_task, "BLE task", TASK_STACK_SIZE, NULL, 10, &GloveApp::ble_task_handle_);
  }
  Serial.println(GloveApp::motor_task_handle_ && GloveApp::ble_task_handle_ ? "SUCCESS" : "FAILURE");
}
