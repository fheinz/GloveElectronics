#pragma once
#include <Arduino.h>

constexpr int LED_PIN = 21;
constexpr int SYNC_SERVER_CLIENT_SELECT_PIN = 5;  // high = server, low = client
constexpr int SYNC_PIN = 20;

constexpr int DRV_EN_PIN = 10;
constexpr int SDA_PIN = 8;
constexpr int SCL_PIN = 2;
constexpr uint16_t I2C_MUX_ADDR = 0x70;
constexpr uint32_t I2C_CLOCK_FREQUENCY = 400000;
constexpr uint16_t DRV2605_I2C_ADDR = 0x5A;

constexpr uint8_t DRV2605_REG_MODE = 0x01;
constexpr uint8_t DRV2605_MODE_PWM = 0x03;
constexpr uint8_t DRV2605_REG_FEEDBACK = 0x1A;
constexpr uint8_t DRV2605_REG_CONTROL3 = 0x1D;
constexpr uint8_t DRV2605_REG_VBATT = 0x21;

constexpr uint8_t DRV2605_LRA_MODE = 0x80;
constexpr uint8_t DRV2605_LRA_OPEN_LOOP = 0x01;

constexpr uint16_t ACTIVE_DUTY_CYCLE = 250;
constexpr uint16_t INACTIVE_DUTY_CYCLE = 128;
constexpr int TARGET_DRIVE_FREQUENCY = 250;  // 250 Hz according to Pfeifer et al. 2021

constexpr float LOW_BATT_THRESHOLD = 3.2f;
const uint32_t TOTAL_RUN_TIME_S = 2 * 60 * 60;

constexpr int PULSE_DURATION = pdMS_TO_TICKS(100);
constexpr int PAUSE_DURATION = pdMS_TO_TICKS(66);
constexpr int ACTIVE_CYCLE_DURATION = 4 * (PULSE_DURATION + PAUSE_DURATION);

#define VIBRATION_PATTERNS(a, b, c, d) VIBRATION_PATTERNS3(a, b, c, d), VIBRATION_PATTERNS3(b, a, c, d), VIBRATION_PATTERNS3(c, a, b, d), VIBRATION_PATTERNS3(d, a, b, c)
#define VIBRATION_PATTERNS3(a, b, c, d) VIBRATION_PATTERNS2(a, b, c, d), VIBRATION_PATTERNS2(a, c, b, d), VIBRATION_PATTERNS2(a, d, b, c)
#define VIBRATION_PATTERNS2(a, b, c, d) VIBRATION_PATTERN(a, b, c, d), VIBRATION_PATTERN(a, b, d, c)
#define VIBRATION_PATTERN(a, b, c, d) ((uint8_t)(((a)&0x03) | (((b)&0x03) << 2) | (((c)&0x03) << 4) | (((d)&0x03) << 6)))

// Bluetooth
constexpr char* SERVICE_UUID = "21014a2d-ebee-4fcb-b8d4-dcf34c19610a";
constexpr char* PATTERN_SYNC_CHARACTERISTIC_UUID = "44f21c6b-b08b-4695-970f-f21a15b538db";
constexpr int BLE_SCAN_TIME_S = 5;
constexpr uint16_t BLE_SCAN_INTERVAL = 0x10;    // 10ms (in 0.625ms units)
constexpr uint16_t BLE_SCAN_WINDOW = 0x10;      // 10ms (in 0.625ms units)
constexpr uint16_t BLE_ADV_MIN_PREF_INTERVAL = 0x06;  // 7.5ms (Apple recommended minimum)
constexpr uint16_t BLE_ADV_MAX_PREF_INTERVAL = 0x12;  // 22.5ms

const TickType_t ble_update_frequency = pdMS_TO_TICKS(100);
constexpr TickType_t FULL_CYCLE_LENGTH = pdMS_TO_TICKS(1000);
constexpr unsigned int CYCLES_PER_ROUND = 5;
constexpr unsigned int ACTIVE_CYCLES_PER_ROUND = 3;
constexpr uint32_t TASK_STACK_SIZE = 10000;
