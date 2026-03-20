#pragma once
#include <Wire.h>
#include "config.h"
#include "GloveApp.h"

class I2CMux {
 public:
  I2CMux() : current_channel_(-1) {}

  void init() {
    digitalWrite(DRV_EN_PIN, HIGH);
    Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK_FREQUENCY);
    // Wait for the driver to be ready (datasheet page 53).
    delayMicroseconds(250);
  }

  void selectChannel(uint8_t channel) {
    if (channel != current_channel_) {
      Wire.beginTransmission(I2C_MUX_ADDR);
      Wire.write(1 << channel);
      Wire.endTransmission();
      current_channel_ = channel;
    }
  }

 private:
  int current_channel_;
};

extern I2CMux i2c_mux;

class DRV2605 {
 public:
  constexpr DRV2605(uint8_t channel) : channel_(channel) {}

  void init() const {
    // This init sequence is adapted from the Adafruit DRV2605 library.
    // Puts the driver out of standby and into PWM input mode.
    setMode(DRV2605_MODE_PWM);
    setFeedback(getFeedback() | DRV2605_LRA_MODE);
    setControl3(getControl3() | DRV2605_LRA_OPEN_LOOP);
  }

  float getVBatt() const {
    i2c_mux.selectChannel(channel_);
    return static_cast<float>(_readRegister(DRV2605_REG_VBATT)) * 5.6f / 255.0f;
  }
  
  void setMode(uint8_t mode) const {
      i2c_mux.selectChannel(channel_);
      _writeRegister(DRV2605_REG_MODE, mode);
  }

  uint8_t getMode() const {
      i2c_mux.selectChannel(channel_);
      return _readRegister(DRV2605_REG_MODE);
  }
  
  void setFeedback(uint8_t value) const {
      i2c_mux.selectChannel(channel_);
      _writeRegister(DRV2605_REG_FEEDBACK, value);
  }

  uint8_t getFeedback() const {
      i2c_mux.selectChannel(channel_);
      return _readRegister(DRV2605_REG_FEEDBACK);
  }

  void setControl3(uint8_t value) const {
      i2c_mux.selectChannel(channel_);
      _writeRegister(DRV2605_REG_CONTROL3, value);
  }
  
  uint8_t getControl3() const {
      i2c_mux.selectChannel(channel_);
      return _readRegister(DRV2605_REG_CONTROL3);
  }

 private:
  void _writeRegister(uint8_t reg_addr, uint8_t val) const {
    uint8_t buf[2] = { reg_addr, val };
    Wire.beginTransmission(DRV2605_I2C_ADDR);
    Wire.write(buf, 2);
    Wire.endTransmission();
  }

  uint8_t _readRegister(uint8_t reg_addr) const {
    uint8_t buf[1] = { reg_addr };
    Wire.beginTransmission(DRV2605_I2C_ADDR);
    Wire.write(buf, 1);
    Wire.endTransmission(/*stop=*/false);
    Wire.requestFrom(DRV2605_I2C_ADDR, (uint8_t)1);
    buf[0] = Wire.read();
    Wire.endTransmission();
    return buf[0];
  }

  const uint8_t channel_;
};

class Motor {
 public:
  constexpr Motor(int pwm_pin, int channel) : pwm_pin_(pwm_pin), driver_(channel) {}

  void init() const {
    pinMode(pwm_pin_, OUTPUT);
    // PWM frequency is drive frequency * 128 (DRV2605 datasheet page 14).
    ledcAttach(pwm_pin_, TARGET_DRIVE_FREQUENCY * 128, 8);
    driver_.init();
  }

  void on() const { ledcWrite(pwm_pin_, ACTIVE_DUTY_CYCLE); }
  void off() const { ledcWrite(pwm_pin_, INACTIVE_DUTY_CYCLE); }
  
  const DRV2605& getDriver() const { return driver_; }

 private:
  const int pwm_pin_;
  DRV2605 driver_;
};

class Glove {
 public:
  Glove() : vbatt_(0.0f), current_pattern_(-1) {}

  void init() {
    for (auto& motor : motors_) motor.init();
  }

  void shutdown(bool with_flashing_leds = true) {
    digitalWrite(DRV_EN_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    if (with_flashing_leds) {
      for (int i = 0; i < 3; ++i) {
        digitalWrite(LED_PIN, HIGH);
        delay(300);
        digitalWrite(LED_PIN, LOW);
        delay(300);
      }
    } else {
      delay(100);
    }
    esp_deep_sleep_start();
  }

  bool isBatteryOK() {
    static int low_battery_iterations = 0;
    vbatt_ = motors_[0].getDriver().getVBatt();
    if (vbatt_ >= LOW_BATT_THRESHOLD) {
      low_battery_iterations = 0;
      return true;
    }
    return ++low_battery_iterations < 4;
  }

  float getBatteryVoltage() const { return vbatt_; }
  void setCurrentPattern(int pattern) { current_pattern_ = pattern; }
  int getCurrentPattern() const { return current_pattern_; }
  static constexpr unsigned int getNumPatterns() { return NUM_VIBRATION_PATTERNS; }

  void runMotors(bool is_client) {
    int pattern = current_pattern_;
    Serial.printf("%010u Running pattern #%02d\n", xTaskGetTickCount(), pattern);

    if (pattern < 0) {
      vTaskDelay(pdMS_TO_TICKS(ACTIVE_CYCLE_DURATION));
      return;
    }

    uint32_t start_time = millis();
    uint8_t seq = vibration_patterns_[pattern];
    
    if (is_client) {
      seq = ~seq;
      digitalWrite(SYNC_PIN, HIGH);
    }
    Serial.printf("%010u Sequence 0x%02x\n", xTaskGetTickCount(), seq);

    for (int ch = 0; ch < NUM_MOTORS; ++ch, seq >>= 2) {
      int finger = seq & 0x03;
      motors_[finger].on();
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(PULSE_DURATION);
      motors_[finger].off();
      digitalWrite(LED_PIN, LOW);
      
      if (ch == 0) {
        if (is_client) {
          digitalWrite(SYNC_PIN, LOW);
        } else if (GloveApp::sync_time != 0) {
          Serial.printf("%010u Skew %dms\n", xTaskGetTickCount(), start_time - GloveApp::sync_time);
        }
      }
      vTaskDelay(PAUSE_DURATION);
    }
    Serial.printf("%010u Done pattern #%02d\n", xTaskGetTickCount(), pattern);
  }

 private:
  static constexpr Motor motors_[] = { Motor(0, 0), Motor(1, 1), Motor(4, 2), Motor(3, 3) };
  static constexpr int NUM_MOTORS = sizeof(motors_) / sizeof(motors_[0]);
  static constexpr uint8_t vibration_patterns_[] = { VIBRATION_PATTERNS(0, 1, 2, 3) };
  static constexpr unsigned int NUM_VIBRATION_PATTERNS = sizeof(vibration_patterns_) / sizeof(vibration_patterns_[0]);
  
  float vbatt_;
  volatile int current_pattern_;
};

extern Glove glove;
