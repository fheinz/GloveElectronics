// Select ESP32C3 Dev Module as the board, and enable "USB CDC On Boot" in the "Tools" menu.

#include <algorithm>
#include <utility>

#include <Wire.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

constexpr int LED_PIN = 21;
constexpr int SYNC_SERVER_CLIENT_SELECT_PIN = 5;  // high = server, low = client

// PIN 20 is an output on the client, and an input on the server.
// If they are connected together (ideally through a 1kOhm resistor for safety),
// the server can use the signal being output by the client to measure how synchronized
// they are.
constexpr int SYNC_PIN = 20;
class GloveApp {
 public:
  static inline volatile uint32_t sync_time = 0;
  static inline volatile TickType_t next_motor_cycle_start = 0;

  static void IRAM_ATTR sync_isr() {
    sync_time = millis();
  }

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

//**********************************************************************************
// I2C device driver

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

I2CMux i2c_mux;

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
    Wire.requestFrom(DRV2605_I2C_ADDR, 1);
    buf[0] = Wire.read();
    Wire.endTransmission();
    return buf[0];
  }

  const uint8_t channel_;
};

//**********************************************************************************
// Motor control

// The chip expects a PWM signal where duty cycle 50 means stopped, and 255 is full speed in one phase, 0 full speed in the opposite phase.
// Though for signal reasons we can't actually use 0% and 100%.
// This should ideally be configurable at run time, we should figure out a UI (maybe a phone app?).
constexpr uint16_t ACTIVE_DUTY_CYCLE = 250;
constexpr uint16_t INACTIVE_DUTY_CYCLE = 128;
constexpr int TARGET_DRIVE_FREQUENCY = 250;  // 250 Hz according to Pfeifer et al. 2021

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

//**********************************************************************************
// Device Management (Power, Sequence, Motors)

constexpr float LOW_BATT_THRESHOLD = 3.2f;
const uint32_t TOTAL_RUN_TIME_S = 2 * 60 * 60;

constexpr int PULSE_DURATION = pdMS_TO_TICKS(100);
constexpr int PAUSE_DURATION = pdMS_TO_TICKS(66);
constexpr int ACTIVE_CYCLE_DURATION = 4 * (PULSE_DURATION + PAUSE_DURATION);

#define VIBRATION_PATTERNS(a, b, c, d) VIBRATION_PATTERNS3(a, b, c, d), VIBRATION_PATTERNS3(b, a, c, d), VIBRATION_PATTERNS3(c, a, b, d), VIBRATION_PATTERNS3(d, a, b, c)
#define VIBRATION_PATTERNS3(a, b, c, d) VIBRATION_PATTERNS2(a, b, c, d), VIBRATION_PATTERNS2(a, c, b, d), VIBRATION_PATTERNS2(a, d, b, c)
#define VIBRATION_PATTERNS2(a, b, c, d) VIBRATION_PATTERN(a, b, c, d), VIBRATION_PATTERN(a, b, d, c)
#define VIBRATION_PATTERN(a, b, c, d) ((uint8_t)(((a)&0x03) | (((b)&0x03) << 2) | (((c)&0x03) << 4) | (((d)&0x03) << 6)))

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

Glove glove;

//**********************************************************************************
// Bluetooth

constexpr char* SERVICE_UUID = "21014a2d-ebee-4fcb-b8d4-dcf34c19610a";
constexpr char* PATTERN_SYNC_CHARACTERISTIC_UUID = "44f21c6b-b08b-4695-970f-f21a15b538db";
constexpr int BLE_SCAN_TIME_S = 5;
constexpr uint16_t BLE_SCAN_INTERVAL = 0x10;    // 10ms (in 0.625ms units)
constexpr uint16_t BLE_SCAN_WINDOW = 0x10;      // 10ms (in 0.625ms units)
constexpr uint16_t BLE_ADV_MIN_PREF_INTERVAL = 0x06;  // 7.5ms (Apple recommended minimum)
constexpr uint16_t BLE_ADV_MAX_PREF_INTERVAL = 0x12;  // 22.5ms

typedef struct __attribute__((packed)) {
  TickType_t sender_timestamp;
  TickType_t next_cycle_start;
  int pattern;
} PatternSyncMessage;

// Thread resources stored in GloveApp

class GloveBLEServer : public BLEServerCallbacks {
 public:
  GloveBLEServer() : server_(nullptr), service_(nullptr), pattern_sync_characteristic_(nullptr),
                     advertising_(nullptr), device_connected_(false), last_device_connected_(false) {}

  void init() {
    BLEDevice::init("Vibrating Glove (Server)");
    server_ = BLEDevice::createServer();
    server_->setCallbacks(this);
    service_ = server_->createService(SERVICE_UUID);
    pattern_sync_characteristic_ = service_->createCharacteristic(
      PATTERN_SYNC_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    setPatternSyncMessage(GloveApp::next_motor_cycle_start, glove.getCurrentPattern());
    service_->start();

    advertising_ = BLEDevice::getAdvertising();
    advertising_->addServiceUUID(SERVICE_UUID);
    advertising_->setScanResponse(true);
    advertising_->setMinPreferred(BLE_ADV_MIN_PREF_INTERVAL);
    advertising_->setMaxPreferred(BLE_ADV_MAX_PREF_INTERVAL);
    advertising_->start();
  }

  void loop() {
    if (device_connected_ && !last_device_connected_) {
      advertising_->stop();
    }
    if (!device_connected_ && last_device_connected_) {
      advertising_->start();
    }
    if (device_connected_) {
      setPatternSyncMessage(GloveApp::next_motor_cycle_start, glove.getCurrentPattern());
      pattern_sync_characteristic_->notify();
    }
    last_device_connected_ = device_connected_;
  }

 private:
  void onConnect(BLEServer*) override {
    device_connected_ = true;
    Serial.println("Client connected");
  }
  
  void onDisconnect(BLEServer*) override {
    device_connected_ = false;
    Serial.println("Client disconnected");
  }

  void setPatternSyncMessage(TickType_t next_cycle, int pattern) {
    PatternSyncMessage message = { xTaskGetTickCount(), next_cycle, pattern };
    pattern_sync_characteristic_->setValue((uint8_t*)&message, sizeof(message));
  }

  BLEServer* server_;
  BLEService* service_;
  BLECharacteristic* pattern_sync_characteristic_;
  BLEAdvertising* advertising_;
  volatile bool device_connected_;
  volatile bool last_device_connected_;
};

GloveBLEServer ble_server;

class GloveBLEClient : public BLEClientCallbacks {
 public:
  GloveBLEClient() : scan_(nullptr), client_(nullptr), connected_to_server_(false) {}

  void init() {
    BLEDevice::init("Vibrating Glove (Client)");
    scan_ = BLEDevice::getScan();
    scan_->setActiveScan(true);
    scan_->setInterval(BLE_SCAN_INTERVAL);
    scan_->setWindow(BLE_SCAN_WINDOW);
    client_ = BLEDevice::createClient();
  }

  void loop() {
    if (!(connected_to_server_ || connectToServer())) return;
  }

 private:
  void onConnect(BLEClient* pclient) override {
    Serial.println("Connected to server");
  }

  void onDisconnect(BLEClient* pclient) override {
    connected_to_server_ = false;
    Serial.println("Disconnected from server");
  }

  static void notifyCallback(BLERemoteCharacteristic* rc, uint8_t* data, size_t len, bool is_notify) {
    static TickType_t last_cycle_start_received = 0;
    TickType_t client_ticks = xTaskGetTickCount();
    PatternSyncMessage message;

    if (len != sizeof(message)) {
      Serial.printf("Wrong data length: %u, expected %d\n", len, sizeof(message));
      return;
    }

    memcpy(&message, data, len);
    TickType_t server_ticks = message.sender_timestamp;
    TickType_t next_cycle_start = message.next_cycle_start;
    glove.setCurrentPattern(message.pattern);

    if (last_cycle_start_received != next_cycle_start) {
      int32_t tick_drift = (uint32_t)(client_ticks - server_ticks);
      tick_drift -= 15; 

      last_cycle_start_received = next_cycle_start;
      GloveApp::next_motor_cycle_start = next_cycle_start + tick_drift;
      Serial.printf(
        "%010u: (%10u, %10u, %d) New pattern starting at %010u(%d)\n",
        client_ticks, server_ticks, next_cycle_start, glove.getCurrentPattern(), GloveApp::next_motor_cycle_start, tick_drift, glove.getCurrentPattern());
        
      GloveApp::notifyMotorTask();
    }
  }

  using NotifyCb = void (*)(BLERemoteCharacteristic*, uint8_t*, size_t, bool);

  bool bindCharacteristic(BLERemoteService* remote_service, const char* char_uuid, NotifyCb cb = nullptr) {
    BLERemoteCharacteristic* rx_char = remote_service->getCharacteristic(char_uuid);
    if (rx_char == nullptr) {
      Serial.printf("Characteristic %s not found on server\n", char_uuid);
      return false;
    }
    
    if (cb != nullptr) {
      if (!rx_char->canNotify()) {
        Serial.printf("Characteristic %s does not support notify capabilities\n", char_uuid);
        return false;
      }
      rx_char->registerForNotify(cb);
    }
    return true;
  }

  bool connectToServer() {
    if (connected_to_server_) return true;

    BLEScanResults* found_devices = scan_->start(BLE_SCAN_TIME_S, false);
    Serial.print("Devices found: ");
    Serial.println(found_devices->getCount());
    for (int dev_id = 0; dev_id < found_devices->getCount(); ++dev_id) {
      auto remote_device = found_devices->getDevice(dev_id);
      BLEUUID service_uuid = remote_device.getServiceUUID();
      if (service_uuid.equals(BLEUUID(SERVICE_UUID))) {
        Serial.println("Found a device! Validating...");
        client_->setClientCallbacks(this);
        client_->connect(&remote_device);
        BLERemoteService* remote_service = client_->getService(SERVICE_UUID);
        if (remote_service == nullptr) {
          Serial.println("We don't have the right service?");
          client_->disconnect();
          continue;
        }

        if (!bindCharacteristic(remote_service, PATTERN_SYNC_CHARACTERISTIC_UUID, notifyCallback)) {
          client_->disconnect();
          continue;
        }

        connected_to_server_ = true;
        break;
      }
    }
    scan_->clearResults();
    return connected_to_server_;
  }

  BLEScan* scan_;
  BLEClient* client_;
  volatile bool connected_to_server_;
};

GloveBLEClient ble_client;

//**********************************************************************************
// Setup & Loop

const TickType_t ble_update_frequency = pdMS_TO_TICKS(100);

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

// Rounds of five 1-second cycles, of which the first three are active, and the last two are passive
constexpr TickType_t FULL_CYCLE_LENGTH = pdMS_TO_TICKS(1000);
constexpr unsigned int CYCLES_PER_ROUND = 5;
constexpr unsigned int ACTIVE_CYCLES_PER_ROUND = 3;

void GloveApp::motor_server_task(void* parameter) {
  pinMode(SYNC_PIN, INPUT);
  attachInterrupt(SYNC_PIN, GloveApp::sync_isr, RISING);
  int cycle_number = 0;

  TickType_t last_wake_time = xTaskGetTickCount();

  while (true) {
    // Select a pattern. 
    glove.setCurrentPattern((cycle_number++ % CYCLES_PER_ROUND < ACTIVE_CYCLES_PER_ROUND) ? random(Glove::getNumPatterns()) : -1);
    // Wait for the start of the new cycle. The BLE cycle runs on a higher frequency, so this wait should give it
    // plenty of oportunity to communicate the new pattern to the client in time for it to execute it in sync.
    GloveApp::next_motor_cycle_start = last_wake_time + FULL_CYCLE_LENGTH;
    vTaskDelayUntil(&last_wake_time, FULL_CYCLE_LENGTH);
    // Execute the pattern, the client ought to have gotten the memo by this time and, depending on the accuracy of
    // our sync, be executing it around now too.
    glove.runMotors(false);
  }
}

void GloveApp::motor_client_task(void* parameter) {
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);

  // Suspend this task indefinitely until the BLE client signals that the sync payload has arrived
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  TickType_t last_wake_time = xTaskGetTickCount();
  vTaskDelayUntil(&last_wake_time, GloveApp::next_motor_cycle_start - last_wake_time);
  while (true) {
    glove.runMotors(true);
    vTaskDelayUntil(&last_wake_time, FULL_CYCLE_LENGTH);
  }
}

constexpr uint32_t TASK_STACK_SIZE = 10000;

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

void setup() {
  Serial.begin();
  delay(500);
  pinMode(LED_PIN, OUTPUT);
  pinMode(DRV_EN_PIN, OUTPUT);
  pinMode(SYNC_SERVER_CLIENT_SELECT_PIN, INPUT_PULLUP);
  i2c_mux.init();

  glove.init();

  bool is_ble_server = digitalRead(SYNC_SERVER_CLIENT_SELECT_PIN) == HIGH;
  GloveApp::start(is_ble_server);
}

// Low-priority housekeeping
void loop() {
  if (!glove.isBatteryOK()) {
    Serial.printf("Low battery (%fv). Shutting down\n", glove.getBatteryVoltage());
    glove.shutdown();
  }

  if (millis() > (TOTAL_RUN_TIME_S * 1000)) {
    Serial.println("We are done! Shutting down now");
    glove.shutdown(false);
  }
}
