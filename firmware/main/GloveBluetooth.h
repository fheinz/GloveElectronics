#pragma once
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "config.h"
#include "GloveHardware.h"
#include "GloveApp.h"

typedef struct __attribute__((packed)) {
  TickType_t sender_timestamp;
  TickType_t next_cycle_start;
  int pattern;
} PatternSyncMessage;

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

extern GloveBLEServer ble_server;
extern GloveBLEClient ble_client;
