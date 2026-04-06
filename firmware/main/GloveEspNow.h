#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

class GloveEspNow {
 public:
  static void init(bool is_server);
  static void addPeer(const uint8_t* mac);
  static void sendSync();
  static void wakeUp();
  static void sleep();
  static void loop();
};
