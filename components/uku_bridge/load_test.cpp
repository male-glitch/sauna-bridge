// Supply load test: makes the ESP32 draw roughly its worst-case current so a power
// source (e.g. the UKU's 5 V knob supply) can be checked with a multimeter.
//   * a low-priority busy loop on core 0 (yields every ms so watchdogs stay happy)
//   * a steady stream of UDP packets to the gateway, keeping the Wi-Fi transmitter busy
// The bridge keeps relaying normally on core 1.
#include "uku_bridge.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include <cstring>

namespace esphome {
namespace uku_bridge {

static const char *const TAG_LT = "uku_bridge.load";

static void cpu_burn(void *arg) {
  auto *on = static_cast<std::atomic<bool> *>(arg);
  volatile uint32_t x = 1;
  for (;;) {
    if (!on->load()) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }
    int64_t t0 = esp_timer_get_time();
    while (esp_timer_get_time() - t0 < 900) x = x * 1664525u + 1013904223u;
    vTaskDelay(1);
  }
}

static void wifi_burn(void *arg) {
  auto *on = static_cast<std::atomic<bool> *>(arg);
  static uint8_t pkt[1000];
  for (size_t i = 0; i < sizeof(pkt); i++) pkt[i] = (uint8_t) (i * 31u);
  int sock = -1;
  sockaddr_in dst{};
  for (;;) {
    if (!on->load()) {
      if (sock >= 0) { close(sock); sock = -1; }
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }
    if (sock < 0) {
      esp_netif_ip_info_t ip{};
      esp_netif_t *nif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
      if (nif == nullptr || esp_netif_get_ip_info(nif, &ip) != ESP_OK || ip.gw.addr == 0) {
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
      sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
      dst.sin_family = AF_INET;
      dst.sin_port = htons(9);  // "discard" port on the router
      dst.sin_addr.s_addr = ip.gw.addr;
    }
    if (sock >= 0) sendto(sock, pkt, sizeof(pkt), 0, (sockaddr *) &dst, sizeof(dst));
    vTaskDelay(pdMS_TO_TICKS(4));  // ~250 packets/s
  }
}

void UkuBridge::set_load_test(bool on) {
  if (on && !load_tasks_started_) {
    xTaskCreatePinnedToCore(cpu_burn, "load_cpu", 2048, &load_test_, 1, nullptr, 0);
    xTaskCreatePinnedToCore(wifi_burn, "load_wifi", 4096, &load_test_, 2, nullptr, 0);
    load_tasks_started_ = true;
  }
  load_test_.store(on);
  ESP_LOGW(TAG_LT, "supply load test %s", on ? "ON" : "OFF");
}

}  // namespace uku_bridge
}  // namespace esphome
