#pragma once
// HUUM UKU wall-panel bus bridge (man-in-the-middle on the RS485 link).
//
// Bus facts (measured on a 2018 UKU 3 EU Combi GSM, Rev C):
//   921600 baud 8N1 on RS485 (terminals 1=A, 2=B, 3=+5V, 4=GND)
//   CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), appended little-endian.
//   Panel -> controller every 20 ms:  01 <button> <encoder> 00 00 + CRC
//       button  : 01 while the knob is pressed
//       encoder : absolute 8-bit counter, 2 counts per detent, CW = +
//   Controller -> panel (in reply to a status frame):
//       9-byte command + CRC: <type> x y w h lenLo lenHi 00 00
//       panel echoes the command CRC ~8 us after the command ends
//       then <len> payload bytes (type 01 = glyph bitmap, type 02 = fill rect)
//       panel acks a payload with "AA A2" ~0.3-0.7 ms after it ends
//       all-zero 9 bytes / len 0 = nothing to draw (echo only, no payload)
//
// v2: the panel's 8 us CRC echo is far faster than any store-and-forward relay can
// manage, so this component now TERMINATES the protocol on each side instead of
// relaying bytes blindly:
//   * towards the controller it echoes command CRCs and acks payloads itself, at once
//   * towards the panel it replays the same commands/payloads at the panel's pace,
//     waiting for the panel's own echo / ack, which are consumed here
//   * panel status frames are forwarded (button/encoder rewritten on request) when
//     the controller side is idle
// Both transceivers run in the ESP32's hardware RS485 half-duplex mode (DE+RE on the
// UART RTS pin), so the driver is released within microseconds of the last stop bit.

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include <atomic>
#include <string>
#include <cstdint>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

namespace esphome {
namespace uku_bridge {

enum class CtrlState : uint8_t { HEADER, PAYLOAD };
enum class PanelWait : uint8_t { NONE, ECHO, ACK };

struct Side {
  uart_port_t port;
  int rx, tx, de;
  QueueHandle_t queue{nullptr};
  uint8_t buf[16];
  size_t n{0};
  int64_t last_rx_us{0};
};

// one thing to send to the panel
struct PanelItem {
  uint16_t off;   // offset in ring
  uint16_t len;
  uint8_t kind;   // 0 = command, 1 = payload chunk
  uint8_t last;   // payload: last chunk of this payload
  uint8_t crc_lo, crc_hi;  // command: expected echo
};

class UkuBridge : public Component {
 public:
  void set_pins(int crx, int ctx, int cde, int prx, int ptx, int pde);
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;

  // ---- commands (safe to call from the main loop / lambdas) ----
  bool press(uint8_t frames = 6);
  static constexpr int MAX_REMOTE_TARGET = 100;
  bool turn(int clicks);
  bool set_target(int celsius);

  // ---- state (-1 / false = unknown) ----
  int get_temp() const { return temp_.load(); }
  int get_target() const { return target_.load(); }
  bool get_heating() const { return heating_.load(); }
  bool get_display_active() const { return display_active_.load(); }
  bool get_link_ok() const { return link_ok_.load(); }
  std::string get_unknown_glyph();
  uint32_t get_frames_relayed() const { return frames_.load(); }
  std::string get_last_screen() const { return std::string(screen_); }
  std::string get_stats() const;

 protected:
  static void task_entry(void *arg);
  void task();
  void open_side(Side &s);

  void feed_from_panel(const uint8_t *data, size_t len, int64_t now);
  void feed_from_ctrl(const uint8_t *data, size_t len, int64_t now);
  void on_ctrl_command(const uint8_t *cmd11, int64_t now);
  void on_ctrl_payload_done();
  void on_panel_status(const uint8_t *f7, int64_t now);
  void ctrl_send(const uint8_t *d, size_t n);

  // panel-bound queue
  bool q_push(const uint8_t *d, size_t n, uint8_t kind, uint8_t last, uint8_t clo, uint8_t chi);
  void pump_panel(int64_t now);
  void panel_reset();
  void abort_payload();
  void pump_status(int64_t now);
  bool work_pending() const;

  void apply_glyph(uint8_t type, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t len, uint32_t hash);
  void recompute_numbers();
  bool first_sighting(char font, uint8_t x, uint16_t len);

  static uint16_t crc16(const uint8_t *d, size_t n);
  static bool is_glitch(uint8_t b);

  Side ctrl_{UART_NUM_1, -1, -1, -1};
  Side panel_{UART_NUM_2, -1, -1, -1};

  // ctrl parser
  CtrlState cstate_{CtrlState::HEADER};
  uint8_t cmd_[11];
  uint16_t payload_left_{0};
  uint16_t payload_len_{0};
  uint32_t payload_hash_{0};
  int64_t ctrl_tx_us_{0};      // when we last queued bytes towards the controller
  int64_t last_cmd_us_{0};

  // panel-bound ring
  static constexpr size_t RING = 16384;
  static constexpr size_t QN = 512;
  uint8_t ring_[RING];
  size_t ring_head_{0}, ring_used_{0};
  PanelItem items_[QN];
  size_t q_head_{0}, q_count_{0};
  bool payload_started_{true};
  PanelWait pwait_{PanelWait::NONE};
  uint8_t exp_lo_{0}, exp_hi_{0};
  int64_t pwait_deadline_{0};
  int64_t panel_quiet_after_{0};
  int64_t lead_wait_since_{0};
  bool skip_payload_{false};
  char abort_info_[96]{0};
  std::atomic<bool> abort_pending_{false};

  // pending status frame for the controller
  uint8_t status_[7];
  bool status_pending_{false};
  int64_t status_us_{0};

  // commands
  std::atomic<uint8_t> press_frames_{0};
  std::atomic<int> encoder_offset_{0};
  std::atomic<int> pending_clicks_{0};

  // decoded screen state
  char big_[3]{'?', '?', '?'};
  char small_[3]{'?', '?', '?'};
  std::atomic<int> temp_{-1};
  std::atomic<int> target_{-1};
  std::atomic<bool> heating_{false};
  std::atomic<bool> display_active_{false};
  std::atomic<bool> link_ok_{false};
  std::atomic<uint32_t> frames_{0};
  int64_t last_status_us_{0};
  int64_t last_draw_us_{0};
  std::atomic<int64_t> last_click_us_{0};
  std::atomic<int64_t> press_set_us_{0};
  std::atomic<int64_t> turn_set_us_{0};

  // diagnostics
  std::atomic<uint32_t> st_cmds_{0}, st_payloads_{0}, st_echo_ok_{0}, st_echo_clipped_{0}, st_echo_to_{0}, st_ack_ok_{0},
      st_ack_to_{0}, st_panel_junk_{0}, st_status_drop_{0}, st_payload_abort_{0}, st_overflow_{0};
  uint8_t junk_[8]{0};
  std::atomic<uint8_t> junk_n_{0};

  char unknown_[64]{0};
  char screen_[48]{0};
  std::atomic<bool> unknown_pending_{false};
  uint32_t last_stats_ms_{0};
  uint32_t seen_[48]{};
  size_t seen_n_{0};
  std::atomic<uint16_t> knob_raw_{0};
  uint16_t knob_logged_{0xFFFF};

  TaskHandle_t task_handle_{nullptr};
};

}  // namespace uku_bridge
}  // namespace esphome
