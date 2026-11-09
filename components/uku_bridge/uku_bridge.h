#pragma once
// HUUM UKU wall-panel bus bridge (man-in-the-middle on the RS485 link).
//
// Bus facts (measured on a 2018 UKU 3 EU Combi GSM, Rev C):
//   921600 baud 8N1 on RS485 (terminals 1=A, 2=B, 3=+5V, 4=GND)
//   CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), appended little-endian.
//   Panel -> controller every 20 ms:  01 <button> <encoder> 00 00 + CRC
//       button  : 01 while the knob is pressed
//       encoder : absolute 8-bit counter, 2 counts per detent, CW = +
//   Controller -> panel: 9-byte command + CRC, then <len> payload bytes
//       <type> x y w h lenLo lenHi 00 00
//       type 01 = draw glyph bitmap, type 02 = fill/clear rect (8-byte param block)
//       all-zero 9 bytes = nothing to do (panel echoes the CRC as ack)
//       panel acks a payload with "AA A2".
//
// This component relays both directions byte-exactly, except:
//   * panel status frames get button/encoder rewritten on request (CRC recomputed)
//   * the single "driver release" glitch byte after each frame is dropped
// and it watches the drawing commands to recover what the screen shows.

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

struct Side {
  uart_port_t port;
  int rx, tx, de;
  QueueHandle_t queue{nullptr};
  // parser scratch
  uint8_t buf[16];
  size_t n{0};
  // output staging for one chunk
  uint8_t out[512];
  size_t outn{0};
  bool pending_glitch{false};
  uint8_t pending_byte{0};
};

class UkuBridge : public Component {
 public:
  void set_pins(int crx, int ctx, int cde, int prx, int ptx, int pde);
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void dump_config() override;

  // ---- commands (safe to call from the main loop / lambdas) ----
  void press(uint8_t frames = 6) { press_frames_.store(frames); }
  // Highest target temperature that may be set REMOTELY. The physical knob is not limited.
  static constexpr int MAX_REMOTE_TARGET = 100;
  // Turn the virtual knob by N detents (+ = clockwise); applied one detent per frame.
  // Raising is refused if the current target is unknown, and clipped at MAX_REMOTE_TARGET.
  bool turn(int clicks);
  // Set target temperature in °C (clipped at MAX_REMOTE_TARGET). Needs the current target to be readable.
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

 protected:
  static void task_entry(void *arg);
  void task();
  void open_side(Side &s);
  void write_side(Side &dst, const uint8_t *data, size_t len);
  void flush_out(Side &src, Side &dst);
  void emit(Side &s, const uint8_t *data, size_t len);

  void feed_from_panel(const uint8_t *data, size_t len);
  void feed_from_ctrl(const uint8_t *data, size_t len);
  void on_ctrl_command(const uint8_t *cmd);
  void on_ctrl_payload_done();
  void apply_glyph(uint8_t type, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t len, uint32_t hash);
  void recompute_numbers();

  static uint16_t crc16(const uint8_t *d, size_t n);
  static bool is_glitch(uint8_t b);

  Side ctrl_{UART_NUM_1, -1, -1, -1};
  Side panel_{UART_NUM_2, -1, -1, -1};

  // ctrl->panel parser
  CtrlState cstate_{CtrlState::HEADER};
  uint8_t cmd_[9];
  uint16_t payload_left_{0};
  uint16_t payload_len_{0};
  uint32_t payload_hash_{0};
  bool drop_next_glitch_ctrl_{false};

  // commands
  std::atomic<uint8_t> press_frames_{0};
  std::atomic<int> encoder_offset_{0};
  std::atomic<int> pending_clicks_{0};

  // decoded screen state
  // big digits (current temp): x = 0x27 tens, 0x43 ones; small (target): x = 0x40,0x51,0x62
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

  // unknown glyph reporting (task writes, loop reads)
  char unknown_[64]{0};
  char screen_[48]{0};
  std::atomic<bool> unknown_pending_{false};

  TaskHandle_t task_handle_{nullptr};
};

}  // namespace uku_bridge
}  // namespace esphome
