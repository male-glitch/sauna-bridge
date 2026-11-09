#include "uku_bridge.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include <cstring>
#include <cstdio>

namespace esphome {
namespace uku_bridge {

static const char *const TAG = "uku_bridge";
static const int BAUD = 921600;
static const int RX_BUF = 2048;
static const int TX_BUF = 2048;
// After the last byte of a chunk has gone out, keep the driver enabled this long
// while waiting for a continuation chunk (bitmap payloads arrive in pieces).
static const int64_t DE_HOLD_US = 40;

// ---------------------------------------------------------------- helpers
uint16_t UkuBridge::crc16(const uint8_t *d, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= (uint16_t) d[i] << 8;
    for (int k = 0; k < 8; k++)
      c = (c & 0x8000) ? (uint16_t) ((c << 1) ^ 0x1021) : (uint16_t) (c << 1);
  }
  return c;  // transmitted low byte first
}

bool UkuBridge::is_glitch(uint8_t b) {
  // A driver being disabled shows up as a short low pulse: a start bit plus a few
  // low data bits, then high -> bytes like C0 E0 F0 F8 FC (and occasionally D0/80).
  return b == 0xC0 || b == 0xE0 || b == 0xF0 || b == 0xF8 || b == 0xFC || b == 0xD0 || b == 0x80;
}

static uint32_t fnv1a(uint32_t h, const uint8_t *d, size_t n) {
  for (size_t i = 0; i < n; i++) {
    h ^= d[i];
    h *= 16777619u;
  }
  return h;
}

// Digit lookup by (font, payload length). Lengths observed on the real display;
// the hash is logged for unknown ones so the table can be extended.
struct DigitEntry { char font; uint16_t len; char digit; };
static const DigitEntry DIGITS[] = {
    // big font 23x34 (current temperature)
    {'B', 258, '2'}, {'B', 300, '0'},
    // small font 17x22 (target temperature)
    {'S', 224, '0'}, {'S', 159, '1'}, {'S', 192, '2'}, {'S', 213, '3'}, {'S', 204, '9'},
};

static char lookup_digit(char font, uint16_t len) {
  for (const auto &e : DIGITS)
    if (e.font == font && e.len == len) return e.digit;
  return '?';
}

// ---------------------------------------------------------------- setup
void UkuBridge::set_pins(int crx, int ctx, int cde, int prx, int ptx, int pde) {
  ctrl_.rx = crx; ctrl_.tx = ctx; ctrl_.de = cde;
  panel_.rx = prx; panel_.tx = ptx; panel_.de = pde;
}

void UkuBridge::open_side(Side &s) {
  uart_config_t cfg = {};
  cfg.baud_rate = BAUD;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_APB;
  uart_param_config(s.port, &cfg);
  uart_set_pin(s.port, s.tx, s.rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(s.port, RX_BUF, TX_BUF, 32, &s.queue, 0);
  // Interrupt on a short idle gap (2 symbol times) so a whole frame arrives as one chunk,
  // or when 64 bytes have piled up during a long bitmap payload.
  uart_set_rx_timeout(s.port, 2);
  uart_set_rx_full_threshold(s.port, 64);
  gpio_reset_pin((gpio_num_t) s.de);
  gpio_set_direction((gpio_num_t) s.de, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t) s.de, 0);
}

void UkuBridge::setup() {
  open_side(ctrl_);
  open_side(panel_);
  xTaskCreatePinnedToCore(UkuBridge::task_entry, "uku_relay", 4096, this, 20, &task_handle_, 1);
}

void UkuBridge::dump_config() {
  ESP_LOGCONFIG(TAG, "UKU bridge: ctrl RX=%d TX=%d DE=%d | panel RX=%d TX=%d DE=%d", ctrl_.rx, ctrl_.tx, ctrl_.de,
                panel_.rx, panel_.tx, panel_.de);
}

// ---------------------------------------------------------------- relay task
void UkuBridge::task_entry(void *arg) { static_cast<UkuBridge *>(arg)->task(); }

void UkuBridge::task() {
  QueueSetHandle_t set = xQueueCreateSet(64);
  xQueueAddToSet(ctrl_.queue, set);
  xQueueAddToSet(panel_.queue, set);
  static uint8_t tmp[512];
  for (;;) {
    QueueSetMemberHandle_t who = xQueueSelectFromSet(set, pdMS_TO_TICKS(100));
    if (who == nullptr) continue;
    uart_event_t ev;
    Side &src = (who == ctrl_.queue) ? ctrl_ : panel_;
    Side &dst = (who == ctrl_.queue) ? panel_ : ctrl_;
    if (xQueueReceive(who, &ev, 0) != pdTRUE) continue;
    if (ev.type == UART_FIFO_OVF || ev.type == UART_BUFFER_FULL) {
      uart_flush_input(src.port);
      xQueueReset(who);
      continue;
    }
    if (ev.type != UART_DATA) continue;
    bool wrote = false;
    for (;;) {
      // Drain everything currently buffered on this side and relay it.
      for (;;) {
        size_t avail = 0;
        uart_get_buffered_data_len(src.port, &avail);
        if (avail == 0) break;
        if (avail > sizeof(tmp)) avail = sizeof(tmp);
        int got = uart_read_bytes(src.port, tmp, avail, 0);
        if (got <= 0) break;
        if (&src == &panel_) feed_from_panel(tmp, got); else feed_from_ctrl(tmp, got);
        if (src.outn) { flush_out(src, dst); wrote = true; }
      }
      // Hold the driver briefly in case a continuation chunk is right behind
      // (bitmap payloads arrive in 64-byte pieces with no idle gap between them).
      bool more = false;
      int64_t t0 = esp_timer_get_time();
      while (esp_timer_get_time() - t0 < DE_HOLD_US) {
        size_t avail = 0;
        uart_get_buffered_data_len(src.port, &avail);
        if (avail) { more = true; break; }
      }
      if (!more) break;
    }
    // chunk really finished: flush parser leftovers as-is, drop a trailing glitch byte
    if (src.n) { emit(src, src.buf, src.n); src.n = 0; }
    src.pending_glitch = false;
    if (src.outn) { flush_out(src, dst); wrote = true; }
    if (wrote) gpio_set_level((gpio_num_t) dst.de, 0);
  }
}

void UkuBridge::emit(Side &s, const uint8_t *data, size_t len) {
  if (s.outn + len > sizeof(s.out)) { len = sizeof(s.out) - s.outn; }
  memcpy(s.out + s.outn, data, len);
  s.outn += len;
}

void UkuBridge::flush_out(Side &src, Side &dst) {
  if (!src.outn) return;
  write_side(dst, src.out, src.outn);
  src.outn = 0;
}

void UkuBridge::write_side(Side &dst, const uint8_t *data, size_t len) {
  gpio_set_level((gpio_num_t) dst.de, 1);
  uart_write_bytes(dst.port, (const char *) data, len);
  uart_wait_tx_done(dst.port, pdMS_TO_TICKS(50));
  // DE is released by the caller after the hold-off window
}

// ---------------------------------------------------------------- panel -> controller
// Stream = 7-byte status frames (01 btn enc 00 00 crcL crcH), 2-byte acks, glitch bytes.
void UkuBridge::feed_from_panel(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[i];
    if (panel_.n == 0) {
      if (panel_.pending_glitch) {  // a glitch-looking byte was followed by more data: it was real
        emit(panel_, &panel_.pending_byte, 1);
        panel_.pending_glitch = false;
      }
      if (b != 0x01) {
        // ack / CRC echo / glitch: pass through, but defer glitch-looking bytes so a
        // trailing one (driver release) can be dropped at chunk end
        if (is_glitch(b)) { panel_.pending_glitch = true; panel_.pending_byte = b; }
        else emit(panel_, &b, 1);
        continue;
      }
    }
    panel_.buf[panel_.n++] = b;
    if (panel_.n < 7) continue;
    uint16_t c = crc16(panel_.buf, 5);
    if ((panel_.buf[5] | (panel_.buf[6] << 8)) == c) {
      // valid status frame: apply pending modifications
      uint8_t f[7];
      memcpy(f, panel_.buf, 5);
      uint8_t pf = press_frames_.load();
      if (pf) { f[1] = 0x01; press_frames_.store(pf - 1); }
      int pc = pending_clicks_.load();
      if (pc > 0) { encoder_offset_.fetch_add(2); pending_clicks_.store(pc - 1); }
      else if (pc < 0) { encoder_offset_.fetch_sub(2); pending_clicks_.store(pc + 1); }
      f[2] = (uint8_t) (f[2] + encoder_offset_.load());
      uint16_t nc = crc16(f, 5);
      f[5] = nc & 0xFF; f[6] = nc >> 8;
      emit(panel_, f, 7);
      frames_.fetch_add(1);
      last_status_us_ = esp_timer_get_time();
      link_ok_.store(true);
      panel_.n = 0;
    } else {
      // not a frame after all: release first byte, slide
      emit(panel_, panel_.buf, 1);
      memmove(panel_.buf, panel_.buf + 1, 6);
      panel_.n = 6;
    }
  }
}

// ---------------------------------------------------------------- controller -> panel
void UkuBridge::feed_from_ctrl(const uint8_t *data, size_t len) {
  size_t i = 0;
  while (i < len) {
    if (cstate_ == CtrlState::PAYLOAD) {
      if (drop_next_glitch_ctrl_) {
        // the controller releases its driver between the command and the payload
        drop_next_glitch_ctrl_ = false;
        if (is_glitch(data[i])) { i++; continue; }
      }
      size_t take = len - i;
      if (take > payload_left_) take = payload_left_;
      emit(ctrl_, data + i, take);
      payload_hash_ = fnv1a(payload_hash_, data + i, take);
      payload_left_ -= take;
      i += take;
      if (payload_left_ == 0) { on_ctrl_payload_done(); cstate_ = CtrlState::HEADER; drop_next_glitch_ctrl_ = true; }
      continue;
    }
    uint8_t b = data[i++];
    if (ctrl_.n == 0 && drop_next_glitch_ctrl_) {
      drop_next_glitch_ctrl_ = false;
      if (is_glitch(b)) continue;
    }
    ctrl_.buf[ctrl_.n++] = b;
    if (ctrl_.n < 11) continue;
    uint16_t c = crc16(ctrl_.buf, 9);
    if ((ctrl_.buf[9] | (ctrl_.buf[10] << 8)) == c) {
      emit(ctrl_, ctrl_.buf, 11);
      memcpy(cmd_, ctrl_.buf, 9);
      ctrl_.n = 0;
      on_ctrl_command(cmd_);
    } else {
      emit(ctrl_, ctrl_.buf, 1);
      memmove(ctrl_.buf, ctrl_.buf + 1, 10);
      ctrl_.n = 10;
    }
  }
}

void UkuBridge::on_ctrl_command(const uint8_t *cmd) {
  uint16_t plen = cmd[5] | (cmd[6] << 8);
  bool allzero = true;
  for (int k = 0; k < 9; k++) if (cmd[k]) { allzero = false; break; }
  if (allzero || plen == 0) {
    drop_next_glitch_ctrl_ = true;
    // nothing to draw this cycle; display considered off after 2 s without drawing
    if (esp_timer_get_time() - last_draw_us_ > 2000000) display_active_.store(false);
    return;
  }
  payload_len_ = plen;
  payload_left_ = plen;
  drop_next_glitch_ctrl_ = true;
  payload_hash_ = 2166136261u;
  cstate_ = CtrlState::PAYLOAD;
  last_draw_us_ = esp_timer_get_time();
  display_active_.store(true);
}

void UkuBridge::on_ctrl_payload_done() {
  apply_glyph(cmd_[0], cmd_[1], cmd_[2], cmd_[3], cmd_[4], payload_len_, payload_hash_);
}

void UkuBridge::apply_glyph(uint8_t type, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t len, uint32_t hash) {
  // --- big digits: current temperature ---
  if (y == 0x3E && h == 0x22) {
    int slot = (x == 0x27) ? 1 : (x == 0x43) ? 2 : (x == 0x0B) ? 0 : -1;
    if (slot >= 0) {
      char d = (type == 0x02) ? ' ' : lookup_digit('B', len);
      if (d == '?' && !unknown_pending_.load()) {
        snprintf(unknown_, sizeof(unknown_), "B x=%02X len=%u hash=%08X", x, len, (unsigned) hash);
        unknown_pending_.store(true);
      }
      big_[slot] = d;
      recompute_numbers();
    }
    return;
  }
  // --- small digits: target temperature ---
  if (y == 0x06 && h == 0x16) {
    int slot = (x == 0x40) ? 0 : (x == 0x51) ? 1 : (x == 0x62) ? 2 : -1;
    if (slot >= 0) {
      char d = (type == 0x02) ? ' ' : lookup_digit('S', len);
      if (d == '?' && !unknown_pending_.load()) {
        snprintf(unknown_, sizeof(unknown_), "S x=%02X len=%u hash=%08X", x, len, (unsigned) hash);
        unknown_pending_.store(true);
      }
      small_[slot] = d;
      recompute_numbers();
    }
    return;
  }
  // --- full-screen repaint (0,0,128,160): happens on start/stop.
  // Observed: type 02 + 1196-byte payload right after the START press,
  // type 01 + 166-byte payload right after the STOP press. Inferred, verify on first run.
  if (x == 0 && y == 0 && w == 0x80 && h == 0xA0) {
    heating_.store(type == 0x02);
    snprintf(screen_, sizeof(screen_), "type=%02X len=%u hash=%08X", type, len, (unsigned) hash);
    return;
  }
}

void UkuBridge::recompute_numbers() {
  auto to_int = [](const char *d, int n) -> int {
    int v = 0; bool any = false;
    for (int i = 0; i < n; i++) {
      if (d[i] == '?') return -1;
      if (d[i] == ' ') continue;
      v = v * 10 + (d[i] - '0'); any = true;
    }
    return any ? v : -1;
  };
  // big_[0] (hundreds) only exists above 99 °C; treat an unseen slot as blank
  char b[3] = {big_[0] == '?' ? ' ' : big_[0], big_[1], big_[2]};
  temp_.store(to_int(b, 3));
  target_.store(to_int(small_, 3));
}

// ---------------------------------------------------------------- main loop side
bool UkuBridge::set_target(int celsius) {
  int cur = target_.load();
  if (cur < 0) return false;
  turn(celsius - cur);
  return true;
}

std::string UkuBridge::get_unknown_glyph() {
  if (!unknown_pending_.load()) return "";
  std::string s(unknown_);
  unknown_pending_.store(false);
  return s;
}

void UkuBridge::loop() {
  // link watchdog: no status frame for 1 s -> link lost
  if (link_ok_.load() && esp_timer_get_time() - last_status_us_ > 1000000) {
    link_ok_.store(false);
    ESP_LOGW(TAG, "no panel status frames for 1 s");
  }
  if (unknown_pending_.load()) {
    ESP_LOGI(TAG, "unknown glyph: %s  (tell me what the display shows)", unknown_);
    unknown_pending_.store(false);
  }
}

}  // namespace uku_bridge
}  // namespace esphome
