#include "uku_bridge.h"
#include "esphome/core/hal.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include <cstring>
#include <cstdio>

namespace esphome {
namespace uku_bridge {

static const char *const TAG = "uku_bridge";
static const int BAUD = 921600;
static const int RX_BUF = 4096;
static const int TX_BUF = 8192;

// timing (microseconds)
static const int64_t STALE_PARTIAL_US = 150;    // bytes of one message arrive back to back
static const int64_t OWN_GLITCH_US = 150;       // our own driver-release blip arrives right after we send
static const int64_t PANEL_GAP_US = 50;         // let the panel release its driver before we talk
static const int64_t ECHO_TIMEOUT_US = 1000;  // fallback only; any reply after a command counts
static const int64_t ACK_TIMEOUT_US = 120000;   // a full-screen payload takes ~52 ms on the wire
static const int64_t CTRL_IDLE_US = 2000;       // controller pauses <1 ms inside a drawing sequence
static const int64_t STATUS_STALE_US = 15000;
static const int64_t PAYLOAD_STALL_US = 5000;   // payload never started
static const int64_t PAYLOAD_GAP_US = 5000;     // payload stopped half way (500 us misfired in the field)
static const size_t PAYLOAD_LEAD = 192;         // buffer this much before streaming a payload to the panel
                                                // (the task was seen idle up to 1.8 ms mid-payload)
static const int64_t CTRL_ACK_MAX_US = 8000;   // forward the knob's ack; if it takes longer, ack ourselves
static const uint16_t MAX_PAYLOAD = 12000;
// Diagnostic: high while the relay task is busy, low while it waits. Logic analyzer channel.
static const gpio_num_t DEBUG_PIN = GPIO_NUM_23;

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
  // A driver being disabled shows up as a short low pulse -> bytes like C0 E0 F0 F8 FC (D0/80).
  return b == 0xC0 || b == 0xE0 || b == 0xF0 || b == 0xF8 || b == 0xFC || b == 0xD0 || b == 0x80 || b == 0x00;
}

static uint32_t fnv1a(uint32_t h, const uint8_t *d, size_t n) {
  for (size_t i = 0; i < n; i++) {
    h ^= d[i];
    h *= 16777619u;
  }
  return h;
}

struct DigitEntry { char font; uint16_t len; char digit; };
static const DigitEntry DIGITS[] = {
    {'B', 258, '2'}, {'B', 300, '0'},
    {'S', 224, '0'}, {'S', 159, '1'}, {'S', 192, '2'}, {'S', 213, '3'}, {'S', 182, '5'}, {'S', 211, '6'}, {'S', 204, '9'},
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
  uart_driver_install(s.port, RX_BUF, TX_BUF, 64, &s.queue, 0);
  uart_param_config(s.port, &cfg);
  // DE+RE (tied together on the module) on the RTS pin; the UART hardware drives it.
  uart_set_pin(s.port, s.tx, s.rx, s.de, UART_PIN_NO_CHANGE);
  // While our own driver is on, the transceiver's RO output floats: hold RX high.
  gpio_set_pull_mode((gpio_num_t) s.rx, GPIO_PULLUP_ONLY);
  uart_set_mode(s.port, UART_MODE_RS485_HALF_DUPLEX);
  uart_set_rx_timeout(s.port, 1);           // event ~1 byte time after a message ends
  uart_set_rx_full_threshold(s.port, 16);   // and every 16 bytes inside long payloads
}

void UkuBridge::setup() {
  gpio_reset_pin(DEBUG_PIN);
  gpio_set_direction(DEBUG_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(DEBUG_PIN, 0);
  open_side(ctrl_);
  open_side(panel_);
  xTaskCreatePinnedToCore(UkuBridge::task_entry, "uku_relay", 6144, this, 20, &task_handle_, 1);
}

void UkuBridge::dump_config() {
  ESP_LOGCONFIG(TAG, "UKU bridge v2.4-diag (knob ack forwarded, task activity on GPIO23): ctrl RX=%d TX=%d DE=%d | panel RX=%d TX=%d DE=%d", ctrl_.rx,
                ctrl_.tx, ctrl_.de, panel_.rx, panel_.tx, panel_.de);
}

// ---------------------------------------------------------------- relay task
void UkuBridge::task_entry(void *arg) { static_cast<UkuBridge *>(arg)->task(); }

bool UkuBridge::work_pending() const {
  return q_count_ || pwait_ != PanelWait::NONE || status_pending_ || cstate_ == CtrlState::PAYLOAD ||
         ctrl_ack_pending_;
}

void UkuBridge::task() {
  QueueSetHandle_t set = xQueueCreateSet(128);
  xQueueAddToSet(ctrl_.queue, set);
  xQueueAddToSet(panel_.queue, set);
  static uint8_t tmp[256];
  for (;;) {
    TickType_t to = work_pending() ? 1 : pdMS_TO_TICKS(20);
    gpio_set_level(DEBUG_PIN, 0);
    QueueSetMemberHandle_t who = xQueueSelectFromSet(set, to);
    gpio_set_level(DEBUG_PIN, 1);
    if (who != nullptr) {
      Side &src = (who == ctrl_.queue) ? ctrl_ : panel_;
      uart_event_t ev;
      if (xQueueReceive(who, &ev, 0) == pdTRUE) {
        if (ev.type == UART_FIFO_OVF || ev.type == UART_BUFFER_FULL) {
          uart_flush_input(src.port);
          xQueueReset(who);
          st_overflow_.fetch_add(1);
          src.n = 0;
        } else if (ev.type == UART_DATA) {
          for (;;) {
            size_t avail = 0;
            uart_get_buffered_data_len(src.port, &avail);
            if (avail == 0) break;
            if (avail > sizeof(tmp)) avail = sizeof(tmp);
            int got = uart_read_bytes(src.port, tmp, avail, 0);
            if (got <= 0) break;
            int64_t now = esp_timer_get_time();
            if (&src == &ctrl_) feed_from_ctrl(tmp, got, now); else feed_from_panel(tmp, got, now);
          }
        }
      }
    }
    int64_t now = esp_timer_get_time();
    // controller stopped half way through a payload: close it so the panel queue moves on
    if (cstate_ == CtrlState::PAYLOAD && now - ctrl_.last_rx_us > PAYLOAD_STALL_US) abort_payload();
    if (ctrl_ack_pending_ && now >= ctrl_ack_deadline_) {
      st_ack_forced_.fetch_add(1);
      send_ctrl_ack();
    }
    pump_panel(now);
    pump_status(now);
  }
}

void UkuBridge::abort_payload() {
  uint32_t n = st_payload_abort_.fetch_add(1);
  if (!abort_pending_.load() && (n < 10 || n % 100 == 0)) {
    int o = snprintf(abort_info_, sizeof(abort_info_), "#%u cmd", (unsigned) n + 1);
    for (int k = 0; k < 11 && o < (int) sizeof(abort_info_) - 4; k++)
      o += snprintf(abort_info_ + o, sizeof(abort_info_) - o, " %02X", cmd_[k]);
    snprintf(abort_info_ + o, sizeof(abort_info_) - o, " | got %u of %u payload bytes",
             (unsigned) (payload_len_ - payload_left_), (unsigned) payload_len_);
    abort_pending_.store(true);
  }
  // complete the payload with zeros so the knob is not left waiting for bytes
  // and does not read our next command as payload
  if (!skip_payload_) {
    static const uint8_t Z[64] = {0};
    uint16_t left = payload_left_;
    while (left) {
      uint16_t k = left > sizeof(Z) ? sizeof(Z) : left;
      left -= k;
      if (!q_push(Z, k, 1, left == 0, 0, 0)) { panel_reset(); break; }
    }
  }
  skip_payload_ = false;
  cstate_ = CtrlState::HEADER;
  ctrl_.n = 0;
}

void UkuBridge::send_ctrl_ack() {
  static const uint8_t ACK[2] = {0xAA, 0xA2};
  ctrl_send(ACK, 2);
  ctrl_ack_pending_ = false;
}

void UkuBridge::ctrl_send(const uint8_t *d, size_t n) {
  uart_write_bytes(ctrl_.port, (const char *) d, n);
  ctrl_tx_us_ = esp_timer_get_time();
}

// ---------------------------------------------------------------- controller side
void UkuBridge::feed_from_ctrl(const uint8_t *data, size_t len, int64_t now) {
  if (cstate_ == CtrlState::HEADER && ctrl_.n && now - ctrl_.last_rx_us > STALE_PARTIAL_US) ctrl_.n = 0;
  // A payload streams without pauses. A pause in the middle means the controller gave up
  // on it: what follows is a new command, not more payload.
  if (cstate_ == CtrlState::PAYLOAD && payload_left_ < payload_len_ && now - ctrl_.last_rx_us > PAYLOAD_GAP_US)
    abort_payload();
  ctrl_.last_rx_us = now;
  size_t i = 0;
  while (i < len) {
    if (cstate_ == CtrlState::PAYLOAD) {
      if (payload_left_ == payload_len_ && is_glitch(data[i]) && now - ctrl_echo_us_ < OWN_GLITCH_US) {
        i++;  // driver-release blip (the controller's after its command, or ours after the echo)
        continue;
      }
      size_t take = len - i;
      if (take > payload_left_) take = payload_left_;
      bool last = (take == payload_left_);
      if (!skip_payload_ && !q_push(data + i, take, 1, last, 0, 0)) {
        panel_reset();          // its command was flushed too: drop the rest of this payload
        skip_payload_ = !last;
      }
      payload_hash_ = fnv1a(payload_hash_, data + i, take);
      payload_left_ -= take;
      i += take;
      if (payload_left_ == 0) {
        // Do NOT ack yet: the knob's own ack is forwarded when it arrives. Acking at once
        // let the controller outrun the knob (capture 16.11: backlog grew past 200 ms).
        ctrl_ack_pending_ = true;
        ctrl_ack_deadline_ = now + CTRL_ACK_MAX_US;
        on_ctrl_payload_done();
        skip_payload_ = false;
        cstate_ = CtrlState::HEADER;
        ctrl_.n = 0;
      }
      continue;
    }
    uint8_t b = data[i++];
    if (ctrl_.n == 11) {
      memmove(ctrl_.buf, ctrl_.buf + 1, 10);
      ctrl_.n = 10;
    }
    ctrl_.buf[ctrl_.n++] = b;
    if (ctrl_.n == 11 && (ctrl_.buf[9] | (ctrl_.buf[10] << 8)) == crc16(ctrl_.buf, 9)) {
      on_ctrl_command(ctrl_.buf, now);
      ctrl_.n = 0;
    }
  }
}

void UkuBridge::on_ctrl_command(const uint8_t *c, int64_t now) {
  ctrl_send(c + 9, 2);  // echo the CRC first: the controller expects it within microseconds
  ctrl_echo_us_ = ctrl_tx_us_;
  st_cmds_.fetch_add(1);
  last_cmd_us_ = now;
  if (!q_push(c, 11, 0, 0, c[9], c[10])) {
    panel_reset();
    q_push(c, 11, 0, 0, c[9], c[10]);
  }
  memcpy(cmd_, c, 11);
  uint16_t plen = c[5] | (c[6] << 8);
  bool allzero = true;
  for (int k = 0; k < 9; k++) if (c[k]) { allzero = false; break; }
  if (allzero || plen == 0 || plen > MAX_PAYLOAD) {
    if (now - last_draw_us_ > 2000000) display_active_.store(false);
    return;
  }
  payload_len_ = plen;
  payload_left_ = plen;
  payload_hash_ = 2166136261u;
  cstate_ = CtrlState::PAYLOAD;
  last_draw_us_ = now;
  display_active_.store(true);
}

void UkuBridge::on_ctrl_payload_done() {
  st_payloads_.fetch_add(1);
  apply_glyph(cmd_[0], cmd_[1], cmd_[2], cmd_[3], cmd_[4], payload_len_, payload_hash_);
}

void UkuBridge::pump_status(int64_t now) {
  if (!status_pending_) return;
  if (now - status_us_ > STATUS_STALE_US) {
    status_pending_ = false;
    st_status_drop_.fetch_add(1);
    return;
  }
  if (cstate_ == CtrlState::PAYLOAD || ctrl_ack_pending_) return;
  if (now - ctrl_.last_rx_us < CTRL_IDLE_US) return;  // controller still mid-sequence
  if (now - ctrl_tx_us_ < 300) return;
  ctrl_send(status_, 7);
  status_pending_ = false;
}

// ---------------------------------------------------------------- panel side
bool UkuBridge::q_push(const uint8_t *d, size_t n, uint8_t kind, uint8_t last, uint8_t clo, uint8_t chi) {
  if (q_count_ >= QN || ring_used_ + n > RING) return false;
  size_t off = (ring_head_ + ring_used_) % RING;
  for (size_t k = 0; k < n; k++) ring_[(off + k) % RING] = d[k];
  ring_used_ += n;
  PanelItem &it = items_[(q_head_ + q_count_) % QN];
  it.off = off; it.len = n; it.kind = kind; it.last = last; it.crc_lo = clo; it.crc_hi = chi;
  q_count_++;
  return true;
}

void UkuBridge::panel_reset() {
  // the panel side fell hopelessly behind: forget everything queued for it
  st_overflow_.fetch_add(1);
  q_head_ = q_count_ = 0;
  ring_head_ = ring_used_ = 0;
  pwait_ = PanelWait::NONE;
  payload_started_ = true;
  lead_wait_since_ = 0;
}

void UkuBridge::pump_panel(int64_t now) {
  for (;;) {
    if (pwait_ != PanelWait::NONE) {
      if (now < pwait_deadline_) return;
      if (pwait_ == PanelWait::ECHO) st_echo_to_.fetch_add(1); else st_ack_to_.fetch_add(1);
      pwait_ = PanelWait::NONE;
    }
    if (!q_count_) return;
    if (now < panel_quiet_after_) {
      if (panel_quiet_after_ - now > 200) return;
      while ((now = esp_timer_get_time()) < panel_quiet_after_) {
      }
    }
    PanelItem &it = items_[q_head_];
    if (it.kind == 1 && !payload_started_) {
      // give the stream a head start so the panel driver never drops out mid-payload
      size_t sum = 0;
      bool have_last = false;
      for (size_t k = 0; k < q_count_; k++) {
        const PanelItem &p = items_[(q_head_ + k) % QN];
        if (p.kind != 1) break;
        sum += p.len;
        if (p.last) { have_last = true; break; }
      }
      if (!have_last && sum < PAYLOAD_LEAD) {
        if (!lead_wait_since_) lead_wait_since_ = now;
        if (now - lead_wait_since_ < 3000) return;
      }
      payload_started_ = true;
      lead_wait_since_ = 0;
    }
    // write (ring may wrap)
    size_t first = it.len;
    if (it.off + first > RING) first = RING - it.off;
    if (first) uart_write_bytes(panel_.port, (const char *) ring_ + it.off, first);
    if (it.len > first) uart_write_bytes(panel_.port, (const char *) ring_, it.len - first);
    ring_head_ = (it.off + it.len) % RING;
    ring_used_ -= it.len;
    uint8_t kind = it.kind, last = it.last, lo = it.crc_lo, hi = it.crc_hi;
    q_head_ = (q_head_ + 1) % QN;
    q_count_--;
    if (kind == 0) {
      pwait_ = PanelWait::ECHO;
      exp_lo_ = lo; exp_hi_ = hi;
      pwait_deadline_ = now + ECHO_TIMEOUT_US;
      payload_started_ = false;
      return;
    }
    if (last) {
      pwait_ = PanelWait::ACK;
      pwait_deadline_ = now + ACK_TIMEOUT_US;
      return;
    }
  }
}

void UkuBridge::feed_from_panel(const uint8_t *data, size_t len, int64_t now) {
  if (panel_.n && now - panel_.last_rx_us > STALE_PARTIAL_US) panel_.n = 0;
  panel_.last_rx_us = now;
  panel_quiet_after_ = now + PANEL_GAP_US;
  if (pwait_ == PanelWait::ECHO && panel_.n == 0 && !(len >= 7 && data[0] == 0x01)) {
    // The knob answers ~8 us after our command; our driver is not always off by then,
    // so the start of its echo can be clipped. Whatever arrives first is the echo.
    if (len >= 2 && data[0] == exp_lo_ && data[1] == exp_hi_) st_echo_ok_.fetch_add(1);
    else st_echo_clipped_.fetch_add(1);
    pwait_ = PanelWait::NONE;
    return;
  }
  for (size_t i = 0; i < len; i++) {
    if (panel_.n == sizeof(panel_.buf)) {
      memmove(panel_.buf, panel_.buf + 1, panel_.n - 1);
      panel_.n--;
    }
    panel_.buf[panel_.n++] = data[i];
    while (panel_.n) {
      uint8_t *b = panel_.buf;
      size_t used = 0;
      if (pwait_ == PanelWait::ECHO && panel_.n >= 2 && b[1] == exp_hi_ && (b[0] == exp_lo_ || b[0] != 0x01)) {
        pwait_ = PanelWait::NONE;
        st_echo_ok_.fetch_add(1);
        used = 2;
      } else if (pwait_ == PanelWait::ACK && panel_.n >= 2 && b[1] == 0xA2) {
        pwait_ = PanelWait::NONE;
        st_ack_ok_.fetch_add(1);
        if (ctrl_ack_pending_) send_ctrl_ack();
        used = 2;
      } else if (b[0] == 0x01) {
        if (panel_.n < 7) break;
        if ((b[5] | (b[6] << 8)) == crc16(b, 5)) {
          on_panel_status(b, now);
          used = 7;
        } else {
          used = 1;
        }
      } else if (pwait_ != PanelWait::NONE && panel_.n < 2) {
        break;
      } else {
        used = 1;
      }
      if (used == 1 && !is_glitch(b[0])) {
        st_panel_junk_.fetch_add(1);
        uint8_t k = junk_n_.load();
        junk_[k % sizeof(junk_)] = b[0];
        junk_n_.store(k + 1);
      }
      memmove(panel_.buf, panel_.buf + used, panel_.n - used);
      panel_.n -= used;
    }
  }
}

void UkuBridge::on_panel_status(const uint8_t *in, int64_t now) {
  uint8_t f[7];
  memcpy(f, in, 5);
  knob_raw_.store((uint16_t) (in[1] << 8 | in[2]));
  uint8_t pf = press_frames_.load();
  if (pf && now - press_set_us_.load() > 1000000) { press_frames_.store(0); pf = 0; }  // stale, drop
  if (pf) { f[1] = 0x01; press_frames_.store(pf - 1); }
  if (pending_clicks_.load() != 0 && now - turn_set_us_.load() > 5000000) pending_clicks_.store(0);  // stale, drop
  int pc = pending_clicks_.load();
  if (pc > 0) { encoder_offset_.fetch_add(2); pending_clicks_.store(pc - 1); last_click_us_.store(now); }
  else if (pc < 0) { encoder_offset_.fetch_sub(2); pending_clicks_.store(pc + 1); last_click_us_.store(now); }
  f[2] = (uint8_t) (f[2] + encoder_offset_.load());
  uint16_t nc = crc16(f, 5);
  f[5] = nc & 0xFF; f[6] = nc >> 8;
  if (status_pending_) st_status_drop_.fetch_add(1);
  memcpy(status_, f, 7);
  status_pending_ = true;
  status_us_ = now;
  frames_.fetch_add(1);
  last_status_us_ = now;
  link_ok_.store(true);
}

// ---------------------------------------------------------------- screen decoding
// report each unknown glyph once, not every time the screen redraws it
bool UkuBridge::first_sighting(char font, uint8_t x, uint16_t len) {
  uint32_t key = ((uint32_t) font << 24) | ((uint32_t) x << 16) | len;
  for (size_t i = 0; i < seen_n_; i++)
    if (seen_[i] == key) return false;
  if (seen_n_ < sizeof(seen_) / sizeof(seen_[0])) seen_[seen_n_++] = key;
  return true;
}

void UkuBridge::apply_glyph(uint8_t type, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t len, uint32_t hash) {
  if (y == 0x3E && h == 0x22) {
    int slot = (x == 0x27) ? 1 : (x == 0x43) ? 2 : (x == 0x0B) ? 0 : -1;
    if (slot >= 0) {
      char d = (type == 0x02) ? ' ' : lookup_digit('B', len);
      if (d == '?' && !unknown_pending_.load() && first_sighting('B', x, len)) {
        snprintf(unknown_, sizeof(unknown_), "B x=%02X len=%u hash=%08X", x, len, (unsigned) hash);
        unknown_pending_.store(true);
      }
      big_[slot] = d;
      recompute_numbers();
    }
    return;
  }
  if (y == 0x06 && h == 0x16) {
    int slot = (x == 0x40) ? 0 : (x == 0x51) ? 1 : (x == 0x62) ? 2 : -1;
    if (slot >= 0) {
      char d = (type == 0x02) ? ' ' : lookup_digit('S', len);
      if (d == '?' && !unknown_pending_.load() && first_sighting('S', x, len)) {
        snprintf(unknown_, sizeof(unknown_), "S x=%02X len=%u hash=%08X", x, len, (unsigned) hash);
        unknown_pending_.store(true);
      }
      small_[slot] = d;
      recompute_numbers();
    }
    return;
  }
  // Full-screen repaints (0,0,128,160). Only the two captured cases change the heating
  // state: type 02 / 1196 bytes right after START, type 01 / 166 bytes right after STOP.
  // Anything else (e.g. the 4772-byte HUUM logo) leaves it unchanged.
  if (x == 0 && y == 0 && w == 0x80 && h == 0xA0) {
    if (type == 0x02 && len == 1196) heating_.store(true);
    else if (type == 0x01 && len == 166) heating_.store(false);
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
  char b[3] = {big_[0] == '?' ? ' ' : big_[0], big_[1], big_[2]};
  temp_.store(to_int(b, 3));
  target_.store(to_int(small_, 3));
}

// ---------------------------------------------------------------- main loop side
bool UkuBridge::press(uint8_t frames) {
  if (!link_ok_.load()) {
    ESP_LOGW(TAG, "press refused: knob link is down");
    return false;
  }
  press_set_us_.store(esp_timer_get_time());
  press_frames_.store(frames);
  return true;
}

bool UkuBridge::turn(int clicks) {
  if (!link_ok_.load()) {
    ESP_LOGW(TAG, "turn refused: knob link is down");
    return false;
  }
  if (clicks > 0) {
    int cur = target_.load();
    if (cur < 0) {
      ESP_LOGW(TAG, "raise refused: current target unknown (wake the display first)");
      return false;
    }
    if (pending_clicks_.load() != 0 || esp_timer_get_time() - last_click_us_.load() < 1500000) {
      ESP_LOGW(TAG, "raise refused: previous change still being applied, try again in a second");
      return false;
    }
    int room = MAX_REMOTE_TARGET - cur;
    if (room <= 0) {
      ESP_LOGW(TAG, "raise refused: remote limit is %d C", MAX_REMOTE_TARGET);
      return false;
    }
    if (clicks > room) clicks = room;
  }
  turn_set_us_.store(esp_timer_get_time());
  pending_clicks_.fetch_add(clicks);
  return true;
}

bool UkuBridge::set_target(int celsius) {
  if (celsius > MAX_REMOTE_TARGET) celsius = MAX_REMOTE_TARGET;
  int cur = target_.load();
  if (cur < 0) return false;
  if (celsius == cur) return true;
  return turn(celsius - cur);
}

std::string UkuBridge::get_unknown_glyph() {
  if (!unknown_pending_.load()) return "";
  std::string s(unknown_);
  unknown_pending_.store(false);
  return s;
}

std::string UkuBridge::get_stats() const {
  char s[200];
  snprintf(s, sizeof(s),
           "status=%u cmds=%u payloads=%u | knob echo ok=%u clipped=%u timeout=%u | knob ack ok=%u timeout=%u forced=%u | junk=%u "
           "status_drop=%u abort=%u resets=%u",
           (unsigned) frames_.load(), (unsigned) st_cmds_.load(), (unsigned) st_payloads_.load(),
           (unsigned) st_echo_ok_.load(), (unsigned) st_echo_clipped_.load(), (unsigned) st_echo_to_.load(), (unsigned) st_ack_ok_.load(),
           (unsigned) st_ack_to_.load(), (unsigned) st_ack_forced_.load(), (unsigned) st_panel_junk_.load(), (unsigned) st_status_drop_.load(),
           (unsigned) st_payload_abort_.load(), (unsigned) st_overflow_.load());
  return std::string(s);
}

void UkuBridge::loop() {
  if (link_ok_.load() && esp_timer_get_time() - last_status_us_ > 1000000) {
    link_ok_.store(false);
    ESP_LOGW(TAG, "no panel status frames for 1 s");
  }
  if (unknown_pending_.load()) {
    ESP_LOGI(TAG, "unknown glyph: %s  (tell me what the display shows)", unknown_);
    unknown_pending_.store(false);
  }
  if (abort_pending_.load()) {
    ESP_LOGW(TAG, "payload stalled: %s", abort_info_);
    abort_pending_.store(false);
  }
  uint16_t kr = knob_raw_.load();
  if (kr != knob_logged_) {
    knob_logged_ = kr;
    ESP_LOGI(TAG, "knob: button=%u encoder=%u", kr >> 8, kr & 0xFF);
  }
  uint32_t ms = millis();
  if (ms - last_stats_ms_ > 10000) {
    last_stats_ms_ = ms;
    ESP_LOGI(TAG, "%s", get_stats().c_str());
    uint8_t n = junk_n_.load();
    if (n) {
      char h[40] = {0};
      size_t cnt = n < sizeof(junk_) ? n : sizeof(junk_);
      for (size_t k = 0; k < cnt; k++) snprintf(h + k * 3, 4, "%02X ", junk_[k]);
      ESP_LOGI(TAG, "last unexpected knob bytes: %s", h);
    }
  }
}

}  // namespace uku_bridge
}  // namespace esphome
