# Sauna bridge — HUUM UKU knob bus, man-in-the-middle on ESP32

Sits between the UKU main module (terminals 1–4) and the wall knob. Relays the
RS485 traffic byte-for-byte, can inject a knob press or a knob turn, and reads
the temperature / target / heating state off the drawing commands the
controller sends to the knob's screen. The knob keeps working exactly as before.

## Parts
- ESP32 DevKitC (WROOM-32U, antenna attached)
- 2× RS485 transceiver module, 3.3 V type (MAX3485 / SP3485), with DE and RE pins
- 5 V USB supply for the ESP32 (do not power the ESP32 from the knob's 5 V rail
  until it has been measured — Wi-Fi peaks are ~300 mA)
- small screw-terminal breakout or WAGO 221s for the four knob wires

## Wiring

Breaker OFF for all of this. The bus side is 5 V logic, nothing dangerous, but
the terminals sit next to 400 V in the UKU box.

```
UKU terminal 1 (A) ───── module A: A
UKU terminal 2 (B) ───── module A: B
UKU terminal 3 (+5V) ──┬─ knob wire 3
UKU terminal 4 (GND) ──┼─ knob wire 4 ──── ESP32 GND  (common ground is required)
                       │
knob wire 1 (A) ────────┼─ module B: A
knob wire 2 (B) ────────┘  module B: B

module A (controller side)        module B (knob side)
  VCC  -> ESP32 3V3                 VCC  -> ESP32 3V3
  GND  -> ESP32 GND                 GND  -> ESP32 GND
  RO   -> GPIO16                    RO   -> GPIO26
  DI   -> GPIO17                    DI   -> GPIO25
  DE+RE (tied together) -> GPIO4    DE+RE (tied together) -> GPIO27
```

If the modules have a 120 Ω termination jumper, either setting is fine at 3 m.
If you can only get MAX485 (5 V) modules: feed VCC from 5 V and put a
10 kΩ / 20 kΩ divider on each RO before the ESP32 pin.

The pins are arbitrary — change them in `sauna-bridge.yaml` if a different
layout suits the box better. Avoid GPIO 0, 2, 12, 15 (boot straps) and 34–39
(input-only) for TX/DE.

## Flashing
1. Home Assistant → ESPHome Builder → New device → paste `sauna-bridge.yaml`.
   The `external_components` line pulls the C++ component straight from this
   repository at compile time, so nothing else needs copying.
   Wi-Fi credentials go in ESPHome's `secrets.yaml` (Secrets button, top right).
2. First flash over USB (Install → Plug into this computer); later updates go OTA.

## First run — bring-up order
1. Power the ESP32 with the modules connected but the knob wires NOT yet attached.
   Check the log for `uku_bridge` dump_config.
2. Breaker on. Connect module A to UKU terminals 1–2 (module B still dangling).
   The knob is disconnected, so the UKU will think no panel is attached. Nothing
   should happen; log stays quiet.
3. Breaker off. Connect knob wires 1–2 to module B, 3–4 to the 5 V / GND rail.
   Breaker on. Within a second "Sauna panel link" should go on and "Sauna frames
   relayed" should start counting ~50/s. The knob should wake and behave normally
   — turn it, press it: if the display responds as before, relay timing is fine.
4. Touch the knob so the display is awake. "Sauna temperature" and "Sauna target
   temperature" should fill in within 5 s. If the log says `unknown glyph: S x=.. len=..`,
   note what the display shows and send me the line — one line per new digit, ten
   per font, then it's complete.
5. Press "Sauna knob press" from HA: the knob should react exactly as to a finger.
   Watch "Sauna heating" — it flips on the full-screen repaint. If it reads
   inverted, the type-01/type-02 guess was wrong; one-line fix.
6. "Sauna target +1 / -1": the target digits on the knob should change by one per
   press. Then the number entity "Sauna target temperature" works too.

## What can go wrong
- Knob flickers / resets / shows garbage → the relay is adding too much latency or
  letting glitch bytes through. Log the counters and tell me; the fallback is a
  frame-level relay instead of chunk-level.
- Link on but no digits → display asleep; touch the knob.
- Heating state wrong → swap `type == 0x02` in `apply_glyph()`.

## Protocol summary (from the logic-analyzer captures)
- 921600 baud 8N1, RS485 A/B on terminals 1/2, +5 V on 3, GND on 4
- CRC-16/CCITT-FALSE (0x1021, init 0xFFFF), little-endian, no bit reflection
- panel → controller, every 20 ms: `01 <button> <encoder> 00 00` + CRC
  - button 01 while pressed; encoder = 8-bit absolute counter, 2 counts per detent
- controller → panel: `<type> x y w h lenLo lenHi 00 00` + CRC, then `len` bytes;
  type 01 = glyph bitmap, type 02 = fill rect (8-byte params);
  all-zero command = nothing to draw. Panel acks by echoing the CRC / `AA A2`.
- screen 128×160: current temp big digits at (39,62),(67,62) 23×34; target at
  (64|81|98, 6) 17×22; full-screen repaint on start/stop.
