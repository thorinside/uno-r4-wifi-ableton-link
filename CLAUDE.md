# link_clock

Arduino UNO R4 WiFi firmware that follows an Ableton Link WiFi session and outputs
a 48 PPQN Eurorack clock on D2.

## Hardware

- **Board:** Arduino UNO R4 WiFi, FQBN `arduino:renesas_uno:unor4wifi`
- **Clock output:** D2 (digital, 5V logic, 2ms pulse width)
- **LED matrix:** 12×8, built-in

## Key Design Points

- **Follow-only** Link client — reads tempo from Link session, never advertises/influences
- **48 PPQN** output clock (fixed via `PPQN_FIXED`; `PPQN_POT=1` enables the A0 selector pot)
- **Pulse scheduler** — one FspTimer ticks at 20 kHz; its ISR raises D2 when the tick passes the
  scheduled next-pulse time (Q16 fixed-point ticks, so fractional periods don't drift) and lowers
  it after the pulse width. Never generate pulses from loop(): WiFiS3 calls block for ms at a time.
- **Phase alignment** — each loop pass derives the timeline phase from the Link tmln (+ ghost
  offset measurement) and nudges the scheduler: slews 1/8 of the error per pass, snaps when the
  error exceeds 3 ms or the timeline re-anchors (`g_force_snap`)
- **2 ms pulse width**, capped at 40% duty for high PPQN × BPM
- **BPM display** on LED matrix (rows 1–5, 3-digit 3×5 font at cols 0, 4, 8)
- **Beat flash** — 50ms full-white matrix flash when pulse_counter rolls to 0 (beat boundary)
- **Disconnected indicator** — link-active pixel at row 0 col 11 lights when receiving Link packets within 3s timeout; walking dot on row 7 when disconnected
- **Default behavior** — outputs 120 BPM clock from power-on, before WiFi connects

## Build

```bash
make compile    # compile sketch via arduino-cli
make upload     # upload (auto-releases serial port)
make deploy     # compile + upload
make monitor    # open serial monitor at 115200
```

## Ableton Link Protocol (UDP Multicast)

- **Multicast group:** `224.76.78.75:20808`
- **Packet magic:** `_asdp_v\x01` (bytes 0–7)
- **Message type:** byte 8 (0x01 = state, 0x03 = disconnect)
- **TLV sections** starting at offset 20:
  - `tmln` (24 bytes): tempo (μs/beat), beat_anchor (μs), time_anchor (μs)
  - Other sections parsed but ignored
- **Tempo→BPM:** `BPM = 60,000,000 / tempo_us`

## Files

- `link_clock/link_clock.ino` — all firmware code
- `Makefile` — build/upload targets
- `CLAUDE.md` — this file

## Verification Steps

1. **Compile:** `make compile` — expect zero warnings
2. **Bench test (without WiFi):** oscilloscope on D2 at 120 BPM should show 96 Hz, 2ms HIGH, ~8.4ms LOW
3. **Upload:** `make deploy` — serial monitor shows IP + "Listening for Ableton Link"
4. **Link sync:** start Ableton Live with Link enabled on the same WiFi network → matrix updates to Live's BPM within 1s, link pixel lights, beat flash rate matches Live
5. **Tempo change:** change BPM in Live → matrix updates within 1s
6. **Disconnect:** quit Live or disable Link → after 3s, link pixel goes dark, walking dot appears on row 7, clock continues at last tempo
