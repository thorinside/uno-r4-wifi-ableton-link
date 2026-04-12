# Ableton Link to Eurorack Clock (Arduino UNO R4 WiFi)

Firmware that turns an Arduino UNO R4 WiFi into a 48 PPQN Eurorack clock generator synchronized to [Ableton Link](https://www.ableton.com/en/link/) via WiFi multicast.

## Features

- **48 PPQN clock output** on D2 (5V logic, 2ms pulse width)
- **Follow-only** Link client — reads tempo/phase, never influences the session
- **LED matrix BPM display** — 3-digit readout with beat flash (50ms full white)
- **Phase-aligned tempo changes** using Link beat anchor
- **Selectable PPQN** via potentiometer on A0 (1, 2, 4, 8, 12, 24, 48)
- **RUN gate** on D4 — HIGH when Link transport is playing
- **Automatic fallback** to 120 BPM before Link connection
- **Disconnect indicator** — walking dot on bottom LED row when no Link peer

## Hardware

| Component | Pin | Notes |
|---|---|---|
| Clock output | D2 | 5V logic; use a level shifter for Eurorack |
| RUN gate | D4 | HIGH when Link transport is playing |
| PPQN select | A0 | Potentiometer, CCW=1 PPQN, CW=48 PPQN |
| LED matrix | Built-in 12x8 | BPM display + beat feedback |
| WiFi | Built-in ESP32-S3 | Joins your local network |

## Setup

1. Install [arduino-cli](https://arduino.github.io/arduino-cli/) and the `arduino:renesas_uno` core
2. Edit `link_clock/link_clock.ino` and set your WiFi credentials:
   ```c
   const char* WIFI_SSID = "YOUR_SSID";
   const char* WIFI_PASS = "YOUR_PASSWORD";
   ```
3. Build and upload:
   ```bash
   make compile
   make upload     # auto-releases serial port
   # or
   make deploy     # compile + upload
   ```
   Override the Arduino CLI path or serial port if needed:
   ```bash
   make upload ACLI=/path/to/arduino-cli PORT=/dev/ttyACM0
   ```
4. Open the serial monitor to verify:
   ```bash
   make monitor
   ```
   Expected output:
   ```
   Connecting to WiFi...
   IP: 192.168.x.x
   Listening for Ableton Link on 224.76.78.75:20808
   ```

## How It Works

The device listens on UDP multicast `224.76.78.75:20808` for Ableton Link timeline packets. Clock pulses are derived directly from the Link timeline each loop iteration — no free-running timer, so the output stays locked to the session.

### Clock Output

| Parameter | Value |
|---|---|
| PPQN | 1-48 (selectable via A0) |
| Pulse width | 2ms HIGH |
| Frequency at 120 BPM, 48 PPQN | 96 Hz |
| Frequency formula | `f = (BPM / 60) * PPQN` |

### LED Matrix Layout

```
Row 0:       [link indicator at col 11]
Rows 1-5:    [3-digit BPM display]
Row 6:       [beat flash bar]
Row 7:       [disconnected walking dot]
```

### Ableton Link Protocol

- **Multicast group:** `224.76.78.75:20808`
- **Packet magic:** `_asdp_v\x01` (bytes 0-7)
- **Message types:** `0x01` state, `0x03` disconnect
- **TLV payload:** `tmln` section carries tempo (us/beat), beat anchor, time anchor
- **Tempo conversion:** `BPM = 60,000,000 / tempo_us`

## Verification

1. **Power on** — matrix shows `120` (default BPM), D2 outputs 96 Hz
2. **WiFi connects** — serial monitor prints IP
3. **Start Ableton Live** with Link enabled on the same network — BPM updates within 1s, link pixel lights, beat flash syncs
4. **Change tempo** in Live — clock updates within 1s
5. **Disable Link** — after 3s timeout, link pixel darkens, walking dot appears, clock continues at last tempo

## Tests

Native C++ unit tests (no hardware required):

```bash
cd test
make test
```

## Files

```
link_clock/link_clock.ino   # All firmware code
Makefile                     # Build/upload targets
test/test_link_clock.cpp     # Unit tests
test/Makefile                # Test build
```

## License

[MIT](LICENSE)

---

Built with [Claude Code](https://claude.ai/code)
