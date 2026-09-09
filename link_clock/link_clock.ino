/*
  Ableton Link -> Eurorack Clock Generator
  Arduino UNO R4 WiFi

  Follow-only Link client: reads tempo/phase from Link session via UDP
  multicast, outputs configurable PPQN clock on D2 with 2ms pulse width.
  PPQN is fixed at 48 (PPQN_FIXED); set PPQN_POT to 1 to select it with a pot on A0.
  Advertises presence (peer count) but never sends timeline data.

  Clock pulses are derived directly from the Link timeline each loop
  iteration — no free-running timer, no phase correction needed.
*/

#include <WiFiS3.h>
#include <WiFiUdp.h>
#include "Arduino_LED_Matrix.h"

// ─── Configuration ──────────────────────────────────────

const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";

#define CLOCK_PIN        2
#define RUN_PIN          4
// PPQN selection. With PPQN_POT set to 1 a pot on A0 picks from PPQN_OPTIONS
// (CCW=1, CW=48). With no pot fitted A0 floats and lands on random settings
// (usually 2 PPQN), so leave PPQN_POT at 0 to lock the output to PPQN_FIXED.
#define PPQN_POT         0
#define PPQN_FIXED       48
static const int PPQN_OPTIONS[] = { 1, 2, 4, 8, 12, 24, 48 };
static const int PPQN_COUNT = sizeof(PPQN_OPTIONS) / sizeof(PPQN_OPTIONS[0]);
#define DEFAULT_BPM      120.0f
#define LINK_TIMEOUT_MS  3000UL
#define FLASH_MS         50

const IPAddress LINK_MCAST(224, 76, 78, 75);
const uint16_t  LINK_PORT = 20808;

static const uint8_t PEER_ID[8] = {
  0x55, 0x4E, 0x4F, 0x52, 0x34, 0x4C, 0x4B, 0x00  // "UNOR4LK\0"
};

// ─── Global State ───────────────────────────────────────

// Clock offset: bridges local micros64() to sender's time domain
int64_t  g_clock_offset     = 0;
bool     g_clock_calibrated = false;
int64_t  g_last_time_origin = 0;  // detect time_origin changes for recalibration

// Link timeline parameters (raw, from most recent packet)
int64_t  g_tl_time_origin = 0;
int64_t  g_tl_beat_origin = 0;  // microbeats

// Timeline (derived)
float    g_bpm           = DEFAULT_BPM;
int64_t  g_tempo_us      = 500000LL;
bool     g_tempo_changed = false;

// PPQN (fixed, or configurable via A0 when PPQN_POT is 1)
int g_ppqn       = PPQN_FIXED;
int g_ppqn_index = PPQN_COUNT - 1;

// Link session
unsigned long g_last_peer_ms = 0;
bool     g_link_active       = false;
bool     g_is_playing        = false;
uint8_t  g_session_id[8]     = {0};
bool     g_session_valid     = false;

// Clock output — derived each loop from timeline
int  g_last_qpos       = -1;
int  g_beat_in_quantum = 0;
bool g_beat_flag       = false;
int64_t g_dbg_total_beats = 0;  // for beat-1 diagnosis

// Display
byte     g_frame[8][12];
bool     g_frame_dirty       = false;
int      g_displayed_bpm     = -1;
bool     g_displayed_link    = false;
bool     g_displayed_playing = false;
bool     g_flash_active      = false;
unsigned long g_flash_start_ms = 0;
int      g_dot_col           = 0;
unsigned long g_dot_ms       = 0;
int      g_last_flash_biq    = -1;
unsigned long g_last_flash_ms = 0;
int      g_displayed_progress = -1;

// Presence broadcast
unsigned long g_last_broadcast_ms = 0;

// Pulse tracking (loop sets HIGH on tick, LOW after pulse width)
unsigned long g_pulse_start_us = 0;
bool          g_pulse_active   = false;
unsigned long g_pulse_width_us = 2000;  // 2ms default

// Measurement (host-to-ghost clock offset via ping/pong)
IPAddress     g_mep_ip;
uint16_t      g_mep_port        = 0;
bool          g_mep_valid       = false;
WiFiUDP       g_measure_udp;
int64_t       g_host_to_ghost   = 0;  // ghost = host + this (microseconds)
bool          g_ghost_valid     = false;
int64_t       g_ping_host_time  = 0;  // host time when last ping sent
unsigned long g_last_ping_ms    = 0;
int64_t       g_prev_ghost_time = 0;  // __gt from last pong, sent as _pgt
bool          g_have_prev_ghost = false;

// Hardware
ArduinoLEDMatrix matrix;
WiFiUDP          g_udp;
uint8_t          g_pkt[256];

// ─── 64-bit Microsecond Counter ─────────────────────────

static uint32_t s_micros_prev = 0;
static uint32_t s_micros_hi   = 0;

uint64_t micros64() {
  noInterrupts();
  uint32_t now = micros();
  if (now < s_micros_prev) s_micros_hi++;
  s_micros_prev = now;
  uint64_t result = ((uint64_t)s_micros_hi << 32) | now;
  interrupts();
  return result;
}

// ─── Font ───────────────────────────────────────────────

static const byte FONT_3X5[10][5] = {
  {0b111, 0b101, 0b101, 0b101, 0b111},  // 0
  {0b010, 0b110, 0b010, 0b010, 0b111},  // 1
  {0b111, 0b001, 0b111, 0b100, 0b111},  // 2
  {0b111, 0b001, 0b011, 0b001, 0b111},  // 3
  {0b101, 0b101, 0b111, 0b001, 0b001},  // 4
  {0b111, 0b100, 0b111, 0b001, 0b111},  // 5
  {0b111, 0b100, 0b111, 0b101, 0b111},  // 6
  {0b111, 0b001, 0b001, 0b001, 0b001},  // 7
  {0b111, 0b101, 0b111, 0b101, 0b111},  // 8
  {0b111, 0b101, 0b111, 0b001, 0b111},  // 9
};

void drawDigit(int digit, int col0, int row0) {
  digit = constrain(digit, 0, 9);
  for (int r = 0; r < 5; r++) {
    for (int c = 0; c < 3; c++) {
      int bit = (FONT_3X5[digit][r] >> (2 - c)) & 1;
      int row = row0 + r, col = col0 + c;
      if (row >= 0 && row < 8 && col >= 0 && col < 12)
        g_frame[row][col] = bit;
    }
  }
}

// ─── Pulse Width ────────────────────────────────────────

void updatePulseWidth() {
  float freq_hz = (g_bpm / 60.0f) * g_ppqn;
  float max_pulse_us = 400000.0f / freq_hz;  // 40% duty cap
  g_pulse_width_us = (max_pulse_us < 2000.0f) ? (unsigned long)max_pulse_us : 2000UL;
}

// ─── Timeline → Pulse Position ──────────────────────────

// Computes the current quantum position (0..4*ppqn-1) directly from
// the Link timeline, or from local time if not yet calibrated.
int computeCurrentQPos() {
  int ppqn = g_ppqn;
  int64_t tempo = g_tempo_us;
  int64_t elapsed;
  int64_t beat_origin_ub;

  if (g_ghost_valid) {
    // Measurement-based: ghost = host + offset (drift-free)
    int64_t ghost_now = (int64_t)micros64() + g_host_to_ghost;
    elapsed = ghost_now - g_tl_time_origin;
    beat_origin_ub = g_tl_beat_origin;
  } else if (g_clock_calibrated) {
    // Fallback: single-sample offset from first tmln (drifts over time)
    int64_t sender_now = (int64_t)micros64() - g_clock_offset;
    elapsed = sender_now - g_tl_time_origin;
    beat_origin_ub = g_tl_beat_origin;
  } else {
    elapsed = (int64_t)micros64();
    beat_origin_ub = 0;
  }

  // Floor division for elapsed beats
  int64_t elapsed_beats, remainder_us;
  if (elapsed >= 0) {
    elapsed_beats = elapsed / tempo;
    remainder_us  = elapsed % tempo;
  } else {
    elapsed_beats = (elapsed - tempo + 1) / tempo;
    remainder_us  = elapsed - elapsed_beats * tempo;
  }

  // Current beat position in microbeats (split multiply to avoid overflow)
  int64_t elapsed_ub = elapsed_beats * 1000000LL
                     + remainder_us * 1000000LL / tempo;
  int64_t beat_ub = beat_origin_ub + elapsed_ub;

  g_dbg_total_beats = beat_ub / 1000000LL;

  // Quantum 4 assumed (matches Ableton default). Quantum is a per-peer
  // choice, not transmitted in the protocol. Change 4 to match your session.
  int64_t quantum_ub = 4 * 1000000LL;
  int64_t phase_ub = beat_ub % quantum_ub;
  if (phase_ub < 0) phase_ub += quantum_ub;

  int biq = (int)(phase_ub / 1000000LL);
  int64_t beat_frac_ub = phase_ub % 1000000LL;
  int pulse = (int)(beat_frac_ub * ppqn / 1000000LL);

  return biq * ppqn + pulse;
}

// ─── Byte-Order Utilities ───────────────────────────────

int64_t readBE64(const uint8_t* p) {
  return ((int64_t)p[0] << 56) | ((int64_t)p[1] << 48) |
         ((int64_t)p[2] << 40) | ((int64_t)p[3] << 32) |
         ((int64_t)p[4] << 24) | ((int64_t)p[5] << 16) |
         ((int64_t)p[6] <<  8) |  (int64_t)p[7];
}

uint32_t readBE32(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
         ((uint32_t)p[2] <<  8) |  (uint32_t)p[3];
}

void writeBE32(uint8_t* p, uint32_t v) {
  p[0] = (v >> 24) & 0xFF;  p[1] = (v >> 16) & 0xFF;
  p[2] = (v >>  8) & 0xFF;  p[3] = v & 0xFF;
}

void writeBE64(uint8_t* p, int64_t v) {
  p[0] = (v >> 56) & 0xFF;  p[1] = (v >> 48) & 0xFF;
  p[2] = (v >> 40) & 0xFF;  p[3] = (v >> 32) & 0xFF;
  p[4] = (v >> 24) & 0xFF;  p[5] = (v >> 16) & 0xFF;
  p[6] = (v >>  8) & 0xFF;  p[7] = v & 0xFF;
}

// ─── Link Packet Parser ────────────────────────────────

void parseLinkPacket(const uint8_t* buf, int len) {
  if (len < 20) return;
  if (memcmp(buf, "_asdp_v\x01", 8) != 0) return;

  uint8_t msg_type = buf[8];
  if (msg_type != 0x01 && msg_type != 0x02) return;

  // Filter self-packets by peer_id at bytes 12-19
  if (memcmp(buf + 12, PEER_ID, 8) == 0) return;

  int offset = 20;
  while (offset + 8 <= len) {
    uint32_t plen = readBE32(buf + offset + 4);
    int payload = offset + 8;
    if (plen > (uint32_t)(len - payload)) break;

    // Log all TLV tags for protocol discovery
    {
      char tag[5] = {0};
      memcpy(tag, buf + offset, 4);
      static char s_seen_tags[8][5];
      static int s_seen_count = 0;
      bool seen = false;
      for (int i = 0; i < s_seen_count; i++) {
        if (memcmp(s_seen_tags[i], tag, 4) == 0) { seen = true; break; }
      }
      if (!seen && s_seen_count < 8) {
        memcpy(s_seen_tags[s_seen_count++], tag, 5);
        Serial.print("TLV: tag='");
        Serial.print(tag);
        Serial.print("' len=");
        Serial.println(plen);
      }
    }

    // ── mep4 section (6 bytes: IPv4 addr + port for measurement) ──
    if (memcmp(buf + offset, "mep4", 4) == 0 && plen >= 6) {
      IPAddress ip(buf[payload], buf[payload+1], buf[payload+2], buf[payload+3]);
      uint16_t port = ((uint16_t)buf[payload+4] << 8) | buf[payload+5];
      if (!g_mep_valid || ip != g_mep_ip || port != g_mep_port) {
        g_mep_ip = ip;
        g_mep_port = port;
        g_mep_valid = true;
        g_ghost_valid = false;
        g_have_prev_ghost = false;
        Serial.print("MEP4: ");
        Serial.print(ip);
        Serial.print(":");
        Serial.println(port);
      }
    }

    // ── tmln section (24 bytes: tempo + beat_origin + time_origin) ──
    if (memcmp(buf + offset, "tmln", 4) == 0 && plen >= 24) {
      int64_t new_tempo_us    = readBE64(buf + payload);
      int64_t new_beat_origin = readBE64(buf + payload + 8);
      int64_t new_time_origin = readBE64(buf + payload + 16);

      if (new_tempo_us > 0) {
        uint64_t local_now = micros64();
        g_tempo_us = new_tempo_us;

        // Offset sample: approximates (local - ghost) when time_origin
        // is a recent ghost-time anchor from the session owner.
        int64_t offset_sample = (int64_t)local_now - new_time_origin;
        bool to_changed = (new_time_origin != g_last_time_origin);

        if (!g_clock_calibrated) {
          g_clock_offset = offset_sample;
          g_clock_calibrated = true;
        } else if (to_changed) {
          int64_t error = offset_sample - g_clock_offset;
          if (error > 100000 || error < -100000) {
            // >100ms jump: snap (timeline re-anchor)
            g_clock_offset = offset_sample;
          } else {
            // Drift tracking: EMA with alpha ~ 1/16
            g_clock_offset += error >> 4;
          }
          g_last_qpos = -1;  // reset pulse tracker on any recalibration
        }
        g_last_time_origin = new_time_origin;

        // Debug: log offset tracking
        static unsigned long s_tmln_dbg_ms = 0;
        if (to_changed || millis() - s_tmln_dbg_ms >= 2000) {
          s_tmln_dbg_ms = millis();
          Serial.print("tmln: TO_chg=");
          Serial.print(to_changed ? "Y" : "N");
          Serial.print(" off=");
          Serial.print((long)(g_clock_offset / 1000));
          Serial.print("ms samp=");
          Serial.print((long)(offset_sample / 1000));
          Serial.println("ms");
        }

        // Store raw timeline parameters
        g_tl_time_origin = new_time_origin;
        g_tl_beat_origin = new_beat_origin;

        // Detect BPM change
        float new_bpm = 60000000.0f / (float)new_tempo_us;
        float delta = new_bpm - g_bpm;
        if (delta < -0.01f || delta > 0.01f) {
          g_bpm           = new_bpm;
          g_tempo_changed = true;
        }

        g_last_peer_ms = millis();
      }
    }

    // ── sess section (8 bytes: session_id) ──
    if (memcmp(buf + offset, "sess", 4) == 0 && plen >= 8) {
      memcpy(g_session_id, buf + payload, 8);
      g_session_valid = true;
    }

    // ── stst section (17 bytes: isPlaying + beats + timestamp) ──
    if (memcmp(buf + offset, "stst", 4) == 0 && plen >= 17) {
      g_is_playing = (buf[payload] != 0);
    }

    offset = payload + (int)plen;
  }
}

// ─── Presence Broadcast (header + sess only) ────────────

int buildPresencePacket(uint8_t* pkt, uint8_t msg_type) {
  memcpy(pkt, "_asdp_v\x01", 8);
  pkt[8] = msg_type;
  pkt[9] = 5;  // TTL
  pkt[10] = pkt[11] = 0;  // groupId
  memcpy(pkt + 12, PEER_ID, 8);
  int p = 20;

  memcpy(pkt + p, "sess", 4);     p += 4;
  writeBE32(pkt + p, 8);          p += 4;
  memcpy(pkt + p, g_session_id, 8); p += 8;

  return p;  // 36 bytes total
}

void sendPresenceBroadcast() {
  uint8_t pkt[64];
  int len = buildPresencePacket(pkt, 0x01);
  g_udp.beginPacket(LINK_MCAST, LINK_PORT);
  g_udp.write(pkt, len);
  g_udp.endPacket();
}

void sendPresenceResponse(IPAddress ip) {
  uint8_t pkt[64];
  int len = buildPresencePacket(pkt, 0x02);
  g_udp.beginPacket(ip, LINK_PORT);
  g_udp.write(pkt, len);
  g_udp.endPacket();
}

// ─── Display ────────────────────────────────────────────

void drawBPM(int bpm) {
  bpm = constrain(bpm, 0, 999);
  memset(g_frame, 0, 7 * 12);  // clear rows 0-6 only

  drawDigit(bpm / 100,       0, 1);
  drawDigit((bpm / 10) % 10, 4, 1);
  drawDigit(bpm % 10,        8, 1);

  g_frame[0][0]  = g_is_playing ? 1 : 0;   // play indicator
  g_frame[0][11] = g_link_active ? 1 : 0;  // link indicator
  g_frame_dirty = true;
}

void drawBeatFlash(int cols) {
  for (int c = 0; c < 12; c++)
    g_frame[6][c] = (c < cols) ? 1 : 0;
  g_frame_dirty = true;
}

void drawProgressBar(int filled) {
  for (int c = 0; c < 12; c++)
    g_frame[7][c] = (c < filled) ? 1 : 0;
  g_frame_dirty = true;
}

void drawDisconnectedDot(int col) {
  for (int c = 0; c < 12; c++)
    g_frame[7][c] = 0;
  g_frame[7][col] = 1;
  g_frame_dirty = true;
}

void renderIfDirty() {
  if (g_frame_dirty) {
    matrix.renderBitmap(g_frame, 8, 12);
    g_frame_dirty = false;
  }
}

// ─── Measurement (clock sync via ping/pong) ────────────

void sendMeasurementPing() {
  uint8_t pkt[64];
  memcpy(pkt, "_link_v\x01", 8);
  pkt[8] = 0x01;  // kPing
  int p = 9;

  // TLV: __ht (our host time)
  g_ping_host_time = (int64_t)micros64();
  memcpy(pkt + p, "__ht", 4); p += 4;
  writeBE32(pkt + p, 8);      p += 4;
  writeBE64(pkt + p, g_ping_host_time); p += 8;

  // TLV: _pgt (previous ghost time from last pong)
  if (g_have_prev_ghost) {
    memcpy(pkt + p, "_pgt", 4); p += 4;
    writeBE32(pkt + p, 8);      p += 4;
    writeBE64(pkt + p, g_prev_ghost_time); p += 8;
  }

  g_measure_udp.beginPacket(g_mep_ip, g_mep_port);
  g_measure_udp.write(pkt, p);
  g_measure_udp.endPacket();
}

void handleMeasurementPong(const uint8_t* buf, int len) {
  if (len < 9) return;
  if (memcmp(buf, "_link_v\x01", 8) != 0) return;
  if (buf[8] != 0x02) return;  // kPong

  int64_t ghost_time = 0;
  int64_t echoed_host_time = 0;
  bool got_gt = false, got_ht = false;

  int off = 9;
  while (off + 8 <= len) {
    uint32_t plen = readBE32(buf + off + 4);
    int pl = off + 8;
    if (plen > (uint32_t)(len - pl)) break;

    if (memcmp(buf + off, "__gt", 4) == 0 && plen >= 8) {
      ghost_time = readBE64(buf + pl);
      got_gt = true;
    }
    if (memcmp(buf + off, "__ht", 4) == 0 && plen >= 8) {
      echoed_host_time = readBE64(buf + pl);
      got_ht = true;
    }
    off = pl + (int)plen;
  }

  if (got_gt) {
    g_prev_ghost_time = ghost_time;
    g_have_prev_ghost = true;
  }

  if (got_gt && got_ht) {
    int64_t local_recv = (int64_t)micros64();
    // NTP-style: offset = ghost_at_peer_receive - midpoint(our_send, our_recv)
    int64_t host_mid = echoed_host_time / 2 + local_recv / 2;
    int64_t sample = ghost_time - host_mid;

    if (!g_ghost_valid) {
      g_host_to_ghost = sample;
      g_ghost_valid = true;
      g_last_qpos = -1;  // re-derive pulse position from measurement
      Serial.print("GHOST: initial off=");
      Serial.print((long)(sample / 1000));
      Serial.println("ms");
    } else {
      // EMA with alpha ~ 1/8 for measurement samples
      g_host_to_ghost += (sample - g_host_to_ghost) >> 3;
    }
  }
}

// ─── Setup ──────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(CLOCK_PIN, OUTPUT);
  digitalWrite(CLOCK_PIN, LOW);
  pinMode(RUN_PIN, OUTPUT);
  digitalWrite(RUN_PIN, LOW);

  matrix.begin();
  memset(g_frame, 0, sizeof(g_frame));

  updatePulseWidth();

  drawBPM((int)DEFAULT_BPM);
  renderIfDirty();

  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  while (WiFi.localIP() == IPAddress(0, 0, 0, 0)) { delay(200); }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  g_udp.beginMulticast(LINK_MCAST, LINK_PORT);
  g_measure_udp.begin(20809);
  Serial.println("Listening for Ableton Link on 224.76.78.75:20808");
}

// ─── Loop ───────────────────────────────────────────────

void loop() {
  unsigned long now = millis();

  // ── 0. Pulse end ──
  if (g_pulse_active && (micros() - g_pulse_start_us >= g_pulse_width_us)) {
    digitalWrite(CLOCK_PIN, LOW);
    g_pulse_active = false;
  }

  // ── 1. Compute pulse position and fire on boundary ──
  int qpos = computeCurrentQPos();
  if (qpos != g_last_qpos) {
    digitalWrite(CLOCK_PIN, HIGH);
    g_pulse_start_us = micros();
    g_pulse_active = true;

    int ppqn = g_ppqn;
    int curr_beat = qpos / ppqn;
    if (g_last_qpos < 0 || curr_beat != g_last_qpos / ppqn) {
      g_beat_in_quantum = curr_beat;
      g_beat_flag = true;
      Serial.print("BEAT: biq=");
      Serial.print(curr_beat);
      Serial.print(" tb=");
      Serial.print((long)g_dbg_total_beats);
      Serial.print(" BO=");
      Serial.print((long)(g_tl_beat_origin / 1000));
      Serial.print(" qpos=");
      Serial.println(qpos);
    }
    g_last_qpos = qpos;
  }

  // ── 2. Link timeout ──
  bool was_active = g_link_active;
  g_link_active = (g_last_peer_ms > 0) && (now - g_last_peer_ms < LINK_TIMEOUT_MS);
  if (was_active && !g_link_active) {
    g_is_playing    = false;
    g_displayed_bpm = -1;
  } else if (!was_active && g_link_active) {
    g_displayed_bpm = -1;
  }

  // ── 3. RUN gate (waits for beat 1 to start) ──
  static bool s_was_playing = false;
  static bool s_run_gate = false;
  if (g_is_playing && !s_was_playing) {
    s_run_gate = false;
    digitalWrite(RUN_PIN, LOW);
  } else if (!g_is_playing && s_was_playing) {
    s_run_gate = false;
    digitalWrite(RUN_PIN, LOW);
  }
  if (g_is_playing && !s_run_gate && g_beat_flag && g_beat_in_quantum == 0) {
    s_run_gate = true;
    digitalWrite(RUN_PIN, HIGH);
  }
  s_was_playing = g_is_playing;

  // ── 4. WiFi reconnection (every 5s) ──
  static unsigned long s_wifi_check_ms = 0;
  static bool s_wifi_lost = false;
  if (now - s_wifi_check_ms >= 5000) {
    s_wifi_check_ms = now;
    if (WiFi.status() != WL_CONNECTED) {
      if (!s_wifi_lost) {
        s_wifi_lost = true;
        Serial.println("WiFi lost, reconnecting...");
      }
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    } else if (s_wifi_lost) {
      s_wifi_lost = false;
      g_udp.stop();
      g_udp.beginMulticast(LINK_MCAST, LINK_PORT);
      Serial.print("WiFi reconnected. IP: ");
      Serial.println(WiFi.localIP());
    }
  }

  // ── 5. PPQN from A0 pot ──
#if PPQN_POT
  static unsigned long s_ppqn_check_ms = 0;
  static int s_ppqn_candidate = -1;
  static int s_ppqn_confirm = 0;
  if (now - s_ppqn_check_ms >= 50) {
    s_ppqn_check_ms = now;
    int idx = (analogRead(A0) * PPQN_COUNT) / 1024;
    if (idx >= PPQN_COUNT) idx = PPQN_COUNT - 1;
    if (idx == s_ppqn_candidate) {
      s_ppqn_confirm++;
    } else {
      s_ppqn_candidate = idx;
      s_ppqn_confirm = 1;
    }
    if (s_ppqn_confirm >= 3 && idx != g_ppqn_index) {
      g_ppqn_index = idx;
      digitalWrite(CLOCK_PIN, LOW);
      g_pulse_active = false;
      g_ppqn = PPQN_OPTIONS[idx];
      g_last_qpos = -1;
      updatePulseWidth();
      g_displayed_bpm = -1;
      Serial.print("PPQN: ");
      Serial.println(g_ppqn);
    }
  }
#endif

  // ── 6. UDP receive ──
  int pktSize = g_udp.parsePacket();
  if (pktSize > 0) {
    int n = g_udp.read(g_pkt, sizeof(g_pkt));
    if (n > 0) {
      parseLinkPacket(g_pkt, n);
      if (g_session_valid && n >= 20 && memcmp(g_pkt, "_asdp_v\x01", 8) == 0
          && g_pkt[8] == 0x01 && memcmp(g_pkt + 12, PEER_ID, 8) != 0) {
        sendPresenceResponse(g_udp.remoteIP());
      }
    }
  }

  // ── 7. Measurement ping/pong ──
  if (g_mep_valid && now - g_last_ping_ms >= 250) {
    g_last_ping_ms = now;
    sendMeasurementPing();
  }
  {
    int mSize = g_measure_udp.parsePacket();
    if (mSize > 0) {
      uint8_t mbuf[128];
      int mn = g_measure_udp.read(mbuf, sizeof(mbuf));
      if (mn > 0) handleMeasurementPong(mbuf, mn);
    }
  }

  // ── 8. Presence broadcast ──
  if (g_session_valid && now - g_last_broadcast_ms >= 1000) {
    g_last_broadcast_ms = now;
    sendPresenceBroadcast();
  }

  // ── 8. Tempo change ──
  if (g_tempo_changed) {
    g_tempo_changed = false;
    updatePulseWidth();
  }

  // ── 9. Beat flash ���─
  int  snap_biq  = g_beat_in_quantum;
  bool snap_beat = g_beat_flag;
  g_beat_flag    = false;

  if (snap_beat) {
    // Duplicate suppression: skip if same biq fired too recently
    unsigned long beat_min_ms = (unsigned long)(g_tempo_us / 2000);
    bool is_dup = (snap_biq == g_last_flash_biq
                   && now - g_last_flash_ms < beat_min_ms);

    if (!is_dup) {
      g_last_flash_biq = snap_biq;
      g_last_flash_ms  = now;
      g_flash_active   = true;
      g_flash_start_ms = now;

      int bpm_int = (int)roundf(g_bpm);
      g_displayed_bpm     = bpm_int;
      g_displayed_link    = g_link_active;
      g_displayed_playing = g_is_playing;
      drawBPM(bpm_int);

      int flash_cols = (snap_biq == 0) ? 12 : 3;
      drawBeatFlash(flash_cols);
    }
  }

  if (g_flash_active && (now - g_flash_start_ms >= FLASH_MS)) {
    g_flash_active  = false;
    g_displayed_bpm = -1;  // force redraw to clear row 6
  }

  // ── 10. Steady-state display ──
  if (!g_flash_active) {
    int bpm_int = (int)roundf(g_bpm);
    if (bpm_int != g_displayed_bpm || g_link_active != g_displayed_link
        || g_is_playing != g_displayed_playing) {
      g_displayed_bpm     = bpm_int;
      g_displayed_link    = g_link_active;
      g_displayed_playing = g_is_playing;
      drawBPM(bpm_int);
    }

    if (g_link_active && g_clock_calibrated) {
      int cols = ((qpos + 1) * 12) / (4 * g_ppqn);
      if (cols != g_displayed_progress) {
        g_displayed_progress = cols;
        drawProgressBar(cols);
      }
    } else if (!g_link_active) {
      g_displayed_progress = -1;
      if (now - g_dot_ms > 200) {
        g_dot_ms = now;
        g_dot_col = (g_dot_col + 1) % 12;
        drawDisconnectedDot(g_dot_col);
      }
    }
  }

  // ── 11. Single render ──
  // Debug status (every 2s)
  static unsigned long s_loop_dbg_ms = 0;
  if (now - s_loop_dbg_ms >= 2000) {
    s_loop_dbg_ms = now;
    Serial.print("loop: qpos=");
    Serial.print(qpos);
    Serial.print("/");
    Serial.print(4 * g_ppqn);
    Serial.print(" biq=");
    Serial.print(g_beat_in_quantum);
    if (g_ghost_valid) {
      Serial.print(" ghost=");
      Serial.print((long)(g_host_to_ghost / 1000));
      Serial.print("ms");
    }
    Serial.print(" ppqn=");
    Serial.println(g_ppqn);
  }

  renderIfDirty();
}
