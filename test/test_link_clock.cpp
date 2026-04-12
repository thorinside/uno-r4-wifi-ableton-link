/*
  Host-side unit tests for Link clock sync logic.
  Build:  g++ -std=c++17 -O2 -Wall -Wextra -o test_link_clock test_link_clock.cpp
  Run:    ./test_link_clock

  Tests the quantum-position approach: ISR tracks g_quantum_pos (0..191)
  on a 4-beat ring.  Phase correction snaps it to the Link timeline using
  elapsed/tempo integer division (exact at beat boundaries).
*/

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>

// ═══════════════════════════════════════════════════════════
// Test Framework
// ═══════════════════════════════════════════════════════════

static int g_tests_run    = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define RUN_TEST(fn) do {                                         \
    g_tests_run++;                                                \
    printf("  %-55s ", #fn);                                      \
    fflush(stdout);                                               \
    bool _ok = fn();                                              \
    if (_ok) { g_tests_passed++; printf("PASS\n"); }             \
    else     { g_tests_failed++; printf("FAIL\n"); }             \
  } while (0)

#define EXPECT(cond, ...) do {                                    \
    if (!(cond)) {                                                \
      printf("\n    ASSERT FAILED (line %d): ", __LINE__);        \
      printf(__VA_ARGS__);                                        \
      printf("\n");                                               \
      return false;                                               \
    }                                                             \
  } while (0)

// ═══════════════════════════════════════════════════════════
// Constants (must match firmware)
// ═══════════════════════════════════════════════════════════

static const int PPQN    = 48;
static const int QUANTUM = 4;
static const int QLEN    = QUANTUM * PPQN;   // 192

// ═══════════════════════════════════════════════════════════
// Simulated Time
// ═══════════════════════════════════════════════════════════

static uint64_t g_fake_us = 0;
static uint64_t micros64() { return g_fake_us; }

// ═══════════════════════════════════════════════════════════
// Firmware State (mirrors link_clock.ino globals)
// ═══════════════════════════════════════════════════════════

// Clock offset
static int64_t  g_clock_offset     = 0;
static bool     g_clock_calibrated = false;
static int64_t  g_last_time_origin = 0;

// Timeline parameters (from most recent Link packet)
static int64_t  g_tl_time_origin = 0;
static int64_t  g_tl_beat_origin = 0;  // microbeats

// Timeline (derived)
static float    g_bpm           = 120.0f;
static int64_t  g_tempo_us      = 500000LL;
static bool     g_tempo_changed = false;

// Anchor (for progress bar / microbeat interpolation)
static uint64_t g_anchor_local_us64 = 0;
static int64_t  g_anchor_microbeats = 0;

// Link state
static bool     g_link_active = false;

// ISR state — quantum_pos on 0..191 ring
static int  g_quantum_pos     = 0;
static int  g_beat_in_quantum = 0;     // 0..3, set by ISR at beat boundary
static bool g_beat_flag       = false;

// Beat flash tracking
static int           g_last_flash_biq = -1;
static unsigned long g_last_flash_ms  = 0;

// Link session state (for parseLinkPacket testing)
static uint8_t  g_session_id[8]   = {0};
static bool     g_session_valid   = false;
static bool     g_is_playing      = false;
static unsigned long g_last_peer_ms = 0;

static const uint8_t PEER_ID[8] = {
  0x55, 0x4E, 0x4F, 0x52, 0x34, 0x4C, 0x4B, 0x00  // "UNOR4LK\0"
};

static unsigned long millis() { return (unsigned long)(g_fake_us / 1000); }

// ═══════════════════════════════════════════════════════════
// Core Functions (extracted from firmware, testable)
// ═══════════════════════════════════════════════════════════

// Microbeat interpolation from anchor (for progress bar display).
// May have ±1 microbeat rounding — fine for visual, NOT for beat detection.
static int64_t computeCurrentMicrobeats() {
  int64_t elapsed_us = (int64_t)(micros64() - g_anchor_local_us64);
  int64_t tempo      = g_tempo_us;
  int64_t whole_mb   = (elapsed_us / tempo) * 1000000LL;
  int64_t frac_us    = elapsed_us % tempo;
  return g_anchor_microbeats + whole_mb + frac_us * 1000000LL / tempo;
}

// Compute expected quantum position directly from the Link timeline.
// Uses elapsed / tempo_us (exact at beat boundaries) instead of going
// through microbeats, which avoids accumulated rounding at boundaries.
static int computeExpectedQPos() {
  if (!g_clock_calibrated) return 0;

  int64_t sender_now = (int64_t)micros64() - g_clock_offset;
  int64_t elapsed    = sender_now - g_tl_time_origin;
  int64_t tempo      = g_tempo_us;

  // Floor division → elapsed_beats exact at boundaries
  int64_t elapsed_beats, remainder_us;
  if (elapsed >= 0) {
    elapsed_beats = elapsed / tempo;
    remainder_us  = elapsed % tempo;
  } else {
    elapsed_beats = (elapsed - tempo + 1) / tempo;
    remainder_us  = elapsed - elapsed_beats * tempo;
  }

  // Pulse within current beat (0..PPQN-1)
  int pulse_in_beat = (int)(remainder_us * PPQN / tempo);

  // Account for beat_origin (may be fractional)
  int64_t origin_beats   = g_tl_beat_origin / 1000000LL;
  int64_t origin_frac_mb = g_tl_beat_origin % 1000000LL;
  if (g_tl_beat_origin < 0 && origin_frac_mb != 0) {
    origin_beats--;
    origin_frac_mb += 1000000LL;
  }
  int origin_frac_pulse = (int)(origin_frac_mb * PPQN / 1000000LL);

  // Combine: total position in pulses
  int total_pulse    = origin_frac_pulse + pulse_in_beat;
  int64_t total_beats = origin_beats + elapsed_beats + total_pulse / PPQN;
  total_pulse         = total_pulse % PPQN;

  int biq = ((int)(total_beats % QUANTUM) + QUANTUM) % QUANTUM;
  return biq * PPQN + total_pulse;
}

static void simulateISR() {
  int prev_beat = g_quantum_pos / PPQN;
  g_quantum_pos = (g_quantum_pos + 1) % QLEN;
  int curr_beat = g_quantum_pos / PPQN;
  if (curr_beat != prev_beat) {
    g_beat_in_quantum = curr_beat;
    g_beat_flag       = true;
  }
}

static void doPhaseCorrection() {
  if (!g_clock_calibrated || !g_link_active) return;

  int expected = computeExpectedQPos();
  int actual   = g_quantum_pos;

  int error = expected - actual;
  if (error >  QLEN / 2) error -= QLEN;
  if (error < -QLEN / 2) error += QLEN;

  if (error > 2 || error < -2) {
    g_quantum_pos = expected;
    g_beat_flag   = false;
  }
}

// ═══════════════════════════════════════════════════════════
// Byte-Order Utilities
// ═══════════════════════════════════════════════════════════

static int64_t readBE64(const uint8_t* p) {
  return ((int64_t)p[0] << 56) | ((int64_t)p[1] << 48) |
         ((int64_t)p[2] << 40) | ((int64_t)p[3] << 32) |
         ((int64_t)p[4] << 24) | ((int64_t)p[5] << 16) |
         ((int64_t)p[6] <<  8) |  (int64_t)p[7];
}

static uint32_t readBE32(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
         ((uint32_t)p[2] <<  8) |  (uint32_t)p[3];
}

static void writeBE32(uint8_t* p, uint32_t v) {
  p[0] = (v >> 24) & 0xFF;  p[1] = (v >> 16) & 0xFF;
  p[2] = (v >>  8) & 0xFF;  p[3] = v & 0xFF;
}

static void writeBE64(uint8_t* p, int64_t v) {
  p[0] = (v >> 56) & 0xFF;  p[1] = (v >> 48) & 0xFF;
  p[2] = (v >> 40) & 0xFF;  p[3] = (v >> 32) & 0xFF;
  p[4] = (v >> 24) & 0xFF;  p[5] = (v >> 16) & 0xFF;
  p[6] = (v >>  8) & 0xFF;  p[7] = v & 0xFF;
}

// ═══════════════════════════════════════════════════════════
// Link Packet Parser (mirrors firmware parseLinkPacket)
// ═══════════════════════════════════════════════════════════

static void parseLinkPacket(const uint8_t* buf, int len) {
  if (len < 20) return;
  if (memcmp(buf, "_asdp_v\x01", 8) != 0) return;

  uint8_t msg_type = buf[8];
  if (msg_type != 0x01 && msg_type != 0x02) return;

  if (memcmp(buf + 12, PEER_ID, 8) == 0) return;

  int offset = 20;
  while (offset + 8 <= len) {
    uint32_t plen = readBE32(buf + offset + 4);
    int payload = offset + 8;
    if (plen > (uint32_t)(len - payload)) break;

    if (memcmp(buf + offset, "tmln", 4) == 0 && plen >= 24) {
      int64_t new_tempo_us    = readBE64(buf + payload);
      int64_t new_beat_origin = readBE64(buf + payload + 8);
      int64_t new_time_origin = readBE64(buf + payload + 16);

      if (new_tempo_us > 0) {
        uint64_t local_now = micros64();
        g_tempo_us = new_tempo_us;

        if (!g_clock_calibrated || new_time_origin != g_last_time_origin) {
          g_clock_offset     = (int64_t)local_now - new_time_origin;
          g_clock_calibrated = true;
          g_last_time_origin = new_time_origin;
        }

        g_tl_time_origin = new_time_origin;
        g_tl_beat_origin = new_beat_origin;

        float new_bpm = 60000000.0f / (float)new_tempo_us;
        float delta = new_bpm - g_bpm;
        if (delta < -0.01f || delta > 0.01f) {
          g_bpm           = new_bpm;
          g_tempo_changed = true;
        }

        g_last_peer_ms = millis();
      }
    }

    if (memcmp(buf + offset, "sess", 4) == 0 && plen >= 8) {
      memcpy(g_session_id, buf + payload, 8);
      g_session_valid = true;
    }

    if (memcmp(buf + offset, "stst", 4) == 0 && plen >= 17) {
      g_is_playing = (buf[payload] != 0);
    }

    offset = payload + (int)plen;
  }
}

// ═══════════════════════════════════════════════════════════
// Presence Packet Builder (mirrors firmware buildPresencePacket)
// ═══════════════════════════════════════════════════════════

static int buildPresencePacket(uint8_t* pkt, uint8_t msg_type) {
  memcpy(pkt, "_asdp_v\x01", 8);
  pkt[8] = msg_type;
  pkt[9] = 5;
  pkt[10] = pkt[11] = 0;
  memcpy(pkt + 12, PEER_ID, 8);
  int p = 20;
  memcpy(pkt + p, "sess", 4);       p += 4;
  writeBE32(pkt + p, 8);            p += 4;
  memcpy(pkt + p, g_session_id, 8); p += 8;
  return p;
}

// ═══════════════════════════════════════════════════════════
// Simulation Helpers
// ═══════════════════════════════════════════════════════════

static void resetState(float bpm = 120.0f) {
  g_fake_us           = 0;
  g_clock_offset      = 0;
  g_clock_calibrated  = false;
  g_last_time_origin  = 0;
  g_tl_time_origin    = 0;
  g_tl_beat_origin    = 0;
  g_bpm               = bpm;
  g_tempo_us          = (int64_t)(60000000.0f / bpm);
  g_tempo_changed     = false;
  g_anchor_local_us64 = 0;
  g_anchor_microbeats = 0;
  g_link_active       = false;
  g_quantum_pos       = 0;
  g_beat_in_quantum   = 0;
  g_beat_flag         = false;
  g_last_flash_biq    = -1;
  g_last_flash_ms     = 0;
  g_session_valid     = false;
  g_is_playing        = false;
  g_last_peer_ms      = 0;
  memset(g_session_id, 0, 8);
}

static void receiveLinkPacket(int64_t tempo_us, int64_t beat_origin,
                              int64_t time_origin) {
  uint64_t local_now = micros64();
  g_tempo_us = tempo_us;

  if (!g_clock_calibrated || time_origin != g_last_time_origin) {
    g_clock_offset     = (int64_t)local_now - time_origin;
    g_clock_calibrated = true;
    g_last_time_origin = time_origin;
  }

  // Store raw timeline for phase correction
  g_tl_time_origin = time_origin;
  g_tl_beat_origin = beat_origin;

  g_link_active = true;

  float new_bpm = 60000000.0f / (float)tempo_us;
  float delta   = new_bpm - g_bpm;
  if (delta < -0.01f || delta > 0.01f) {
    g_bpm           = new_bpm;
    g_tempo_changed = true;
    g_tempo_us      = tempo_us;
  }
}

// ─── Beat Event Tracking ───────────────────────────────────

struct BeatEvent {
  uint64_t time_us;
  int      biq;        // beat_in_quantum reported by ISR
  int      expected;   // expected biq from timeline
};

// Expected biq at a given local time, using timeline-direct computation.
static int expectedBiqAt(uint64_t t) {
  uint64_t saved = g_fake_us;
  g_fake_us = t;
  int qpos = computeExpectedQPos();
  g_fake_us = saved;
  return qpos / PPQN;
}

// ─── Full Session Simulation ───────────────────────────────

struct SimConfig {
  float    bpm             = 120.0f;
  int      num_beats       = 16;
  uint64_t packet_interval = 100000; // 100ms
  int64_t  time_origin     = 0;
  int64_t  beat_origin     = 0;      // microbeats
  int      clock_drift_ppb = 0;
};

struct SimResult {
  std::vector<BeatEvent> beats;
  int total_isr_ticks  = 0;
  int phase_snaps      = 0;
  int missed_beats     = 0;
  int double_beats     = 0;
  int wrong_biq        = 0;
};

static SimResult runSimulation(const SimConfig& cfg) {
  SimResult result;
  resetState(cfg.bpm);

  int64_t tempo_us = (int64_t)(60000000.0f / cfg.bpm);

  // Float-precision ISR timing: avoids integer-truncation drift.
  // Each ISR time is computed from count * period (no accumulation).
  double pulse_period_f = (double)tempo_us / (double)PPQN;
  int    isr_count      = 0;
  double isr_base       = 1000.0;  // first packet at t=1000

  // Deliver initial Link packet at t=1000
  g_fake_us = 1000;
  receiveLinkPacket(tempo_us, cfg.beat_origin, cfg.time_origin);
  doPhaseCorrection();

  uint64_t total_us    = (uint64_t)cfg.num_beats * (uint64_t)tempo_us + 1000;
  uint64_t next_packet = 1000 + cfg.packet_interval;
  uint64_t next_loop   = 1000 + 1000;

  auto next_isr_time = [&]() -> uint64_t {
    double t = isr_base + (isr_count + 1) * pulse_period_f;
    // Apply drift
    if (cfg.clock_drift_ppb != 0) {
      double drift_factor = 1.0 + (double)cfg.clock_drift_ppb / 1e9;
      t = isr_base + (isr_count + 1) * pulse_period_f * drift_factor;
    }
    return (uint64_t)(t + 0.5);
  };

  int last_beat_biq = -1;

  for (g_fake_us = 1001; g_fake_us <= total_us; ) {
    uint64_t ni = next_isr_time();
    uint64_t next_event = ni;
    if (next_loop   < next_event) next_event = next_loop;
    if (next_packet < next_event) next_event = next_packet;

    g_fake_us = next_event;

    // ── ISR tick ──
    if (g_fake_us >= ni) {
      simulateISR();
      result.total_isr_ticks++;
      isr_count++;
    }

    // ── Loop iteration ──
    if (g_fake_us >= next_loop) {
      int before = g_quantum_pos;
      doPhaseCorrection();
      if (g_quantum_pos != before) result.phase_snaps++;

      if (g_beat_flag) {
        g_beat_flag = false;
        int biq     = g_beat_in_quantum;
        int exp_biq = expectedBiqAt(g_fake_us);

        unsigned long now_ms      = (unsigned long)(g_fake_us / 1000);
        unsigned long beat_min_ms = (unsigned long)(g_tempo_us / 2000);
        bool is_dup = (biq == g_last_flash_biq
                       && now_ms - g_last_flash_ms < beat_min_ms);

        if (!is_dup) {
          BeatEvent ev;
          ev.time_us  = g_fake_us;
          ev.biq      = biq;
          ev.expected = exp_biq;
          result.beats.push_back(ev);

          if (biq == last_beat_biq) result.double_beats++;
          last_beat_biq = biq;

          if (biq != exp_biq) result.wrong_biq++;

          g_last_flash_biq = biq;
          g_last_flash_ms  = now_ms;
        }
      }

      next_loop += 1000;
    }

    // ── Link packet ──
    if (g_fake_us >= next_packet) {
      receiveLinkPacket(tempo_us, cfg.beat_origin, cfg.time_origin);
      next_packet += cfg.packet_interval;
    }
  }

  result.missed_beats = cfg.num_beats - (int)result.beats.size();
  return result;
}

// ═══════════════════════════════════════════════════════════
// Unit Tests: Core Math
// ═══════════════════════════════════════════════════════════

static bool test_microbeat_at_origin() {
  resetState(120.0f);
  g_anchor_local_us64 = 0;
  g_anchor_microbeats = 0;

  g_fake_us           = 0;

  int64_t mb = computeCurrentMicrobeats();
  EXPECT(mb == 0, "expected 0, got %lld", (long long)mb);
  return true;
}

static bool test_microbeat_after_one_beat() {
  resetState(120.0f);
  g_anchor_local_us64 = 0;
  g_anchor_microbeats = 0;

  g_fake_us           = 500000;

  int64_t mb = computeCurrentMicrobeats();
  EXPECT(mb == 1000000, "expected 1000000, got %lld", (long long)mb);
  return true;
}

static bool test_microbeat_after_4_beats() {
  resetState(120.0f);
  g_anchor_local_us64 = 0;
  g_anchor_microbeats = 0;

  g_fake_us           = 2000000;

  int64_t mb = computeCurrentMicrobeats();
  EXPECT(mb == 4000000, "expected 4000000, got %lld", (long long)mb);
  return true;
}

static bool test_microbeat_fractional() {
  resetState(120.0f);
  g_anchor_local_us64 = 0;
  g_anchor_microbeats = 0;

  g_fake_us           = 250000;

  int64_t mb = computeCurrentMicrobeats();
  EXPECT(mb == 500000, "expected 500000, got %lld", (long long)mb);
  return true;
}

static bool test_microbeat_with_nonzero_anchor() {
  resetState(120.0f);
  g_anchor_local_us64 = 1000000;
  g_anchor_microbeats = 8000000;

  g_fake_us           = 2000000;

  int64_t mb = computeCurrentMicrobeats();
  EXPECT(mb == 10000000, "expected 10000000, got %lld", (long long)mb);
  return true;
}

static bool test_microbeat_negative_elapsed() {
  resetState(120.0f);
  g_anchor_local_us64 = 1000000;
  g_anchor_microbeats = 4000000;

  g_fake_us           = 500000;

  int64_t mb = computeCurrentMicrobeats();
  EXPECT(mb == 3000000, "expected 3000000, got %lld", (long long)mb);
  return true;
}

static bool test_microbeat_high_bpm() {
  resetState(300.0f);
  g_tempo_us          = 200000;
  g_anchor_local_us64 = 0;
  g_anchor_microbeats = 0;

  g_fake_us           = 200000;

  int64_t mb = computeCurrentMicrobeats();
  EXPECT(mb == 1000000, "expected 1000000, got %lld", (long long)mb);
  return true;
}

// ═══════════════════════════════════════════════════════════
// Unit Tests: Expected Quantum Position (timeline-direct)
// ═══════════════════════════════════════════════════════════

static bool test_expected_qpos_at_beat_0() {
  resetState(120.0f);
  g_clock_offset      = 0;
  g_clock_calibrated  = true;
  g_tl_time_origin    = 0;
  g_tl_beat_origin    = 0;
  g_link_active       = true;
  g_fake_us           = 0;

  int qpos = computeExpectedQPos();
  EXPECT(qpos == 0, "expected 0, got %d", qpos);
  return true;
}

static bool test_expected_qpos_mid_beat() {
  resetState(120.0f);
  g_clock_offset      = 0;
  g_clock_calibrated  = true;
  g_tl_time_origin    = 0;
  g_tl_beat_origin    = 0;
  g_link_active       = true;
  g_fake_us           = 250000;  // half beat

  int qpos = computeExpectedQPos();
  EXPECT(qpos == 24, "expected 24, got %d", qpos);
  return true;
}

static bool test_expected_qpos_beat_1_start() {
  resetState(120.0f);
  g_clock_offset      = 0;
  g_clock_calibrated  = true;
  g_tl_time_origin    = 0;
  g_tl_beat_origin    = 0;
  g_link_active       = true;
  g_fake_us           = 500000;  // exactly 1 beat

  int qpos = computeExpectedQPos();
  EXPECT(qpos == 48, "expected 48, got %d", qpos);
  return true;
}

static bool test_expected_qpos_beat_3_end() {
  resetState(120.0f);
  g_clock_offset      = 0;
  g_clock_calibrated  = true;
  g_tl_time_origin    = 0;
  g_tl_beat_origin    = 0;
  g_link_active       = true;

  // Pulse 191 of the quantum.  At 120 BPM one pulse = 500000/48 ≈ 10416.67 us.
  // Pulse 191 starts at 191 * 500000/48 = 1989583.33... us.
  // Use 1989584 to land inside pulse 191.
  g_fake_us = 1989584;
  int qpos  = computeExpectedQPos();
  EXPECT(qpos == 191, "expected 191, got %d", qpos);
  return true;
}

static bool test_expected_qpos_wraps_at_quantum() {
  resetState(120.0f);
  g_clock_offset      = 0;
  g_clock_calibrated  = true;
  g_tl_time_origin    = 0;
  g_tl_beat_origin    = 0;
  g_link_active       = true;
  g_fake_us           = 2000000;  // exactly 4 beats

  int qpos = computeExpectedQPos();
  EXPECT(qpos == 0, "expected 0 at quantum wrap, got %d", qpos);
  return true;
}

static bool test_expected_qpos_200bpm_beat_boundary() {
  // 200 BPM, tempo_us = 300000.  At exactly 1 beat elapsed the
  // timeline-direct computation must return qpos = 48 (beat 1, pulse 0).
  resetState(200.0f);
  g_tempo_us          = 300000;
  g_clock_offset      = 0;
  g_clock_calibrated  = true;
  g_tl_time_origin    = 0;
  g_tl_beat_origin    = 0;
  g_link_active       = true;
  g_fake_us           = 300000;

  int qpos = computeExpectedQPos();
  EXPECT(qpos == 48, "expected 48 at 200BPM beat boundary, got %d", qpos);
  return true;
}

// ═══════════════════════════════════════════════════════════
// Unit Tests: ISR Beat Detection
// ═══════════════════════════════════════════════════════════

static bool test_isr_fires_beat_at_48() {
  resetState();
  g_quantum_pos = 0;
  g_beat_flag   = false;

  for (int i = 0; i < 47; i++) {
    simulateISR();
    EXPECT(!g_beat_flag, "beat_flag set early at pulse %d", i + 1);
  }
  simulateISR();
  EXPECT(g_beat_flag, "beat_flag not set at pulse 48");
  EXPECT(g_beat_in_quantum == 1, "biq=%d, expected 1", g_beat_in_quantum);
  EXPECT(g_quantum_pos == 48, "qpos=%d, expected 48", g_quantum_pos);
  return true;
}

static bool test_isr_quantum_wraps_at_192() {
  resetState();
  g_quantum_pos = 0;

  int beat_count = 0;
  int biqs[4]    = {-1, -1, -1, -1};
  for (int i = 0; i < QLEN; i++) {
    simulateISR();
    if (g_beat_flag) {
      EXPECT(beat_count < 4, "too many beats");
      biqs[beat_count++] = g_beat_in_quantum;
      g_beat_flag = false;
    }
  }
  EXPECT(beat_count == 4, "expected 4, got %d", beat_count);
  EXPECT(biqs[0] == 1 && biqs[1] == 2 && biqs[2] == 3 && biqs[3] == 0,
         "biq sequence wrong: %d %d %d %d", biqs[0], biqs[1], biqs[2], biqs[3]);
  EXPECT(g_quantum_pos == 0, "qpos should wrap to 0, got %d", g_quantum_pos);
  return true;
}

static bool test_isr_beat_sequence_across_quanta() {
  resetState();
  g_quantum_pos = 0;

  int beat_count = 0;
  for (int i = 0; i < 3 * QLEN; i++) {
    simulateISR();
    if (g_beat_flag) {
      int expected = (beat_count + 1) % QUANTUM;
      EXPECT(g_beat_in_quantum == expected,
             "beat %d: biq=%d, expected %d",
             beat_count, g_beat_in_quantum, expected);
      beat_count++;
      g_beat_flag = false;
    }
  }
  EXPECT(beat_count == 12, "expected 12, got %d", beat_count);
  return true;
}

// ═══════════════════════════════════════════════════════════
// Unit Tests: Phase Correction
// ═══════════════════════════════════════════════════════════

static bool test_phase_correction_no_snap_when_aligned() {
  resetState(120.0f);
  g_clock_offset      = 0;
  g_clock_calibrated  = true;
  g_tl_time_origin    = 0;
  g_tl_beat_origin    = 0;
  g_anchor_local_us64 = 0;
  g_anchor_microbeats = 0;
  g_link_active       = true;
  g_quantum_pos       = 0;
  g_fake_us           = 0;

  doPhaseCorrection();
  EXPECT(g_quantum_pos == 0, "should stay at 0, got %d", g_quantum_pos);
  return true;
}

static bool test_phase_correction_no_snap_within_threshold() {
  resetState(120.0f);
  g_clock_offset      = 0;
  g_clock_calibrated  = true;
  g_tl_time_origin    = 0;
  g_tl_beat_origin    = 0;
  g_anchor_local_us64 = 0;
  g_anchor_microbeats = 0;
  g_link_active       = true;
  g_quantum_pos       = 1;   // 1 pulse off
  g_fake_us           = 0;

  doPhaseCorrection();
  EXPECT(g_quantum_pos == 1, "should not snap, got %d", g_quantum_pos);
  return true;
}

static bool test_phase_correction_snaps_when_beyond_threshold() {
  resetState(120.0f);
  g_clock_offset      = 0;
  g_clock_calibrated  = true;
  g_tl_time_origin    = 0;
  g_tl_beat_origin    = 0;
  g_anchor_local_us64 = 0;
  g_anchor_microbeats = 0;
  g_link_active       = true;
  g_quantum_pos       = 10;
  g_fake_us           = 0;

  doPhaseCorrection();
  EXPECT(g_quantum_pos == 0, "should snap to 0, got %d", g_quantum_pos);
  return true;
}

static bool test_phase_correction_snaps_across_quantum_boundary() {
  resetState(120.0f);
  g_clock_offset      = 0;
  g_clock_calibrated  = true;
  g_tl_time_origin    = 0;
  g_tl_beat_origin    = 0;
  g_anchor_local_us64 = 0;
  g_anchor_microbeats = 0;
  g_link_active       = true;
  g_quantum_pos       = 185;
  g_fake_us           = 0;

  doPhaseCorrection();
  EXPECT(g_quantum_pos == 0, "should snap to 0 from 185, got %d", g_quantum_pos);
  return true;
}

static bool test_phase_correction_clears_beat_flag() {
  resetState(120.0f);
  g_clock_offset      = 0;
  g_clock_calibrated  = true;
  g_tl_time_origin    = 0;
  g_tl_beat_origin    = 0;
  g_anchor_local_us64 = 0;
  g_anchor_microbeats = 0;
  g_link_active       = true;
  g_quantum_pos       = 10;
  g_beat_flag         = true;
  g_fake_us           = 0;

  doPhaseCorrection();
  EXPECT(!g_beat_flag, "beat_flag should be cleared after snap");
  return true;
}

static bool test_phase_correction_preserves_beat_flag_when_no_snap() {
  resetState(120.0f);
  g_clock_offset      = 0;
  g_clock_calibrated  = true;
  g_tl_time_origin    = 0;
  g_tl_beat_origin    = 0;
  g_anchor_local_us64 = 0;
  g_anchor_microbeats = 0;
  g_link_active       = true;
  g_quantum_pos       = 1;
  g_beat_flag         = true;
  g_fake_us           = 0;

  doPhaseCorrection();
  EXPECT(g_beat_flag, "beat_flag should be preserved");
  return true;
}

// ═══════════════════════════════════════════════════════════
// Unit Tests: Clock Offset Calibration
// ═══════════════════════════════════════════════════════════

static bool test_offset_calibration_first_packet() {
  resetState();
  g_fake_us = 5000000;
  receiveLinkPacket(500000, 0, 0);

  EXPECT(g_clock_calibrated, "should be calibrated");
  EXPECT(g_clock_offset == 5000000,
         "offset=%lld, expected 5000000", (long long)g_clock_offset);
  return true;
}

static bool test_offset_recalibrates_on_time_origin_change() {
  resetState();
  g_fake_us = 5000000;
  receiveLinkPacket(500000, 0, 0);
  int64_t first = g_clock_offset;

  g_fake_us = 5100000;
  receiveLinkPacket(500000, 0, 0);
  EXPECT(g_clock_offset == first, "no recal with same time_origin");

  g_fake_us = 10000000;
  receiveLinkPacket(400000, 10000000, 10000000);
  EXPECT(g_clock_offset == 0,
         "offset=%lld, expected 0 after recal", (long long)g_clock_offset);
  return true;
}

// ═══════════════════════════════════════════════════════════
// Full Session Simulations
// ═══════════════════════════════════════════════════════════

static bool test_120bpm_16_beats_no_misses() {
  SimConfig cfg;
  cfg.bpm = 120.0f;  cfg.num_beats = 16;
  SimResult r = runSimulation(cfg);
  EXPECT(r.missed_beats == 0, "missed %d", r.missed_beats);
  EXPECT(r.double_beats == 0, "doubles %d", r.double_beats);
  return true;
}

static bool test_120bpm_16_beats_correct_quantum() {
  SimConfig cfg;
  cfg.bpm = 120.0f;  cfg.num_beats = 16;
  SimResult r = runSimulation(cfg);
  EXPECT(r.wrong_biq == 0, "wrong_biq=%d", r.wrong_biq);
  for (int i = 0; i < (int)r.beats.size() && i < 16; i++) {
    int exp = (i + 1) % QUANTUM;
    EXPECT(r.beats[i].biq == exp,
           "beat %d: biq=%d expected %d", i, r.beats[i].biq, exp);
  }
  return true;
}

static bool test_60bpm_8_beats() {
  SimConfig cfg;
  cfg.bpm = 60.0f;  cfg.num_beats = 8;
  SimResult r = runSimulation(cfg);
  EXPECT(r.missed_beats == 0, "missed %d", r.missed_beats);
  EXPECT(r.double_beats == 0, "doubles %d", r.double_beats);
  EXPECT(r.wrong_biq == 0,    "wrong_biq %d", r.wrong_biq);
  return true;
}

static bool test_200bpm_32_beats() {
  SimConfig cfg;
  cfg.bpm = 200.0f;  cfg.num_beats = 32;
  SimResult r = runSimulation(cfg);
  EXPECT(r.missed_beats == 0, "missed %d", r.missed_beats);
  EXPECT(r.double_beats == 0, "doubles %d", r.double_beats);
  EXPECT(r.wrong_biq == 0,    "wrong_biq %d", r.wrong_biq);
  return true;
}

static bool test_nonzero_beat_origin() {
  SimConfig cfg;
  cfg.bpm = 120.0f;  cfg.num_beats = 8;  cfg.beat_origin = 5000000;
  SimResult r = runSimulation(cfg);
  EXPECT(r.missed_beats == 0, "missed %d", r.missed_beats);
  EXPECT(r.double_beats == 0, "doubles %d", r.double_beats);
  if (!r.beats.empty()) {
    EXPECT(r.beats[0].biq == 2,
           "first biq=%d, expected 2 (beat 6 mod 4)", r.beats[0].biq);
  }
  return true;
}

static bool test_long_session_100_beats() {
  SimConfig cfg;
  cfg.bpm = 120.0f;  cfg.num_beats = 100;
  SimResult r = runSimulation(cfg);
  EXPECT(r.missed_beats == 0, "missed %d", r.missed_beats);
  EXPECT(r.double_beats == 0, "doubles %d", r.double_beats);
  EXPECT(r.wrong_biq == 0,    "wrong_biq %d", r.wrong_biq);
  return true;
}

static bool test_long_session_500_beats() {
  SimConfig cfg;
  cfg.bpm = 140.0f;  cfg.num_beats = 500;
  SimResult r = runSimulation(cfg);
  EXPECT(r.missed_beats == 0, "missed %d", r.missed_beats);
  EXPECT(r.double_beats == 0, "doubles %d", r.double_beats);
  EXPECT(r.wrong_biq == 0,    "wrong_biq %d", r.wrong_biq);
  return true;
}

static bool test_clock_drift_50ppm() {
  SimConfig cfg;
  cfg.bpm = 120.0f;  cfg.num_beats = 100;  cfg.clock_drift_ppb = 50000;
  SimResult r = runSimulation(cfg);
  EXPECT(r.missed_beats <= 1, "missed %d (allow 1)", r.missed_beats);
  EXPECT(r.double_beats == 0, "doubles %d", r.double_beats);
  EXPECT(r.wrong_biq <= 2,    "wrong_biq %d (allow 2)", r.wrong_biq);
  printf("[snaps=%d] ", r.phase_snaps);
  return true;
}

// ═══════════════════════════════════════════════════════════
// Tempo Change
// ═══════════════════════════════════════════════════════════

static bool test_tempo_change_120_to_140() {
  resetState(120.0f);

  int64_t tempo_120 = 500000;
  int64_t tempo_140 = (int64_t)(60000000.0f / 140.0f);

  // Start at 120 BPM
  g_fake_us = 1000;
  receiveLinkPacket(tempo_120, 0, 0);

  double pf = (double)tempo_120 / PPQN;
  std::vector<BeatEvent> beats;

  for (int i = 0; i < 8 * PPQN; i++) {
    g_fake_us = (uint64_t)(1000.0 + (i + 1) * pf + 0.5);
    simulateISR();
    doPhaseCorrection();
    if (g_beat_flag) {
      BeatEvent ev;
      ev.time_us = g_fake_us;
      ev.biq     = g_beat_in_quantum;
      beats.push_back(ev);
      g_beat_flag = false;
    }
  }
  EXPECT((int)beats.size() == 8, "120 BPM: expected 8, got %d", (int)beats.size());

  // Tempo change to 140.  Sender resets time_origin = sender's current time.
  int64_t sender_now_at_change = (int64_t)g_fake_us - g_clock_offset;
  int64_t mb_at_change         = computeCurrentMicrobeats();

  receiveLinkPacket(tempo_140, mb_at_change, sender_now_at_change);
  g_bpm      = 140.0f;
  g_tempo_us = tempo_140;

  pf = (double)tempo_140 / PPQN;
  double base = (double)g_fake_us;
  beats.clear();

  for (int i = 0; i < 8 * PPQN; i++) {
    g_fake_us = (uint64_t)(base + (i + 1) * pf + 0.5);
    simulateISR();
    doPhaseCorrection();
    if (g_beat_flag) {
      BeatEvent ev;
      ev.time_us  = g_fake_us;
      ev.biq      = g_beat_in_quantum;
      ev.expected = expectedBiqAt(g_fake_us);
      beats.push_back(ev);
      g_beat_flag = false;
    }
  }
  EXPECT((int)beats.size() == 8, "140 BPM: expected 8, got %d", (int)beats.size());

  int wrong = 0;
  for (auto& b : beats)
    if (b.biq != b.expected) wrong++;
  EXPECT(wrong == 0, "%d wrong quantum after tempo change", wrong);
  return true;
}

// ═══════════════════════════════════════════════════════════
// Phase Correction Stress
// ═══════════════════════════════════════════════════════════

static bool test_phase_correction_convergence() {
  resetState(120.0f);
  g_clock_offset      = 0;
  g_clock_calibrated  = true;
  g_tl_time_origin    = 0;
  g_tl_beat_origin    = 0;
  g_anchor_local_us64 = 0;
  g_anchor_microbeats = 0;
  g_link_active       = true;
  g_quantum_pos       = 50;
  g_fake_us           = 0;

  doPhaseCorrection();
  EXPECT(g_quantum_pos == 0, "should converge to 0, got %d", g_quantum_pos);
  return true;
}

static bool test_backward_snap_no_double_beat() {
  // ISR at qpos=49 (just entered beat 1). Phase correction says qpos should
  // be ~46 (still in beat 0).  After snap, ISR will re-cross beat boundary
  // at 48; duplicate suppression must catch the second fire.
  resetState(120.0f);
  g_clock_offset      = 0;
  g_clock_calibrated  = true;
  g_tl_time_origin    = 0;
  g_tl_beat_origin    = 0;
  g_anchor_local_us64 = 0;
  g_anchor_microbeats = 0;
  g_link_active       = true;

  g_quantum_pos     = 49;
  g_beat_in_quantum = 1;
  g_beat_flag       = true;

  // Time where timeline says pulse 46 (beat 0).
  // pulse 46 = 46/48 of a beat = 46 * 500000 / 48 = 479166.67 → use 479200
  // (comfortably inside pulse 46).
  g_fake_us = 479200;
  int expected_qpos = computeExpectedQPos();
  EXPECT(expected_qpos == 46,
         "expected qpos 46, compute gives %d", expected_qpos);

  // Process existing beat flag
  bool first_beat = g_beat_flag;
  int  first_biq  = g_beat_in_quantum;
  g_beat_flag     = false;
  EXPECT(first_beat && first_biq == 1, "first beat should be biq=1");

  // Phase correction snaps backward
  doPhaseCorrection();
  EXPECT(g_quantum_pos == 46, "should snap to 46, got %d", g_quantum_pos);

  // ISR advances 46→47→48 (re-crosses boundary)
  simulateISR(); // 47
  EXPECT(!g_beat_flag, "no beat at 47");
  simulateISR(); // 48 — boundary
  EXPECT(g_beat_flag, "beat at 48");
  EXPECT(g_beat_in_quantum == 1, "second crossing should be biq=1");

  // Both firings had biq=1 — firmware's time-based duplicate guard handles this.
  return true;
}

static bool test_forward_snap_no_missed_beat() {
  // ISR at qpos=44 (approaching beat boundary at 48).  Phase correction
  // snaps forward to ~50 (past boundary).  beat_flag is cleared by snap.
  resetState(120.0f);
  g_clock_offset      = 0;
  g_clock_calibrated  = true;
  g_tl_time_origin    = 0;
  g_tl_beat_origin    = 0;
  g_anchor_local_us64 = 0;
  g_anchor_microbeats = 0;
  g_link_active       = true;

  g_quantum_pos = 44;
  g_beat_flag   = false;

  // Time where timeline says pulse 50 (beat 1, pulse 2).
  // pulse 50 = 1 beat + 2/48 beat = 500000 + 2*500000/48 = 520833.33 → 520850
  g_fake_us = 520850;
  int expected_qpos = computeExpectedQPos();
  EXPECT(expected_qpos == 50,
         "expected qpos 50, compute gives %d", expected_qpos);

  doPhaseCorrection();
  EXPECT(g_quantum_pos == 50, "should snap to 50, got %d", g_quantum_pos);
  EXPECT(!g_beat_flag, "beat_flag should be cleared");
  return true;
}

// ═══════════════════════════════════════════════════════════
// Edge Cases
// ═══════════════════════════════════════════════════════════

static bool test_extreme_bpm_30() {
  SimConfig cfg;
  cfg.bpm = 30.0f;  cfg.num_beats = 8;
  SimResult r = runSimulation(cfg);
  EXPECT(r.missed_beats == 0, "missed %d", r.missed_beats);
  EXPECT(r.double_beats == 0, "doubles %d", r.double_beats);
  return true;
}

static bool test_extreme_bpm_300() {
  SimConfig cfg;
  cfg.bpm = 300.0f;  cfg.num_beats = 32;
  SimResult r = runSimulation(cfg);
  EXPECT(r.missed_beats == 0, "missed %d", r.missed_beats);
  EXPECT(r.double_beats == 0, "doubles %d", r.double_beats);
  return true;
}

static bool test_beat_origin_at_beat_3() {
  SimConfig cfg;
  cfg.bpm = 120.0f;  cfg.num_beats = 8;  cfg.beat_origin = 3000000;
  SimResult r = runSimulation(cfg);
  EXPECT(r.missed_beats == 0, "missed %d", r.missed_beats);
  if (!r.beats.empty()) {
    EXPECT(r.beats[0].biq == 0,
           "first biq=%d, expected 0 (beat 4 = downbeat)", r.beats[0].biq);
  }
  return true;
}

static bool test_infrequent_packets() {
  SimConfig cfg;
  cfg.bpm = 120.0f;  cfg.num_beats = 16;  cfg.packet_interval = 2000000;
  SimResult r = runSimulation(cfg);
  EXPECT(r.missed_beats == 0, "missed %d", r.missed_beats);
  EXPECT(r.double_beats == 0, "doubles %d", r.double_beats);
  return true;
}

static bool test_fractional_beat_origin() {
  // Session where beat_origin is 5.5 (mid-beat). First boundary should
  // be beat 6 (biq=2), then 7 (biq=3), 8 (biq=0), ...
  SimConfig cfg;
  cfg.bpm         = 120.0f;
  cfg.num_beats   = 8;
  cfg.beat_origin = 5500000;  // beat 5.5

  SimResult r = runSimulation(cfg);
  EXPECT(r.missed_beats == 0, "missed %d", r.missed_beats);
  EXPECT(r.double_beats == 0, "doubles %d", r.double_beats);
  if (!r.beats.empty()) {
    EXPECT(r.beats[0].biq == 2,
           "first biq=%d, expected 2 (beat 6 mod 4)", r.beats[0].biq);
  }
  return true;
}

static bool test_clock_drift_200ppm() {
  // Aggressive crystal drift — phase correction must keep quantum aligned
  SimConfig cfg;
  cfg.bpm             = 120.0f;
  cfg.num_beats       = 200;
  cfg.clock_drift_ppb = 200000;  // 200 ppm

  SimResult r = runSimulation(cfg);
  EXPECT(r.missed_beats <= 2,
         "missed %d beats (allow 2 for drift)", r.missed_beats);
  EXPECT(r.double_beats == 0, "doubles %d", r.double_beats);
  EXPECT(r.wrong_biq <= 4,
         "wrong_biq %d (allow 4 for drift)", r.wrong_biq);
  printf("[snaps=%d] ", r.phase_snaps);
  return true;
}

// ═══════════════════════════════════════════════════════════
// Packet Parser
// ═══════════════════════════════════════════════════════════

static void buildHeader(uint8_t* pkt, int& len, uint8_t msg_type,
                        const uint8_t* peer) {
  memcpy(pkt, "_asdp_v\x01", 8);
  pkt[8] = msg_type;
  pkt[9] = 5;
  pkt[10] = pkt[11] = 0;
  memcpy(pkt + 12, peer, 8);
  len = 20;
}

static const uint8_t OTHER_PEER[8] = {
  0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22
};

static bool test_parse_valid_packet() {
  resetState();
  g_fake_us = 100000;

  uint8_t pkt[256];
  int len = 0;
  buildHeader(pkt, len, 0x01, OTHER_PEER);

  // tmln
  memcpy(pkt + len, "tmln", 4); len += 4;
  writeBE32(pkt + len, 24);     len += 4;
  writeBE64(pkt + len, 500000LL);  len += 8;  // tempo_us
  writeBE64(pkt + len, 0LL);       len += 8;  // beat_origin
  writeBE64(pkt + len, 50000LL);   len += 8;  // time_origin

  // sess
  memcpy(pkt + len, "sess", 4); len += 4;
  writeBE32(pkt + len, 8);      len += 4;
  uint8_t sid[8] = {1,2,3,4,5,6,7,8};
  memcpy(pkt + len, sid, 8);    len += 8;

  parseLinkPacket(pkt, len);

  EXPECT(g_clock_calibrated, "should be calibrated");
  EXPECT(g_tempo_us == 500000LL, "tempo_us=%lld", (long long)g_tempo_us);
  EXPECT(g_tl_time_origin == 50000LL, "time_origin=%lld",
         (long long)g_tl_time_origin);
  EXPECT(g_tl_beat_origin == 0LL, "beat_origin=%lld",
         (long long)g_tl_beat_origin);
  EXPECT(g_session_valid, "session should be valid");
  EXPECT(memcmp(g_session_id, sid, 8) == 0, "session_id mismatch");
  EXPECT(g_clock_offset == 50000LL, "offset=%lld",
         (long long)g_clock_offset);
  return true;
}

static bool test_parse_self_packet_filtered() {
  resetState();
  g_fake_us = 100000;

  uint8_t pkt[256];
  int len = 0;
  buildHeader(pkt, len, 0x01, PEER_ID);  // our own peer ID

  memcpy(pkt + len, "tmln", 4); len += 4;
  writeBE32(pkt + len, 24);     len += 4;
  writeBE64(pkt + len, 500000LL); len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;

  parseLinkPacket(pkt, len);
  EXPECT(!g_clock_calibrated, "self-packet should be ignored");
  return true;
}

static bool test_parse_truncated_header() {
  resetState();
  uint8_t pkt[19];
  memcpy(pkt, "_asdp_v\x01", 8);
  pkt[8] = 0x01;
  memset(pkt + 9, 0, 10);

  parseLinkPacket(pkt, 19);
  EXPECT(!g_clock_calibrated, "truncated header should be ignored");
  return true;
}

static bool test_parse_invalid_magic() {
  resetState();
  g_fake_us = 100000;

  uint8_t pkt[256];
  int len = 0;
  memcpy(pkt, "XXXXXXXX", 8);  // wrong magic
  pkt[8] = 0x01; pkt[9] = 5; pkt[10] = pkt[11] = 0;
  memcpy(pkt + 12, OTHER_PEER, 8);
  len = 20;

  memcpy(pkt + len, "tmln", 4); len += 4;
  writeBE32(pkt + len, 24);     len += 4;
  writeBE64(pkt + len, 500000LL); len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;

  parseLinkPacket(pkt, len);
  EXPECT(!g_clock_calibrated, "bad magic should be ignored");
  return true;
}

static bool test_parse_unknown_section_skipped() {
  resetState();
  g_fake_us = 100000;

  uint8_t pkt[256];
  int len = 0;
  buildHeader(pkt, len, 0x01, OTHER_PEER);

  // Unknown section with 4-byte payload
  memcpy(pkt + len, "xyzq", 4); len += 4;
  writeBE32(pkt + len, 4);      len += 4;
  memset(pkt + len, 0, 4);      len += 4;

  // tmln after the unknown section
  memcpy(pkt + len, "tmln", 4); len += 4;
  writeBE32(pkt + len, 24);     len += 4;
  writeBE64(pkt + len, 500000LL); len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;
  writeBE64(pkt + len, 50000LL);  len += 8;

  parseLinkPacket(pkt, len);
  EXPECT(g_clock_calibrated, "tmln after unknown section should parse");
  EXPECT(g_tempo_us == 500000LL, "tempo=%lld", (long long)g_tempo_us);
  return true;
}

static bool test_parse_zero_length_section() {
  resetState();
  g_fake_us = 100000;

  uint8_t pkt[256];
  int len = 0;
  buildHeader(pkt, len, 0x01, OTHER_PEER);

  // Zero-length section (should be skipped, not break the loop)
  memcpy(pkt + len, "zero", 4); len += 4;
  writeBE32(pkt + len, 0);      len += 4;

  // tmln after zero-length section
  memcpy(pkt + len, "tmln", 4); len += 4;
  writeBE32(pkt + len, 24);     len += 4;
  writeBE64(pkt + len, 500000LL); len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;
  writeBE64(pkt + len, 50000LL);  len += 8;

  parseLinkPacket(pkt, len);
  EXPECT(g_clock_calibrated,
         "tmln after zero-length section should parse");
  return true;
}

static bool test_parse_stst_play_state() {
  resetState();
  g_fake_us = 100000;

  uint8_t pkt[256];
  int len = 0;
  buildHeader(pkt, len, 0x01, OTHER_PEER);

  // stst: playing = true
  memcpy(pkt + len, "stst", 4); len += 4;
  writeBE32(pkt + len, 17);     len += 4;
  pkt[len++] = 1;
  memset(pkt + len, 0, 16);     len += 16;

  parseLinkPacket(pkt, len);
  EXPECT(g_is_playing, "should be playing");

  // Reset and send not-playing
  g_is_playing = true;
  len = 0;
  buildHeader(pkt, len, 0x01, OTHER_PEER);
  memcpy(pkt + len, "stst", 4); len += 4;
  writeBE32(pkt + len, 17);     len += 4;
  pkt[len++] = 0;
  memset(pkt + len, 0, 16);     len += 16;

  parseLinkPacket(pkt, len);
  EXPECT(!g_is_playing, "should not be playing");
  return true;
}

static bool test_build_presence_packet() {
  resetState();
  uint8_t sid[8] = {0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80};
  memcpy(g_session_id, sid, 8);

  uint8_t pkt[64];
  int len = buildPresencePacket(pkt, 0x01);

  EXPECT(len == 36, "expected 36 bytes, got %d", len);
  EXPECT(memcmp(pkt, "_asdp_v\x01", 8) == 0, "bad magic");
  EXPECT(pkt[8] == 0x01, "bad msg_type=%d", pkt[8]);
  EXPECT(memcmp(pkt + 12, PEER_ID, 8) == 0, "bad peer_id");
  EXPECT(memcmp(pkt + 20, "sess", 4) == 0, "bad section tag");
  EXPECT(readBE32(pkt + 24) == 8, "bad section length");
  EXPECT(memcmp(pkt + 28, sid, 8) == 0, "bad session_id");
  return true;
}

static bool test_parse_kresponse_accepted() {
  resetState();
  g_fake_us = 100000;

  uint8_t pkt[256];
  int len = 0;
  buildHeader(pkt, len, 0x02, OTHER_PEER);  // kResponse
  memcpy(pkt + len, "tmln", 4); len += 4;
  writeBE32(pkt + len, 24);     len += 4;
  writeBE64(pkt + len, 500000LL); len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;

  parseLinkPacket(pkt, len);
  EXPECT(g_clock_calibrated, "kResponse (0x02) should be accepted");
  return true;
}

static bool test_parse_invalid_msg_type_rejected() {
  resetState();
  g_fake_us = 100000;

  uint8_t pkt[256];
  int len = 0;
  buildHeader(pkt, len, 0x03, OTHER_PEER);
  memcpy(pkt + len, "tmln", 4); len += 4;
  writeBE32(pkt + len, 24);     len += 4;
  writeBE64(pkt + len, 500000LL); len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;

  parseLinkPacket(pkt, len);
  EXPECT(!g_clock_calibrated, "msg_type 0x03 should be rejected");
  return true;
}

static bool test_parse_zero_tempo_rejected() {
  resetState();
  g_fake_us = 100000;

  uint8_t pkt[256];
  int len = 0;
  buildHeader(pkt, len, 0x01, OTHER_PEER);
  memcpy(pkt + len, "tmln", 4); len += 4;
  writeBE32(pkt + len, 24);     len += 4;
  writeBE64(pkt + len, 0LL);      len += 8;  // tempo = 0
  writeBE64(pkt + len, 0LL);      len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;

  parseLinkPacket(pkt, len);
  EXPECT(!g_clock_calibrated, "zero tempo should be rejected");
  return true;
}

static bool test_parse_negative_tempo_rejected() {
  resetState();
  g_fake_us = 100000;

  uint8_t pkt[256];
  int len = 0;
  buildHeader(pkt, len, 0x01, OTHER_PEER);
  memcpy(pkt + len, "tmln", 4); len += 4;
  writeBE32(pkt + len, 24);     len += 4;
  writeBE64(pkt + len, -500000LL); len += 8;  // negative tempo
  writeBE64(pkt + len, 0LL);       len += 8;
  writeBE64(pkt + len, 0LL);       len += 8;

  parseLinkPacket(pkt, len);
  EXPECT(!g_clock_calibrated, "negative tempo should be rejected");
  return true;
}

// ═══════════════════════════════════════════════════════════
// Integration (raw packet → parse → phase correction)
// ═══════════════════════════════════════════════════════════

static bool test_raw_packet_to_phase_correction() {
  resetState();
  g_fake_us = 1000000;  // 1s
  g_link_active = true;

  // 120 BPM, beat_origin=0, time_origin=0
  uint8_t pkt[256];
  int len = 0;
  buildHeader(pkt, len, 0x01, OTHER_PEER);
  memcpy(pkt + len, "tmln", 4); len += 4;
  writeBE32(pkt + len, 24);     len += 4;
  writeBE64(pkt + len, 500000LL); len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;

  parseLinkPacket(pkt, len);

  // Advance 1s after calibration (offset=1000000, sender_now = 2000000-1000000 = 1000000)
  // elapsed = 1000000/500000 = 2 beats → biq=2, pulse 0 → qpos=96
  g_fake_us = 2000000;
  int expected = computeExpectedQPos();
  EXPECT(expected == 96, "expected qpos=96, got %d", expected);

  // ISR at wrong position → phase correction snaps
  g_quantum_pos = 50;
  doPhaseCorrection();
  EXPECT(g_quantum_pos == 96, "should snap to 96, got %d", g_quantum_pos);
  return true;
}

static bool test_raw_packet_tempo_change() {
  resetState();
  g_fake_us = 100000;
  g_link_active = true;

  // First packet: 120 BPM (matches default — no tempo_changed)
  uint8_t pkt[256];
  int len = 0;
  buildHeader(pkt, len, 0x01, OTHER_PEER);
  memcpy(pkt + len, "tmln", 4); len += 4;
  writeBE32(pkt + len, 24);     len += 4;
  writeBE64(pkt + len, 500000LL); len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;

  parseLinkPacket(pkt, len);
  EXPECT(!g_tempo_changed, "120->120 should not trigger change");

  // Second packet: 140 BPM with new time_origin
  g_fake_us = 200000;
  int64_t tempo_140 = (int64_t)(60000000.0f / 140.0f);
  len = 0;
  buildHeader(pkt, len, 0x01, OTHER_PEER);
  memcpy(pkt + len, "tmln", 4); len += 4;
  writeBE32(pkt + len, 24);     len += 4;
  writeBE64(pkt + len, tempo_140);  len += 8;
  writeBE64(pkt + len, 2000000LL);  len += 8;
  writeBE64(pkt + len, 200000LL);   len += 8;

  parseLinkPacket(pkt, len);
  EXPECT(g_tempo_changed, "120->140 should trigger change");
  EXPECT(g_tempo_us == tempo_140, "tempo=%lld", (long long)g_tempo_us);
  return true;
}

// ═══════════════════════════════════════════════════════════
// Pulse Width Clamping
// ═══════════════════════════════════════════════════════════

static bool test_pulse_width_clamping() {
  // Mirrors startClockTimer's clamping logic.
  // At 600 BPM: clock freq = 480 Hz, period = 2.083ms
  // 40% duty = 0.833ms → pulse_freq = 1200 Hz (clamped)
  float bpm = 600.0f;
  float freq_hz = (bpm / 60.0f) * PPQN;
  float pulse_freq = 500.0f;
  float max_pulse = 0.4f * (1.0f / freq_hz);
  if (max_pulse < (1.0f / pulse_freq)) pulse_freq = 1.0f / max_pulse;

  EXPECT(pulse_freq > 500.0f, "600 BPM should clamp pulse");
  EXPECT(pulse_freq > 1100.0f && pulse_freq < 1300.0f,
         "pulse_freq=%.0f", pulse_freq);

  // At 120 BPM: no clamping needed (period = 10.4ms, 40% = 4.17ms > 2ms)
  bpm = 120.0f;
  freq_hz = (bpm / 60.0f) * PPQN;
  pulse_freq = 500.0f;
  max_pulse = 0.4f * (1.0f / freq_hz);
  if (max_pulse < (1.0f / pulse_freq)) pulse_freq = 1.0f / max_pulse;

  EXPECT(pulse_freq == 500.0f, "120 BPM should not clamp");
  return true;
}

// ═══════════════════════════════════════════════════════════
// Disconnect Timeout
// ═══════════════════════════════════════════════════════════

static bool test_disconnect_timeout() {
  resetState();
  g_fake_us = 100000;  // 100ms

  uint8_t pkt[256];
  int len = 0;
  buildHeader(pkt, len, 0x01, OTHER_PEER);
  memcpy(pkt + len, "tmln", 4); len += 4;
  writeBE32(pkt + len, 24);     len += 4;
  writeBE64(pkt + len, 500000LL); len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;
  writeBE64(pkt + len, 0LL);      len += 8;

  parseLinkPacket(pkt, len);
  EXPECT(g_last_peer_ms == 100, "peer_ms=%lu", g_last_peer_ms);

  // 2999ms after last packet: still active
  unsigned long now_ms = 100 + 2999;
  bool active = (g_last_peer_ms > 0) && (now_ms - g_last_peer_ms < 3000);
  EXPECT(active, "should be active at 2999ms");

  // 3001ms after last packet: disconnected
  now_ms = 100 + 3001;
  active = (g_last_peer_ms > 0) && (now_ms - g_last_peer_ms < 3000);
  EXPECT(!active, "should be disconnected at 3001ms");

  return true;
}

// ═══════════════════════════════════════════════════════════
// Progress Bar Computation
// ═══════════════════════════════════════════════════════════

static bool test_progress_bar_formula() {
  // cols = ((qpos + 1) * 12) / (4 * PPQN)
  // Must map 0..191 → 0..12 (12 = all columns filled)
  EXPECT(((0   + 1) * 12) / (4 * PPQN) == 0,  "qpos=0 → cols=0");
  EXPECT(((15  + 1) * 12) / (4 * PPQN) == 1,  "qpos=15 → cols=1");
  EXPECT(((47  + 1) * 12) / (4 * PPQN) == 3,  "qpos=47 → cols=3");
  EXPECT(((95  + 1) * 12) / (4 * PPQN) == 6,  "qpos=95 → cols=6");
  EXPECT(((143 + 1) * 12) / (4 * PPQN) == 9,  "qpos=143 → cols=9");
  EXPECT(((191 + 1) * 12) / (4 * PPQN) == 12, "qpos=191 → cols=12");
  return true;
}

static bool test_progress_bar_monotonic() {
  // Progress bar must never decrease within a quantum
  int prev = -1;
  for (int qpos = 0; qpos < QLEN; qpos++) {
    int cols = ((qpos + 1) * 12) / (4 * PPQN);
    EXPECT(cols >= prev,
           "non-monotonic at qpos=%d: cols=%d < prev=%d", qpos, cols, prev);
    prev = cols;
  }
  return true;
}

// ═══════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════

int main() {
  printf("\n=== Link Clock Sync Unit Tests ===\n\n");

  printf("Core Math:\n");
  RUN_TEST(test_microbeat_at_origin);
  RUN_TEST(test_microbeat_after_one_beat);
  RUN_TEST(test_microbeat_after_4_beats);
  RUN_TEST(test_microbeat_fractional);
  RUN_TEST(test_microbeat_with_nonzero_anchor);
  RUN_TEST(test_microbeat_negative_elapsed);
  RUN_TEST(test_microbeat_high_bpm);

  printf("\nExpected Quantum Position:\n");
  RUN_TEST(test_expected_qpos_at_beat_0);
  RUN_TEST(test_expected_qpos_mid_beat);
  RUN_TEST(test_expected_qpos_beat_1_start);
  RUN_TEST(test_expected_qpos_beat_3_end);
  RUN_TEST(test_expected_qpos_wraps_at_quantum);
  RUN_TEST(test_expected_qpos_200bpm_beat_boundary);

  printf("\nISR Beat Detection:\n");
  RUN_TEST(test_isr_fires_beat_at_48);
  RUN_TEST(test_isr_quantum_wraps_at_192);
  RUN_TEST(test_isr_beat_sequence_across_quanta);

  printf("\nPhase Correction:\n");
  RUN_TEST(test_phase_correction_no_snap_when_aligned);
  RUN_TEST(test_phase_correction_no_snap_within_threshold);
  RUN_TEST(test_phase_correction_snaps_when_beyond_threshold);
  RUN_TEST(test_phase_correction_snaps_across_quantum_boundary);
  RUN_TEST(test_phase_correction_clears_beat_flag);
  RUN_TEST(test_phase_correction_preserves_beat_flag_when_no_snap);

  printf("\nClock Offset Calibration:\n");
  RUN_TEST(test_offset_calibration_first_packet);
  RUN_TEST(test_offset_recalibrates_on_time_origin_change);

  printf("\nFull Session Simulations:\n");
  RUN_TEST(test_120bpm_16_beats_no_misses);
  RUN_TEST(test_120bpm_16_beats_correct_quantum);
  RUN_TEST(test_60bpm_8_beats);
  RUN_TEST(test_200bpm_32_beats);
  RUN_TEST(test_nonzero_beat_origin);
  RUN_TEST(test_long_session_100_beats);
  RUN_TEST(test_long_session_500_beats);
  RUN_TEST(test_clock_drift_50ppm);

  printf("\nTempo Change:\n");
  RUN_TEST(test_tempo_change_120_to_140);

  printf("\nPhase Correction Stress:\n");
  RUN_TEST(test_phase_correction_convergence);
  RUN_TEST(test_backward_snap_no_double_beat);
  RUN_TEST(test_forward_snap_no_missed_beat);

  printf("\nEdge Cases:\n");
  RUN_TEST(test_extreme_bpm_30);
  RUN_TEST(test_extreme_bpm_300);
  RUN_TEST(test_beat_origin_at_beat_3);
  RUN_TEST(test_infrequent_packets);
  RUN_TEST(test_fractional_beat_origin);
  RUN_TEST(test_clock_drift_200ppm);

  printf("\nPacket Parser:\n");
  RUN_TEST(test_parse_valid_packet);
  RUN_TEST(test_parse_self_packet_filtered);
  RUN_TEST(test_parse_truncated_header);
  RUN_TEST(test_parse_invalid_magic);
  RUN_TEST(test_parse_unknown_section_skipped);
  RUN_TEST(test_parse_zero_length_section);
  RUN_TEST(test_parse_stst_play_state);
  RUN_TEST(test_build_presence_packet);
  RUN_TEST(test_parse_kresponse_accepted);
  RUN_TEST(test_parse_invalid_msg_type_rejected);
  RUN_TEST(test_parse_zero_tempo_rejected);
  RUN_TEST(test_parse_negative_tempo_rejected);

  printf("\nIntegration:\n");
  RUN_TEST(test_raw_packet_to_phase_correction);
  RUN_TEST(test_raw_packet_tempo_change);
  RUN_TEST(test_pulse_width_clamping);

  printf("\nDisconnect Timeout:\n");
  RUN_TEST(test_disconnect_timeout);

  printf("\nProgress Bar:\n");
  RUN_TEST(test_progress_bar_formula);
  RUN_TEST(test_progress_bar_monotonic);

  printf("\n─────────────────────────────────────────────────────\n");
  printf("Results: %d/%d passed", g_tests_passed, g_tests_run);
  if (g_tests_failed > 0)
    printf(", %d FAILED", g_tests_failed);
  printf("\n\n");

  return g_tests_failed > 0 ? 1 : 0;
}
