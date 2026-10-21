// RC car actuator bridge: USB serial commands -> hardware PWM for
// DS3225 steering servo + THW-1060 brushed ESC.
//
// Architecture
//   loop()        : USB serial parser + telemetry (comms only)
//   controlTask() : 100 Hz, owns the actuators: failsafe, slew, reverse logic
//   LEDC hardware : generates the 50 Hz pulse trains with zero CPU load
// The comms layer is the only part that changes when this becomes a
// micro-ROS node; controlTask() and the PWM layer stay as they are.

#include <Arduino.h>
#include <Preferences.h>
#include <driver/ledc.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <sys/time.h>

#include "pins.h"

static const char *FW_VERSION = "0.1.0";

// ---------------------------------------------------------------------------
// Fixed limits from the datasheets (docs/datasheets/*.md). Never exceeded, even in RAW.
// ---------------------------------------------------------------------------
// DS3225: 500..2500 us, 1500 us neutral, 50..330 Hz frame, 3 us deadband
constexpr uint16_t SERVO_ABS_MIN_US = 500;
constexpr uint16_t SERVO_ABS_MAX_US = 2500;
constexpr uint16_t SERVO_MAX_HZ     = 330;
// THW-1060: standard RC throttle. Hard window; tuned endpoints live in Config.
constexpr uint16_t ESC_ABS_MIN_US   = 900;
constexpr uint16_t ESC_ABS_MAX_US   = 2100;
constexpr uint16_t ESC_MAX_HZ       = 400;   // above 50 Hz: verify ESC first
constexpr uint16_t BOOT_NEUTRAL_US  = 1500;

constexpr uint32_t PWM_RES_BITS = 14;   // at 50 Hz: 20000 us / 16384 = 1.22 us per tick

constexpr ledc_mode_t      LEDC_MODE      = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t     STEER_TIMER    = LEDC_TIMER_0;
constexpr ledc_timer_t     THROTTLE_TIMER = LEDC_TIMER_1;
constexpr ledc_channel_t   STEER_CH       = LEDC_CHANNEL_0;
constexpr ledc_channel_t   THROTTLE_CH    = LEDC_CHANNEL_1;

// ---------------------------------------------------------------------------
// Tunable config, persisted in NVS flash
// ---------------------------------------------------------------------------
constexpr uint32_t CFG_MAGIC = 0xC0FFEE02;  // bump when Config layout changes

struct Config {
  uint32_t magic        = CFG_MAGIC;
  uint16_t steerCenterUs = 1500;
  uint16_t steerLeftUs   = 1800;   // pulse at full LEFT lock (may be < center)
  uint16_t steerRightUs  = 1200;   // pulse at full RIGHT lock
  float    maxSteerDeg   = 25.0f;  // road-wheel angle at full lock
  float    headingGain   = 1.0f;   // steering deg per heading deg
  float    speedLimit    = 0.30f;  // speed 1.0 -> this fraction of full throttle
  uint16_t deadbandUs    = 0;      // ESC dead zone skipped around neutral
  float    accelPerSec   = 2.0f;   // max change of speed command per second
  uint16_t timeoutMs     = 300;    // no command for this long -> failsafe
  uint8_t  doubleTapRev  = 1;      // ESC needs brake-neutral-reverse sequence
  uint16_t brakeMs       = 150;    // double-tap reverse: brake pulse length
  uint16_t gapMs         = 150;    // double-tap reverse: neutral gap length
  float    brakeLevel    = 0.5f;   // double-tap reverse: brake strength
  uint16_t escMinUs      = 1000;   // full reverse
  uint16_t escNeutralUs  = 1500;
  uint16_t escMaxUs      = 2000;   // full forward
  uint16_t servoHz       = 50;     // DS3225 accepts 50..330
  uint16_t escHz         = 50;     // RC standard; raise only if the ESC accepts it
  uint16_t controlHz     = 100;    // control loop rate
};

struct Command {
  float    headingDeg = 0.0f;  // + = left (ROS REP-103), relative to car's nose
  float    speed      = 0.0f;  // -1..1 normalized
  uint64_t stampMs    = 0;
};

struct Output {
  uint16_t steerUs    = 1500;
  uint16_t throttleUs = BOOT_NEUTRAL_US;
  float    steerDeg   = 0.0f;
};

static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static Config  g_cfg;
static Command g_cmd;
static Output  g_out;
static bool    g_armed       = false;
static int     g_rawSteerUs  = -1;     // >0: calibration override
static bool    g_failsafeEvt = false;  // set by control task, reported by loop
static uint32_t g_telemetryHz = 0;
static bool    g_timeSynced  = false;

static Preferences g_prefs;

static uint64_t nowMs() { return esp_timer_get_time() / 1000ULL; }

template <typename T>
static T clampv(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ---------------------------------------------------------------------------
// PWM layer (LEDC). Once a duty is set the hardware keeps emitting pulses.
// ---------------------------------------------------------------------------
static uint32_t usToDuty(uint32_t us, uint32_t hz) {
  const uint32_t periodUs = 1000000UL / hz;
  return (uint32_t)(((uint64_t)us << PWM_RES_BITS) / periodUs);
}

static void pwmInit(int pin, ledc_timer_t timer, ledc_channel_t ch,
                    uint32_t hz, uint16_t initialUs) {
  ledc_timer_config_t t = {};
  t.speed_mode      = LEDC_MODE;
  t.duty_resolution = (ledc_timer_bit_t)PWM_RES_BITS;
  t.timer_num       = timer;
  t.freq_hz         = hz;
  t.clk_cfg         = LEDC_AUTO_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&t));

  ledc_channel_config_t c = {};
  c.gpio_num   = pin;
  c.speed_mode = LEDC_MODE;
  c.channel    = ch;
  c.timer_sel  = timer;
  c.duty       = usToDuty(initialUs, hz);
  c.hpoint     = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&c));
}

static void pwmWriteUs(ledc_channel_t ch, uint32_t hz, uint16_t us) {
  ledc_set_duty(LEDC_MODE, ch, usToDuty(us, hz));
  ledc_update_duty(LEDC_MODE, ch);
}

// ---------------------------------------------------------------------------
// Mapping
// ---------------------------------------------------------------------------
// heading (deg) -> road-wheel steering angle (deg), open loop for now.
// With an IMU this becomes a closed heading loop; the interface stays.
static float headingToSteerDeg(float headingDeg, const Config &c) {
  return clampv(headingDeg * c.headingGain, -c.maxSteerDeg, c.maxSteerDeg);
}

// steering angle -> pulse, piecewise linear so an asymmetric linkage works.
static uint16_t steerDegToUs(float deg, const Config &c) {
  float frac = deg / c.maxSteerDeg;  // -1 (right) .. +1 (left)
  float us = frac >= 0 ? c.steerCenterUs + frac * (c.steerLeftUs - c.steerCenterUs)
                       : c.steerCenterUs + -frac * (c.steerRightUs - c.steerCenterUs);
  uint16_t lo = min(c.steerLeftUs, c.steerRightUs);
  uint16_t hi = max(c.steerLeftUs, c.steerRightUs);
  return clampv<uint16_t>((uint16_t)lroundf(us), lo, hi);
}

// throttle fraction (-1..1, already speed-limited) -> ESC pulse
static uint16_t throttleToUs(float u, const Config &c) {
  if (fabsf(u) < 0.001f) return c.escNeutralUs;
  float span = (u > 0 ? c.escMaxUs - c.escNeutralUs : c.escNeutralUs - c.escMinUs) - c.deadbandUs;
  float us = c.escNeutralUs + (u > 0 ? 1 : -1) * (c.deadbandUs + fabsf(u) * span);
  return clampv<uint16_t>((uint16_t)lroundf(us), c.escMinUs, c.escMaxUs);
}

// ---------------------------------------------------------------------------
// Control task: the only writer of the PWM outputs
// ---------------------------------------------------------------------------
enum class RevState { Forward, Brake, Gap, Reverse };

static void controlTask(void *) {
  esp_task_wdt_add(nullptr);  // hang -> chip reset -> no pulses -> ESC cuts in 0.1 s
  TickType_t last = xTaskGetTickCount();
  float speedOut = 0.0f;
  RevState rs = RevState::Forward;
  uint64_t rsSince = 0;
  uint16_t servoHz = 50, escHz = 50;  // what the LEDC timers currently run at
  uint16_t controlHz = 100;

  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / controlHz));
    esp_task_wdt_reset();

    Config cfg; Command cmd; bool armed; int raw;
    portENTER_CRITICAL(&g_mux);
    cfg = g_cfg; cmd = g_cmd; armed = g_armed; raw = g_rawSteerUs;
    portEXIT_CRITICAL(&g_mux);

    // Runtime frame-rate changes (tuning.toml -> SET servo_hz / esc_hz)
    if (cfg.servoHz != servoHz) { ledc_set_freq(LEDC_MODE, STEER_TIMER, cfg.servoHz); servoHz = cfg.servoHz; }
    if (cfg.escHz != escHz)     { ledc_set_freq(LEDC_MODE, THROTTLE_TIMER, cfg.escHz); escHz = cfg.escHz; }
    controlHz = cfg.controlHz;

    const uint64_t now = nowMs();
    const bool stale = (now - cmd.stampMs) > cfg.timeoutMs;
    if (armed && stale) {
      armed = false;
      portENTER_CRITICAL(&g_mux);
      g_armed = false; g_failsafeEvt = true;
      portEXIT_CRITICAL(&g_mux);
    }

    // Speed: slew-limited toward target; disarm snaps to neutral.
    const float target = armed ? clampv(cmd.speed, -1.0f, 1.0f) : 0.0f;
    const float step = cfg.accelPerSec / controlHz;
    speedOut = armed ? speedOut + clampv(target - speedOut, -step, step) : 0.0f;
    float u = speedOut * cfg.speedLimit;

    // Many brushed ESCs (Fwd/Brake/Rev mode) treat the first reverse pulse as
    // BRAKE and only reverse after a return to neutral. Do that sequence here.
    if (cfg.doubleTapRev) {
      switch (rs) {
        case RevState::Forward:
          if (u < 0) { rs = RevState::Brake; rsSince = now; u = -cfg.brakeLevel; }
          break;
        case RevState::Brake:
          if (u >= 0) rs = RevState::Forward;
          else if (now - rsSince >= cfg.brakeMs) { rs = RevState::Gap; rsSince = now; u = 0; }
          else u = -cfg.brakeLevel;
          break;
        case RevState::Gap:
          if (u >= 0) rs = RevState::Forward;
          else if (now - rsSince >= cfg.gapMs) rs = RevState::Reverse;
          else u = 0;
          break;
        case RevState::Reverse:
          if (u > 0) rs = RevState::Forward;
          break;
      }
    }

    // Steering follows commands even when disarmed (safe steering tuning).
    const float steerDeg = headingToSteerDeg(cmd.headingDeg, cfg);
    const uint16_t steerUs = raw > 0
        ? clampv<uint16_t>((uint16_t)raw, SERVO_ABS_MIN_US, SERVO_ABS_MAX_US)
        : steerDegToUs(steerDeg, cfg);
    const uint16_t thrUs = throttleToUs(u, cfg);

    pwmWriteUs(STEER_CH, servoHz, steerUs);
    pwmWriteUs(THROTTLE_CH, escHz, thrUs);

    portENTER_CRITICAL(&g_mux);
    g_out.steerUs = steerUs; g_out.throttleUs = thrUs; g_out.steerDeg = steerDeg;
    portEXIT_CRITICAL(&g_mux);
  }
}

// ---------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------
static void loadConfig() {
  Config c;
  g_prefs.begin("rccar", true);
  size_t n = g_prefs.getBytes("cfg", &c, sizeof(c));
  g_prefs.end();
  if (n == sizeof(c) && c.magic == CFG_MAGIC) g_cfg = c;
}

static bool saveConfig() {
  Config c;
  portENTER_CRITICAL(&g_mux); c = g_cfg; portEXIT_CRITICAL(&g_mux);
  g_prefs.begin("rccar", false);
  size_t n = g_prefs.putBytes("cfg", &c, sizeof(c));
  g_prefs.end();
  return n == sizeof(c);
}

// ---------------------------------------------------------------------------
// Serial protocol (one ASCII command per line, see docs)
// ---------------------------------------------------------------------------
static void printConfig() {
  Config c;
  portENTER_CRITICAL(&g_mux); c = g_cfg; portEXIT_CRITICAL(&g_mux);
  Serial.printf("CFG center=%u left=%u right=%u maxdeg=%.1f gain=%.2f limit=%.2f "
                "deadband=%u accel=%.2f timeout=%u revmode=%u brake_ms=%u gap_ms=%u "
                "brake_level=%.2f esc_min=%u esc_neutral=%u esc_max=%u servo_hz=%u "
                "esc_hz=%u control_hz=%u\n",
                c.steerCenterUs, c.steerLeftUs, c.steerRightUs, c.maxSteerDeg,
                c.headingGain, c.speedLimit, c.deadbandUs, c.accelPerSec,
                c.timeoutMs, c.doubleTapRev, c.brakeMs, c.gapMs, c.brakeLevel,
                c.escMinUs, c.escNeutralUs, c.escMaxUs, c.servoHz, c.escHz, c.controlHz);
}

static void printTelemetry() {
  Command cmd; Output o; bool armed;
  portENTER_CRITICAL(&g_mux); cmd = g_cmd; o = g_out; armed = g_armed; portEXIT_CRITICAL(&g_mux);
  struct timeval tv; gettimeofday(&tv, nullptr);
  uint64_t unixMs = g_timeSynced ? (uint64_t)tv.tv_sec * 1000ULL + tv.tv_usec / 1000 : 0;
  Serial.printf("TEL t=%llu unix=%llu hdg=%.1f spd=%.2f steer_deg=%.1f steer_us=%u thr_us=%u armed=%d\n",
                nowMs(), unixMs, cmd.headingDeg, cmd.speed, o.steerDeg,
                o.steerUs, o.throttleUs, armed);
}

static bool setKey(Config &c, const char *key, float v) {
  if      (!strcasecmp(key, "maxdeg"))   c.maxSteerDeg = clampv(v, 1.0f, 60.0f);
  else if (!strcasecmp(key, "gain"))     c.headingGain = clampv(v, 0.0f, 10.0f);
  else if (!strcasecmp(key, "limit"))    c.speedLimit  = clampv(v, 0.0f, 1.0f);
  else if (!strcasecmp(key, "deadband")) c.deadbandUs  = (uint16_t)clampv(v, 0.0f, 200.0f);
  else if (!strcasecmp(key, "accel"))    c.accelPerSec = clampv(v, 0.1f, 100.0f);
  else if (!strcasecmp(key, "timeout"))  c.timeoutMs   = (uint16_t)clampv(v, 50.0f, 2000.0f);
  else if (!strcasecmp(key, "revmode"))  c.doubleTapRev = v != 0;
  else if (!strcasecmp(key, "brake_ms")) c.brakeMs     = (uint16_t)clampv(v, 0.0f, 1000.0f);
  else if (!strcasecmp(key, "gap_ms"))   c.gapMs       = (uint16_t)clampv(v, 0.0f, 1000.0f);
  else if (!strcasecmp(key, "brake_level")) c.brakeLevel = clampv(v, 0.0f, 1.0f);
  else if (!strcasecmp(key, "esc_min"))     c.escMinUs     = (uint16_t)clampv(v, (float)ESC_ABS_MIN_US, 1500.0f);
  else if (!strcasecmp(key, "esc_neutral")) c.escNeutralUs = (uint16_t)clampv(v, 1300.0f, 1700.0f);
  else if (!strcasecmp(key, "esc_max"))     c.escMaxUs     = (uint16_t)clampv(v, 1500.0f, (float)ESC_ABS_MAX_US);
  else if (!strcasecmp(key, "servo_hz"))    c.servoHz   = (uint16_t)clampv(v, 50.0f, (float)SERVO_MAX_HZ);
  else if (!strcasecmp(key, "esc_hz"))      c.escHz     = (uint16_t)clampv(v, 50.0f, (float)ESC_MAX_HZ);
  else if (!strcasecmp(key, "control_hz"))  c.controlHz = (uint16_t)clampv(v, 20.0f, 500.0f);
  else return false;
  return true;
}

static void handleLine(char *line) {
  char *tok = strtok(line, " \t\r");
  if (!tok) return;
  char *a1 = strtok(nullptr, " \t\r");
  char *a2 = strtok(nullptr, " \t\r");

  if (!strcasecmp(tok, "D")) {  // D <heading_deg> <speed>
    if (!a1 || !a2) { Serial.println("ERR usage: D <heading_deg> <speed>"); return; }
    portENTER_CRITICAL(&g_mux);
    g_cmd.headingDeg = clampv(strtof(a1, nullptr), -180.0f, 180.0f);
    g_cmd.speed      = clampv(strtof(a2, nullptr), -1.0f, 1.0f);
    g_cmd.stampMs    = nowMs();
    portEXIT_CRITICAL(&g_mux);
    // no reply: commands stream at 20-50 Hz, telemetry is the feedback
  } else if (!strcasecmp(tok, "ARM")) {
    portENTER_CRITICAL(&g_mux);
    g_cmd.speed = 0; g_cmd.stampMs = nowMs(); g_armed = true;
    portEXIT_CRITICAL(&g_mux);
    Serial.println("OK ARM");
  } else if (!strcasecmp(tok, "DISARM") || !strcasecmp(tok, "STOP")) {
    portENTER_CRITICAL(&g_mux); g_armed = false; g_cmd.speed = 0; portEXIT_CRITICAL(&g_mux);
    Serial.println("OK DISARM");
  } else if (!strcasecmp(tok, "PING")) {
    Serial.printf("PONG %s %llu\n", a1 ? a1 : "0", nowMs());
  } else if (!strcasecmp(tok, "TIME")) {  // TIME <unix_ms>
    if (!a1) { Serial.println("ERR usage: TIME <unix_ms>"); return; }
    uint64_t ms = strtoull(a1, nullptr, 10);
    struct timeval tv = { (time_t)(ms / 1000), (suseconds_t)((ms % 1000) * 1000) };
    settimeofday(&tv, nullptr);
    g_timeSynced = true;
    Serial.printf("OK TIME %llu\n", ms);
  } else if (!strcasecmp(tok, "RAW")) {  // RAW <steer_us> | RAW OFF
    if (!a1) { Serial.println("ERR usage: RAW <us>|OFF"); return; }
    int us = strcasecmp(a1, "OFF") ? clampv<int>(atoi(a1), SERVO_ABS_MIN_US, SERVO_ABS_MAX_US) : -1;
    portENTER_CRITICAL(&g_mux); g_rawSteerUs = us; portEXIT_CRITICAL(&g_mux);
    Serial.printf("OK RAW %d\n", us);
  } else if (!strcasecmp(tok, "CAL")) {  // CAL LEFT|CENTER|RIGHT <us>
    if (!a1 || !a2) { printConfig(); return; }
    uint16_t us = clampv<int>(atoi(a2), SERVO_ABS_MIN_US, SERVO_ABS_MAX_US);
    bool ok = true;
    portENTER_CRITICAL(&g_mux);
    if      (!strcasecmp(a1, "LEFT"))   g_cfg.steerLeftUs = us;
    else if (!strcasecmp(a1, "CENTER")) g_cfg.steerCenterUs = us;
    else if (!strcasecmp(a1, "RIGHT"))  g_cfg.steerRightUs = us;
    else ok = false;
    portEXIT_CRITICAL(&g_mux);
    if (ok) printConfig(); else Serial.println("ERR CAL LEFT|CENTER|RIGHT <us>");
  } else if (!strcasecmp(tok, "SET")) {  // SET <key> <value>
    if (!a1 || !a2) { Serial.println("ERR usage: SET <key> <value>"); return; }
    Config c;
    portENTER_CRITICAL(&g_mux); c = g_cfg; portEXIT_CRITICAL(&g_mux);
    if (!setKey(c, a1, strtof(a2, nullptr))) { Serial.printf("ERR unknown key %s\n", a1); return; }
    if (!(c.escMinUs < c.escNeutralUs && c.escNeutralUs < c.escMaxUs)) {
      Serial.println("ERR need esc_min < esc_neutral < esc_max"); return;
    }
    portENTER_CRITICAL(&g_mux); g_cfg = c; portEXIT_CRITICAL(&g_mux);
    printConfig();
  } else if (!strcasecmp(tok, "GET")) {
    printConfig();
  } else if (!strcasecmp(tok, "SAVE")) {
    Config c;
    portENTER_CRITICAL(&g_mux); c = g_cfg; portEXIT_CRITICAL(&g_mux);
    bool between = (c.steerCenterUs > min(c.steerLeftUs, c.steerRightUs)) &&
                   (c.steerCenterUs < max(c.steerLeftUs, c.steerRightUs));
    if (!between) { Serial.println("ERR center must lie between left and right"); return; }
    Serial.println(saveConfig() ? "OK SAVE" : "ERR SAVE failed");
  } else if (!strcasecmp(tok, "DEFAULTS")) {
    portENTER_CRITICAL(&g_mux); g_cfg = Config(); portEXIT_CRITICAL(&g_mux);
    printConfig();  // not persisted until SAVE
  } else if (!strcasecmp(tok, "TEL")) {  // TEL <hz>, 0 = off
    g_telemetryHz = a1 ? clampv<int>(atoi(a1), 0, 50) : 10;
    Serial.printf("OK TEL %u\n", g_telemetryHz);
  } else if (!strcasecmp(tok, "SENS")) {  // reserved: future sensor reads
    Serial.printf("ERR SENS %s not implemented\n", a1 ? a1 : "?");
  } else if (!strcasecmp(tok, "INFO")) {
    Serial.printf("INFO fw=%s chip=%s rev=%d psram=%u flash=%u steer_pin=%d throttle_pin=%d\n",
                  FW_VERSION, ESP.getChipModel(), ESP.getChipRevision(),
                  ESP.getPsramSize(), ESP.getFlashChipSize(),
                  PIN_STEER_PWM, PIN_THROTTLE_PWM);
  } else {
    Serial.printf("ERR unknown command %s\n", tok);
  }
}

// ---------------------------------------------------------------------------
void setup() {
  // Outputs first: neutral pulses from the very first moment, so the ESC
  // learns 1500 us as neutral when it is switched on.
  // Always boot at 50 Hz / 1500 us; saved frame rates are applied by the
  // control task once it starts.
  pwmInit(PIN_STEER_PWM, STEER_TIMER, STEER_CH, 50, BOOT_NEUTRAL_US);
  pwmInit(PIN_THROTTLE_PWM, THROTTLE_TIMER, THROTTLE_CH, 50, BOOT_NEUTRAL_US);

  Serial.begin(115200);
  loadConfig();
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr,
                          configMAX_PRIORITIES - 2, nullptr, 1);
  Serial.printf("READY fw=%s\n", FW_VERSION);
}

void loop() {
  static char buf[96];
  static size_t len = 0;
  static uint64_t lastTel = 0;

  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n') { buf[len] = '\0'; handleLine(buf); len = 0; }
    else if (len < sizeof(buf) - 1) buf[len++] = ch;
    else len = 0;  // overlong line: drop it
  }

  bool fs;
  portENTER_CRITICAL(&g_mux); fs = g_failsafeEvt; g_failsafeEvt = false; portEXIT_CRITICAL(&g_mux);
  if (fs) Serial.println("EVT FAILSAFE command timeout -> disarmed");

  if (g_telemetryHz && nowMs() - lastTel >= 1000 / g_telemetryHz) {
    lastTel = nowMs();
    printTelemetry();
  }
  delay(1);
}
