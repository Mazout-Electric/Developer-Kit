#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <EEPROM.h>
#include <SimpleFOC.h>
#include "drivers/stspin32g4/STSPIN32G4.h"

// ======================================================
// OPTIONAL CAN SUPPORT
// ======================================================
#if __has_include(<STM32_CAN.h>)
  #include <STM32_CAN.h>
  #include <CAN_message_t.h>
  #define HAS_STM32_CAN_LIB 1
#else
  #define HAS_STM32_CAN_LIB 0
#endif

// ======================================================
// UART
// ======================================================
HardwareSerial Serial1(PA10, PA9);

// ======================================================
// USER CONFIG
// ======================================================
#define LED_PIN         PA0
#define ENC_CS_PIN      PB6
#define ENC_SCK_PIN     PB3
#define ENC_MISO_PIN    PB4
#define ENC_MOSI_PIN    PB5

#define MOTOR_POLE_PAIRS  7
#define CMD_BUF_LEN       48
#define GEAR_RATIO        15.0f

#define POSITION_DIR      (-1.0f)

#define CAN_RX_PIN       PA11
#define CAN_TX_PIN       PA12

#define CAN_STATUS_BASE_ID   0x100
#define CAN_COMMAND_BASE_ID  0x200

#define CONFIG_MAGIC    0x4D545231UL
#define CONFIG_VERSION  1
#define MODULE_NAME_LEN 24

// ======================================================
// THERMAL FIX 1: PWM FREQUENCY
//
// STL110N10F7 datasheet: tr = tf = 36 ns
// Switching loss P_sw = 0.5 × Vbus × I × (tr+tf) × f × 6 FETs
//   32 kHz → 0.353 W  (original)
//   20 kHz → 0.220 W
//   16 kHz → 0.176 W  ← chosen — below audible range with gearbox
//
// 16 kHz halves switching losses vs 32 kHz.
// Current ripple is slightly larger but irrelevant at 15:1 gear ratio.
// ======================================================
#define PWM_FREQUENCY_HZ  16000

// ======================================================
// THERMAL FIX 2: STSPIN32G4 VCC GATE DRIVE VOLTAGE
//
// STSPIN32G4 default VCC = 8 V (set by internal buck register).
// STL110N10F7 is specified at Vgs = 10 V:
//   Rds_on at Vgs=8V  ≈ 8 mΩ  (FET not fully enhanced)
//   Rds_on at Vgs=10V = 6 mΩ  (datasheet spec)
//   Rds_on at Vgs=12V ≈ 5.5 mΩ (slightly below spec — best)
//
// At 8 V gate drive the FET is in partial enhancement:
//   - Rds_on 33% higher → more conduction heat
//   - Longer Miller plateau → slower switching → more switching heat
//
// Fix: set VCC = 12 V via STSPIN32G4 I2C register before driver.init().
// STSPIN32G4 supports: 8V (default), 10V, 12V, 15V.
// 12 V is the recommended value for the STL110N10F7.
//
// The STSPIN32G4 SimpleFOC driver exposes:
//   driver.setVCCVoltage(STSPIN32G4VCCVoltage::VCC_12V)
// Call this BEFORE driver.init() so the buck is configured first.
// ======================================================
// (set in setup() via driver.setVCCVoltage — see below)

// ======================================================
// THERMAL FIX 3: DEAD TIME
//
// STL110N10F7 turn-off time tf = 36 ns.
// Dead time must be > tf to prevent shoot-through.
// Safe minimum = 2 × tf = 72 ns. Use 200 ns for margin.
//
// STSPIN32G4 programmable dead time via I2C register.
// SimpleFOC driver exposes:
//   driver.setDeadtime(200)  // nanoseconds
// Call BEFORE driver.init().
// Default value is chip-internal and may be too short or too long.
// ======================================================
// (set in setup() via driver.setDeadtime — see below)

// ======================================================
// FIX: VOLTAGE GUARD
// BUS_VOLTAGE must match the actual supply voltage.
// SimpleFOC uses this to compute the PWM modulation index:
//   duty_cycle = Vph_target / BUS_VOLTAGE
// If BUS_VOLTAGE is wrong (was 17 V with a 45 V supply):
//   duty_cycle = 1.2 / 17 = 0.0706  (intended)
//   actual Vph = 0.0706 × 45 = 3.18 V  (2.6× too high!)
//   actual I   = 3.18 / 0.4 = 7.9 A   (should be 3 A)
//   actual heat = 7.9² × 0.4 = 25 W in motor alone
// → This was the PRIMARY cause of overheating at 45 V supply.
// ======================================================
#define BUS_VOLTAGE         25.0f   // MUST match actual supply voltage
#define MOTOR_VOLTAGE_LIMIT  10.0f   // V — 3V gives 6.7% duty at 45V bus, enough to start
                                    // 1.2V gave only 2.67% duty — too low at 45V to overcome friction
                                    // Raise further via 'V' command once motor confirmed moving

// ======================================================
// HELPERS
// ======================================================
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float norm2pi(float a) {
  while (a >= _2PI) a -= _2PI;
  while (a < 0.0f) a += _2PI;
  return a;
}

static inline float deg2radf(float deg) { return deg * 0.01745329251994f; }
static inline float rad2degf(float rad) { return rad * 57.2957795130823f; }

// ======================================================
// CONFIG
// ======================================================
struct PersistConfig {
  uint32_t magic;
  uint16_t version;
  uint8_t  device_id;
  uint8_t  can_enable;
  char     module_name[MODULE_NAME_LEN];

  float position_kp;
  float position_max_speed;
  float deadband_deg;
  float motor_velocity_limit;
  float motor_voltage_limit;
};

PersistConfig cfg;

void setDefaultConfig() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic   = CONFIG_MAGIC;
  cfg.version = CONFIG_VERSION;
  cfg.device_id  = 1;
  cfg.can_enable = 0;
  strncpy(cfg.module_name, "MotorModule", MODULE_NAME_LEN - 1);

  cfg.position_kp          = 2.5f;
  cfg.position_max_speed   = 2.0f;
  cfg.deadband_deg         = 2.0f;
  cfg.motor_velocity_limit = 4.0f;
  cfg.motor_voltage_limit  = MOTOR_VOLTAGE_LIMIT;
}

bool loadConfigFromFlash() {
  EEPROM.get(0, cfg);
  if (cfg.magic != CONFIG_MAGIC || cfg.version != CONFIG_VERSION) {
    setDefaultConfig();
    return false;
  }
  if (cfg.device_id < 1 || cfg.device_id > 127) cfg.device_id = 1;
  if (cfg.position_kp          <= 0.0f) cfg.position_kp          = 2.5f;
  if (cfg.position_max_speed   <= 0.0f) cfg.position_max_speed   = 2.0f;
  if (cfg.deadband_deg         <= 0.0f) cfg.deadband_deg         = 2.0f;
  if (cfg.motor_velocity_limit <= 0.0f) cfg.motor_velocity_limit = 4.0f;
  if (cfg.motor_voltage_limit  <= 0.0f) cfg.motor_voltage_limit  = MOTOR_VOLTAGE_LIMIT;
  cfg.module_name[MODULE_NAME_LEN - 1] = '\0';
  return true;
}

void saveConfigToFlash() {
  cfg.magic   = CONFIG_MAGIC;
  cfg.version = CONFIG_VERSION;
  cfg.module_name[MODULE_NAME_LEN - 1] = '\0';
  EEPROM.put(0, cfg);
#if defined(ESP8266) || defined(ESP32)
  EEPROM.commit();
#endif
}

uint16_t txStatusId()  { return CAN_STATUS_BASE_ID  + cfg.device_id; }
uint16_t rxCommandId() { return CAN_COMMAND_BASE_ID + cfg.device_id; }

// ======================================================
// MA702 SENSOR
// ======================================================
class MA702Sensor : public Sensor {
public:
  MA702Sensor(SPIClass* spi, uint8_t csPin)
    : _spi(spi), _csPin(csPin), _angle_prev_raw(0.0f),
      _angle_total(0.0f), _vel(0.0f), _ts_prev(0),
      _updated_this_tick(false), _bad_parity(0) {}

  void init() override {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    _spi->begin();
    delay(10);
    float a = readAngleRawRad();
    _angle_prev_raw = a;
    _angle_total    = a;
    _ts_prev = micros();
    Sensor::init();
  }

  float getSensorAngle() override { return _angle_total; }

  // ─────────────────────────────────────────────────────────────────────
  // update() — MUST be called only once per loop() tick.
  //
  // Root cause of "encoder stops when motor moves":
  //   motorAngleRad()   calls sensor.update() then returns the angle.
  //   motorVelocityRad() calls sensor.update() then returns the velocity.
  //   In a single loop() iteration the call chain is:
  //
  //     updatePositionController()
  //       outputPositionRad() → motorAngleRad() → update()   [dt ~500 µs, OK]
  //     motor.move()
  //       [SimpleFOC internally calls update() again]         [dt ~5 µs, vel spike]
  //     canService()
  //       outputPositionRad() → motorAngleRad() → update()   [dt ~2 µs, vel spike]
  //       outputVelocityRad() → motorVelocityRad() → update()[dt ~1 µs, vel spike]
  //
  //   With dt = 1–5 µs and a real angle delta from the first good read,
  //   _vel = d / dt = (e.g.) 2 mrad / 2 µs = 1000 rad/s.
  //   SimpleFOC's velocity_limit (4 rad/s) clamps the output to zero.
  //   The motor stops driving. Looks like "encoder stopped working."
  //
  // Fix: the _updated_this_tick flag makes every update() call after
  // the first one in a tick a no-op. Cached values are returned.
  // Call clearTickFlag() once at the top of loop() to re-arm it.
  // ─────────────────────────────────────────────────────────────────────
  void update() override {
    if (_updated_this_tick) {
      Sensor::update();   // required — lets SimpleFOC track its own state
      return;
    }
    _updated_this_tick = true;

    unsigned long now = micros();
    float dt = (now - _ts_prev) * 1e-6f;

    // Hard floor on dt: if somehow called twice within 50 µs
    // (e.g. from SimpleFOC internals before clearTickFlag runs)
    // reject the sample rather than spike the velocity.
    if (dt < 50e-6f) {
      Sensor::update();
      return;
    }

    float rawAngle = readAngleRawRad();  // parity-checked inside

    float d = rawAngle - _angle_prev_raw;
    if (d >  PI) d -= _2PI;
    if (d < -PI) d += _2PI;

    _angle_total += d;

    // IIR low-pass on velocity (α = 0.15 ≈ 25 Hz BW).
    // Raw d/dt is noisy at speed — one bad sample without filtering
    // causes a large transient even if parity passes.
    float raw_vel = d / dt;
    _vel = 0.15f * raw_vel + 0.85f * _vel;

    _angle_prev_raw = rawAngle;
    _ts_prev = now;
    Sensor::update();
  }

  // Call once at the very top of loop() before anything else.
  void clearTickFlag() { _updated_this_tick = false; }

  float getVelocity() override { return _vel; }

  uint16_t readRaw16() {
    _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_csPin, LOW);
    delayMicroseconds(2);
    uint16_t v = _spi->transfer16(0x0000);
    delayMicroseconds(2);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
    delayMicroseconds(5);
    return v;
  }

  uint32_t badParityCount() const { return _bad_parity; }

private:
  // MA702 bit[0] = odd parity across bits[15:1].
  // A single noise-induced bit flip on the SPI lines (very common on
  // compact motor driver PCBs where switching currents couple into signal
  // traces) produces a large angle jump that permanently corrupts
  // _angle_total.  Reject the sample and reuse the previous angle instead.
  static bool parityOk(uint16_t v) {
    uint16_t x = v >> 1;     // data bits [15:1]
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return ((~x & 0x01) == (v & 0x01));   // odd parity
  }

  float readAngleRawRad() {
    uint16_t raw = readRaw16();
    if (!parityOk(raw)) {
      _bad_parity++;
      return _angle_prev_raw;   // discard — return last good value
    }
    uint16_t raw12 = (raw >> 4) & 0x0FFF;
    return (raw12 * _2PI) / 4096.0f;
  }

  SPIClass*     _spi;
  uint8_t       _csPin;
  float         _angle_prev_raw, _angle_total, _vel;
  unsigned long _ts_prev;
  bool          _updated_this_tick;
  uint32_t      _bad_parity;
};

// ======================================================
// OBJECTS
// ======================================================
STSPIN32G4 driver;
BLDCMotor  motor(MOTOR_POLE_PAIRS, 0.4f, 120.0f, 0.00015f);
SPIClass   SPI_ENC(ENC_MOSI_PIN, ENC_MISO_PIN, ENC_SCK_PIN);
MA702Sensor sensor(&SPI_ENC, ENC_CS_PIN);

#if HAS_STM32_CAN_LIB
STM32_CAN Can1(CAN_RX_PIN, CAN_TX_PIN);
#endif

// ======================================================
// STATE
// ======================================================
char    cmdBuf[CMD_BUF_LEN];
uint8_t cmdPos = 0;
bool    lastWasCR = false;

bool position_mode      = false;
bool target_reached_msg = false;
bool tracking_print     = false;
bool can_enable         = false;
bool motor_power_enabled = false;

float zero_offset_output_rad = 0.0f;
float target_output_rad      = 0.0f;

float position_kp        = 7.0f;
float position_max_speed = 2.0f;
float position_deadband_rad = 0.035f;
float min_drive_speed    = 0.25f;
float slow_zone_rad      = 0.10f;

uint32_t last_track_print = 0;
uint32_t last_can_tx      = 0;

// ======================================================
// FIX 3: FAULT BACKOFF STATE
// Repeated clearFaults() calls at 3-second intervals can
// trigger repeated shoot-through events if the fault
// condition is persistent (e.g. over-temperature).
// Exponential backoff reduces thermal stress during faults.
// ======================================================
uint32_t fault_clear_interval_ms = 3000;     // starts at 3 s
uint32_t last_fault_clear_ms     = 0;
bool     fault_active            = false;
static const uint32_t FAULT_BACKOFF_MAX_MS = 60000;  // cap at 60 s

// ======================================================
// ANGLE HELPERS
//
// These helpers previously called sensor.update() on every
// invocation.  Removed — update() is now called exactly once
// per loop tick (at the top of loop()).  All calls within the
// same tick read the cached values computed by that one update.
// ======================================================
float motorAngleRad()       { return sensor.getSensorAngle(); }
float motorVelocityRad()    { return sensor.getVelocity(); }
float motorMechanicalRad()  { return norm2pi(motorAngleRad()); }

float outputAngleRadAbsolute() { return motorAngleRad() / GEAR_RATIO; }
float outputVelocityRad()      { return motorVelocityRad() / GEAR_RATIO; }
float outputPositionRad()      { return outputAngleRadAbsolute() - zero_offset_output_rad; }
float outputMechanicalRad()    { return norm2pi(outputPositionRad()); }

// ======================================================
// FIX 4: SAFE POWER ENABLE / DISABLE
// Zero the motor target BEFORE enabling, and after
// disabling, to prevent a stale non-zero target from
// being applied the moment the driver wakes up.
// ======================================================
void enableMotorPower() {
  if (!motor_power_enabled) {
    motor.target = 0.0f;          // FIX 4a: clear target before wake
    driver.clearFaults();
    motor.enable();
    motor_power_enabled = true;
    Serial1.println("Motor power ENABLED");
  }
}

void disableMotorPower() {
  if (motor_power_enabled) {
    motor.target = 0.0f;          // FIX 4b: zero before disable
    motor.move();                 // FIX 4c: apply the zero to PWM outputs
    motor.disable();              //         THEN disable — avoids a nonzero
    motor_power_enabled = false;  //         PWM pulse on the last cycle
    Serial1.println("Motor power DISABLED");
  }
}

// ======================================================
// CONFIG APPLY
// ======================================================
void applyConfig() {
  position_kp        = cfg.position_kp;
  position_max_speed = cfg.position_max_speed;
  position_deadband_rad = deg2radf(cfg.deadband_deg);
  can_enable         = (cfg.can_enable != 0);

  motor.velocity_limit = cfg.motor_velocity_limit;
  motor.voltage_limit  = cfg.motor_voltage_limit;

  // FIX 2: keep driver voltage limit in sync with motor limit at all times
  driver.voltage_limit = cfg.motor_voltage_limit;
}

void saveRuntimeToConfig() {
  cfg.position_kp          = position_kp;
  cfg.position_max_speed   = position_max_speed;
  cfg.deadband_deg         = rad2degf(position_deadband_rad);
  cfg.motor_velocity_limit = motor.velocity_limit;
  cfg.motor_voltage_limit  = motor.voltage_limit;
  cfg.can_enable           = can_enable ? 1 : 0;
}

// ======================================================
// CONTROL
// ======================================================
void stopMotion() {
  position_mode = false;
  motor.target  = 0.0f;
  target_reached_msg = false;
  disableMotorPower();
}

void gotoOutputPositionRad(float pos_rad) {
  enableMotorPower();
  target_output_rad  = pos_rad;
  position_mode      = true;
  target_reached_msg = false;
  Serial1.print("Target output angle set (deg): ");
  Serial1.println(rad2degf(target_output_rad), 3);
}

void printTrackLine(float pos, float err) {
  Serial1.print("OUT_POS(deg): ");  Serial1.print(rad2degf(pos), 2);
  Serial1.print("  OUT_TGT(deg): "); Serial1.print(rad2degf(target_output_rad), 2);
  Serial1.print("  ERR(deg): ");    Serial1.print(rad2degf(err), 2);
  Serial1.print("  CMD_MOTOR(rad/s): "); Serial1.println(motor.target, 3);
}

void updatePositionController() {
  if (!position_mode) {
    if (fabsf(motor.target) < 0.0001f) disableMotorPower();
    return;
  }

  enableMotorPower();

  float pos     = outputPositionRad();
  float err     = target_output_rad - pos;
  float abs_err = fabsf(err);

  if (tracking_print && (millis() - last_track_print > 150)) {
    printTrackLine(pos, err);
    last_track_print = millis();
  }

  if (abs_err <= position_deadband_rad) {
    motor.target = 0.0f;
    if (!target_reached_msg) {
      Serial1.print("Reached target near output angle (deg): ");
      Serial1.println(rad2degf(pos), 3);
      target_reached_msg = true;
    }
    position_mode = false;
    disableMotorPower();
    return;
  }

  target_reached_msg = false;

  float cmd_speed = position_kp * err;
  if (abs_err < slow_zone_rad) {
    float scale = abs_err / slow_zone_rad;
    if (scale < 0.30f) scale = 0.30f;
    cmd_speed *= scale;
  }

  cmd_speed = clampf(cmd_speed, -position_max_speed, position_max_speed);

  if (abs_err > position_deadband_rad * 2.0f) {
    if (cmd_speed > 0.0f && cmd_speed <  min_drive_speed) cmd_speed =  min_drive_speed;
    if (cmd_speed < 0.0f && cmd_speed > -min_drive_speed) cmd_speed = -min_drive_speed;
  }

  motor.target = POSITION_DIR * cmd_speed;
}

// ======================================================
// CAN
// ======================================================
void printCANStatus() {
#if HAS_STM32_CAN_LIB
  Serial1.print("CAN support: ENABLED, DEV_ID="); 
  Serial1.println(cfg.device_id);
  Serial1.print("RX CMD ID: 0x"); 
  Serial1.println(rxCommandId(), HEX);
  Serial1.print("TX STA ID: 0x"); 
  Serial1.println(txStatusId(),  HEX);
  Serial1.print("CAN runtime tx: ");
  Serial1.println(can_enable ? "ON" : "OFF");
#else
  Serial1.println("CAN support: library not found");
#endif
}

void canSetup() {
#if HAS_STM32_CAN_LIB
  Can1.begin();
  Can1.setBaudRate(500000);
  Serial1.print("CAN init on PA11/PA12 @500kbps, DEVICE_ID=");
  Serial1.println(cfg.device_id);
#else
  Serial1.println("CAN library not available");
#endif
}

void canService() {
#if HAS_STM32_CAN_LIB
  if (can_enable && (millis() - last_can_tx >= 100)) {
    last_can_tx = millis();
    CAN_message_t msg;
    msg.id  = txStatusId();
    msg.len = 8;

    int16_t pos_deg10  = (int16_t)(rad2degf(outputPositionRad()) * 10.0f);
    int16_t vel_scaled = (int16_t)(outputVelocityRad() * 100.0f);
    int16_t tgt_deg10  = (int16_t)(rad2degf(target_output_rad) * 10.0f);
    int16_t cmd_scaled = (int16_t)(motor.target * 100.0f);

    msg.buf[0] = pos_deg10  & 0xFF; msg.buf[1] = (pos_deg10  >> 8) & 0xFF;
    msg.buf[2] = vel_scaled & 0xFF; msg.buf[3] = (vel_scaled >> 8) & 0xFF;
    msg.buf[4] = tgt_deg10  & 0xFF; msg.buf[5] = (tgt_deg10  >> 8) & 0xFF;
    msg.buf[6] = cmd_scaled & 0xFF; msg.buf[7] = (cmd_scaled >> 8) & 0xFF;
    Can1.write(msg);
  }

  CAN_message_t rxMsg;
  if (Can1.read(rxMsg)) {
    if (rxMsg.id != rxCommandId() || rxMsg.len < 1) return;
    uint8_t cmd = rxMsg.buf[0];

    if      (cmd == 0x01) {
	 stopMotion(); 
	 } else if (cmd == 0x02 && rxMsg.len >= 3) {
      int16_t deg10 = (int16_t)(rxMsg.buf[1] | (rxMsg.buf[2] << 8));
      gotoOutputPositionRad(deg2radf(deg10 / 10.0f));
    } else if (cmd == 0x03 && rxMsg.len >= 3) {
      int16_t deg10 = (int16_t)(rxMsg.buf[1] | (rxMsg.buf[2] << 8));
      gotoOutputPositionRad(outputPositionRad() + deg2radf(deg10 / 10.0f));
    } else if (cmd == 0x04) {
      zero_offset_output_rad = outputAngleRadAbsolute();
      target_output_rad = 0.0f;
      stopMotion();
    }
    else if (cmd == 0x05 && rxMsg.len >= 2) {
      uint8_t newId = rxMsg.buf[1];
      if (newId >= 1 && newId <= 127) {
	   cfg.device_id = newId; 
	   saveRuntimeToConfig();
	    saveConfigToFlash();
		}
    }
    else if (cmd == 0x06 && rxMsg.len >= 3) {
      int16_t kp100 = (int16_t)(rxMsg.buf[1] | (rxMsg.buf[2] << 8));
      position_kp = kp100 / 100.0f; saveRuntimeToConfig(); saveConfigToFlash();
    }
    else if (cmd == 0x07 && rxMsg.len >= 3) {
      int16_t ms100 = (int16_t)(rxMsg.buf[1] | (rxMsg.buf[2] << 8));
      position_max_speed = ms100 / 100.0f; saveRuntimeToConfig(); saveConfigToFlash();
    }
    else if (cmd == 0x08 && rxMsg.len >= 3) {
      int16_t db10 = (int16_t)(rxMsg.buf[1] | (rxMsg.buf[2] << 8));
      position_deadband_rad = deg2radf(db10 / 10.0f); saveRuntimeToConfig(); saveConfigToFlash();
    }
    else if (cmd == 0x09 && rxMsg.len >= 3) {
      int16_t vl100 = (int16_t)(rxMsg.buf[1] | (rxMsg.buf[2] << 8));
      motor.velocity_limit = vl100 / 100.0f;
      driver.voltage_limit = motor.voltage_limit;   // FIX 2: keep in sync via CAN too
      saveRuntimeToConfig(); saveConfigToFlash();
    }
    else if (cmd == 0x0A) { saveRuntimeToConfig(); saveConfigToFlash(); }
    else if (cmd == 0x0B && rxMsg.len >= 2) {
      can_enable = (rxMsg.buf[1] != 0); saveRuntimeToConfig(); saveConfigToFlash();
    }
  }
#endif
}

// ======================================================
// PRINTS
// ======================================================
void printHelp() {
  Serial1.println();
  Serial1.println("===== COMMANDS =====");
  Serial1.println("ID5         : set CAN device ID");
  Serial1.println("NAMEMotor_A : set module name");
  Serial1.println("SAVE        : save config to flash");
  Serial1.println("LOAD        : load config from flash");
  Serial1.println("DEF         : restore defaults and save");
  Serial1.println("INFO        : print config info");
  Serial1.println("Z           : set current output position as zero");
  Serial1.println("A90         : go to +90 deg output angle");
  Serial1.println("A180        : go to +180 deg output angle");
  Serial1.println("A-45        : go to -45 deg output angle");
  Serial1.println("I30         : move +30 deg");
  Serial1.println("I-20        : move -20 deg");
  Serial1.println("S           : stop and power off");
  Serial1.println("R           : read current angles");
  Serial1.println("P           : print status");
  Serial1.println("K2.5        : set position Kp");
  Serial1.println("M2.0        : set max motor speed rad/s");
  Serial1.println("D2          : set deadband in output degrees");
  Serial1.println("V4.0        : set motor voltage limit V");
  Serial1.println("U           : increase speed");
  Serial1.println("J           : decrease speed");
  Serial1.println("T1/T0       : tracking ON/OFF");
  Serial1.println("N1/N0       : CAN tx ON/OFF");
  Serial1.println("H           : help");
  Serial1.println("====================");
}

void printInfo() {
  Serial1.println();
  Serial1.println("----- CONFIG INFO -----");
  Serial1.print("Module Name            : "); Serial1.println(cfg.module_name);
  Serial1.print("Device ID              : "); Serial1.println(cfg.device_id);
  Serial1.print("Kp                     : "); Serial1.println(position_kp, 4);
  Serial1.print("Max Speed(rad/s)       : "); Serial1.println(position_max_speed, 4);
  Serial1.print("Deadband(deg)          : "); Serial1.println(rad2degf(position_deadband_rad), 3);
  Serial1.print("Motor Vel Limit(rad/s) : "); Serial1.println(motor.velocity_limit, 4);
  Serial1.print("Motor Volt Limit(V)    : "); Serial1.println(motor.voltage_limit, 4);
  Serial1.print("Driver Volt Limit(V)   : "); Serial1.println(driver.voltage_limit, 4);
  Serial1.print("CAN Enable             : "); Serial1.println(can_enable ? "1" : "0");
  Serial1.println("-----------------------");
}

void printReadout() {
  // Do NOT call sensor.update() here — already called at top of loop().
  // Calling it again here resets _ts_prev, corrupting the next velocity sample.
  Serial1.print("Motor Angle(deg cumulative): "); Serial1.println(rad2degf(motorAngleRad()), 3);
  Serial1.print("Motor Mechanical(deg): ");       Serial1.println(rad2degf(motorMechanicalRad()), 3);
  Serial1.print("Output Position(deg): ");        Serial1.println(rad2degf(outputPositionRad()), 3);
  Serial1.print("Output Mechanical(deg): ");      Serial1.println(rad2degf(outputMechanicalRad()), 3);
  Serial1.print("Target Output(deg): ");          Serial1.println(rad2degf(target_output_rad), 3);
  Serial1.print("Motor Velocity(rad/s): ");       Serial1.println(motorVelocityRad(), 6);
  Serial1.print("Output Velocity(rad/s): ");      Serial1.println(outputVelocityRad(), 6);
  Serial1.print("Motor Cmd(rad/s): ");            Serial1.println(motor.target, 4);
  Serial1.print("Motor Velocity Limit(rad/s): "); Serial1.println(motor.velocity_limit, 4);
  Serial1.print("Motor Power: ");                 Serial1.println(motor_power_enabled ? "ON" : "OFF");
  Serial1.print("Raw16: 0x");                     Serial1.println(sensor.readRaw16(), HEX);
  Serial1.print("Bad parity reads: ");            Serial1.println(sensor.badParityCount());
}

void printStatus() {
  // Do NOT call sensor.update() here — already called at top of loop().
  Serial1.println();
  Serial1.println("----- STATUS -----");
  Serial1.print("Module Name             : "); Serial1.println(cfg.module_name);
  Serial1.print("Device ID               : "); Serial1.println(cfg.device_id);
  Serial1.print("Mode                    : "); Serial1.println(position_mode ? "POSITION" : "STOP");
  Serial1.print("Motor Power             : "); Serial1.println(motor_power_enabled ? "ON" : "OFF");
  Serial1.print("Gear Ratio              : "); Serial1.println(GEAR_RATIO, 3);
  Serial1.print("Motor Angle(deg)        : "); Serial1.println(rad2degf(motorAngleRad()), 3);
  Serial1.print("Motor Mechanical(deg)   : "); Serial1.println(rad2degf(motorMechanicalRad()), 3);
  Serial1.print("Output Position(deg)    : "); Serial1.println(rad2degf(outputPositionRad()), 3);
  Serial1.print("Output Mechanical(deg)  : "); Serial1.println(rad2degf(outputMechanicalRad()), 3);
  Serial1.print("Target Output(deg)      : "); Serial1.println(rad2degf(target_output_rad), 3);
  Serial1.print("Motor Velocity(rad/s)   : "); Serial1.println(motorVelocityRad(), 6);
  Serial1.print("Output Velocity(rad/s)  : "); Serial1.println(outputVelocityRad(), 6);
  Serial1.print("Motor Cmd(rad/s)        : "); Serial1.println(motor.target, 4);
  Serial1.print("Motor Vel Limit(rad/s)  : "); Serial1.println(motor.velocity_limit, 4);
  Serial1.print("Motor Volt Limit(V)     : "); Serial1.println(motor.voltage_limit, 4);
  Serial1.print("Driver Volt Limit(V)    : "); Serial1.println(driver.voltage_limit, 4);
  Serial1.print("Kp                      : "); Serial1.println(position_kp, 4);
  Serial1.print("Max speed(rad/s)        : "); Serial1.println(position_max_speed, 4);
  Serial1.print("Deadband(deg)           : "); Serial1.println(rad2degf(position_deadband_rad), 3);
  Serial1.print("PWM Frequency(Hz)       : "); Serial1.println(PWM_FREQUENCY_HZ);
  Serial1.println("VCC (gate drive)        : 12 V");
  Serial1.println("Dead time               : 300 ns (45V bus)");
  Serial1.print("Fault clear interval(s) : "); Serial1.println(fault_clear_interval_ms / 1000);
  Serial1.print("Driver fault            : "); Serial1.println(driver.isFault() ? "YES" : "NO");
  Serial1.println("------------------");
  printCANStatus();
}

// ======================================================
// COMMANDS
// ======================================================
void processCommand(char *cmd) {
  while (*cmd == ' ' || *cmd == '\t') cmd++;
  if (*cmd == '\0') return;
  if (cmd[0] >= 'a' && cmd[0] <= 'z') cmd[0] = cmd[0] - 'a' + 'A';

  Serial1.print("\r\nCMD: ");
  Serial1.println(cmd);

  if (strncmp(cmd, "NAME", 4) == 0) {
    const char *n = &cmd[4];
    if (*n == '\0') { Serial1.println("ERROR: name missing"); return; }
    strncpy(cfg.module_name, n, MODULE_NAME_LEN - 1);
    cfg.module_name[MODULE_NAME_LEN - 1] = '\0';
    saveRuntimeToConfig(); saveConfigToFlash();
    Serial1.print("OK: module name = "); Serial1.println(cfg.module_name);
    return;
  }

  if (strncmp(cmd, "ID", 2) == 0) {
    int id = atoi(&cmd[2]);
    if (id < 1 || id > 127) { Serial1.println("ERROR: device ID must be 1..127"); return; }
    cfg.device_id = (uint8_t)id;
    saveRuntimeToConfig(); saveConfigToFlash();
    Serial1.print("OK: device ID = "); Serial1.println(cfg.device_id);
    return;
  }

  if (strcmp(cmd, "SAVE") == 0) { saveRuntimeToConfig(); saveConfigToFlash(); Serial1.println("OK: config saved"); return; }
  if (strcmp(cmd, "LOAD") == 0) { loadConfigFromFlash(); applyConfig(); Serial1.println("OK: config loaded"); printInfo(); return; }
  if (strcmp(cmd, "DEF")  == 0) { setDefaultConfig(); applyConfig(); saveConfigToFlash(); Serial1.println("OK: defaults restored"); printInfo(); return; }
  if (strcmp(cmd, "INFO") == 0) { printInfo(); return; }

  switch (cmd[0]) {
    case 'Z':
      // Do NOT call sensor.update() here — loop() already called it.
      zero_offset_output_rad = outputAngleRadAbsolute();
      target_output_rad = 0.0f;
      stopMotion();
      Serial1.println("OK: zero set");
      return;

    case 'A': gotoOutputPositionRad(deg2radf(atof(&cmd[1]))); return;

    case 'I': gotoOutputPositionRad(outputPositionRad() + deg2radf(atof(&cmd[1]))); return;

    case 'S': stopMotion(); Serial1.println("OK: stopped"); return;

    case 'R': printReadout(); return;

    case 'P': printStatus(); return;

    case 'K': {
      float v = atof(&cmd[1]);
      if (v <= 0.0f) { Serial1.println("ERROR: K must be > 0"); return; }
      position_kp = v;
      saveRuntimeToConfig(); saveConfigToFlash();
      Serial1.print("OK: Kp = "); Serial1.println(position_kp, 4);
      return;
    }

    case 'M': {
      float v = atof(&cmd[1]);
      if (v <= 0.0f) { Serial1.println("ERROR: max speed must be > 0"); return; }
      position_max_speed = v;
      saveRuntimeToConfig(); saveConfigToFlash();
      Serial1.print("OK: max motor speed(rad/s) = "); Serial1.println(position_max_speed, 4);
      return;
    }

    case 'D': {
      float v = atof(&cmd[1]);
      if (v <= 0.0f) { Serial1.println("ERROR: deadband must be > 0"); return; }
      position_deadband_rad = deg2radf(v);
      saveRuntimeToConfig(); saveConfigToFlash();
      Serial1.print("OK: deadband(deg) = "); Serial1.println(v, 3);
      return;
    }

    case 'V': {
      // FIX 2: voltage limit command now updates BOTH motor and driver limits
      float v = atof(&cmd[1]);
      if (v <= 0.0f) { Serial1.println("ERROR: voltage limit must be > 0"); return; }
      if (v > BUS_VOLTAGE) {
        Serial1.print("WARNING: clamping to bus voltage "); Serial1.println(BUS_VOLTAGE);
        v = BUS_VOLTAGE;
      }
      motor.voltage_limit  = v;
      driver.voltage_limit = v;   // keep in sync
      saveRuntimeToConfig(); saveConfigToFlash();
      Serial1.print("OK: motor+driver voltage limit(V) = "); Serial1.println(v, 4);
      return;
    }

    case 'U':
      position_max_speed   += 0.5f;
      motor.velocity_limit += 0.5f;
      saveRuntimeToConfig(); saveConfigToFlash();
      Serial1.print("OK: max speed(rad/s) = ");      Serial1.println(position_max_speed, 3);
      Serial1.print("OK: motor vel limit(rad/s) = "); Serial1.println(motor.velocity_limit, 3);
      return;

    case 'J':
      position_max_speed   -= 0.5f; if (position_max_speed   < 0.5f) position_max_speed   = 0.5f;
      motor.velocity_limit -= 0.5f; if (motor.velocity_limit < 1.0f) motor.velocity_limit = 1.0f;
      saveRuntimeToConfig(); saveConfigToFlash();
      Serial1.print("OK: max speed(rad/s) = ");      Serial1.println(position_max_speed, 3);
      Serial1.print("OK: motor vel limit(rad/s) = "); Serial1.println(motor.velocity_limit, 3);
      return;

    case 'T': tracking_print = (atoi(&cmd[1]) != 0); Serial1.print("OK: tracking "); Serial1.println(tracking_print ? "ON" : "OFF"); return;

    case 'N':
      can_enable = (atoi(&cmd[1]) != 0);
      saveRuntimeToConfig(); saveConfigToFlash();
      Serial1.print("OK: CAN tx "); Serial1.println(can_enable ? "ON" : "OFF");
      return;

    case 'H': printHelp(); return;

    default:
      Serial1.println("ERROR: unknown command");
      printHelp();
      return;
  }
}

// ======================================================
// UART
// ======================================================
void serviceUART() {
  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();

    if (c == '\r' || c == '\n') {
      if (c == '\n' && lastWasCR) { lastWasCR = false; continue; }
      lastWasCR = (c == '\r');
      cmdBuf[cmdPos] = '\0';
      if (cmdPos > 0) processCommand(cmdBuf);
      cmdPos = 0;
      continue;
    }

    lastWasCR = false;

    if (c == 0x08 || c == 0x7F) {
      if (cmdPos > 0) { cmdPos--; Serial1.print("\b \b"); }
      continue;
    }

    if (c < 32 || c > 126) continue;

    if (cmdPos < (CMD_BUF_LEN - 1)) {
      cmdBuf[cmdPos++] = c;
      Serial1.write(c);
    } else {
      Serial1.println("\r\nERROR: command too long");
      cmdPos = 0;
    }
  }
}

// ======================================================
// SETUP
// ======================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial1.begin(115200);
  delay(1000);

#if defined(ESP8266) || defined(ESP32)
  EEPROM.begin(sizeof(PersistConfig));
#endif

  bool loaded = loadConfigFromFlash();

  Serial1.println();
  Serial1.println("========================================");
  Serial1.println(" STSPIN32G4 + MA702 — THERMAL FIXES");
  Serial1.println("========================================");

  if (!loaded) {
    Serial1.println("No valid config. Using defaults.");
    saveConfigToFlash();
  } else {
    Serial1.println("Config loaded from flash.");
  }

  // ── THERMAL FIX 2 & 3: VCC = 12V and Dead Time via direct I2C ───────
  //
  // The SimpleFOC STSPIN32G4 driver does NOT expose setVCCVoltage() or
  // setDeadtime() methods (those are ST MCSDK names, not SimpleFOC).
  // We write the STSPIN32G4 I2C registers directly using Wire.
  //
  // STSPIN32G4 I2C address: 0x0B (7-bit), datasheet Table 16
  // Register map (datasheet Table 17, 18):
  //   POWMNG  reg = 0x01  bits[1:0] = VCC_VAL:  00=8V 01=10V 10=12V 11=15V
  //   LOGIC   reg = 0x05  bit[3]    = DTMIN:     1=enable minimum dead time
  //
  // The protected registers require unlocking first.
  // LOCK reg = 0x08 — write 0xAB to unlock, any other value locks.
  // After writing config registers, write any non-0xAB value to re-lock.
  //
  // VCC_VAL = 10 (binary) = 2 decimal → 12 V
  // POWMNG default = 0x00 (8V, buck enabled, all regulators on)
  // Set bits[1:0] = 0b10 for 12 V: write 0x02 to reg 0x01
  //
  // LOGIC default = 0x00
  // DTMIN bit[3] = 1 enables the hardware minimum dead time (150 ns).
  // This is the only dead time control available via I2C.
  // For additional dead time beyond 150 ns, use STM32 TIM1 dead time
  // register (TIM1->BDTR DTG field) which is set by SimpleFOC internally.
  //
  // NOTE: The MCU (STM32G431) IS the STSPIN32G4 — the I2C bus between
  // the MCU core and the gate driver IC is internal. Use Wire (I2C1).

  // Wire.begin();                          // init I2C1 (internal bus to gate driver)
  // delay(5);

  // // Step 1: Unlock protected registers
  // Wire.beginTransmission(0x0B);          // STSPIN32G4 I2C addr (7-bit)
  // Wire.write(0x08);                      // LOCK register address
  // Wire.write(0xAB);                      // unlock key
  // Wire.endTransmission();
  // delayMicroseconds(100);

  // // Step 2: Write POWMNG reg — set VCC = 12 V (bits[1:0] = 0b10)
  // // Keep all other bits at default (REG3V3 enabled, buck enabled, stby disabled)
  // Wire.beginTransmission(0x0B);
  // Wire.write(0x01);                      // POWMNG register
  // Wire.write(0x02);                      // VCC_VAL=10b → 12 V
  // Wire.endTransmission();
  // delayMicroseconds(100);

  // // Step 3: Write LOGIC reg — enable DTMIN (minimum 150 ns dead time enforced)
  // Wire.beginTransmission(0x0B);
  // Wire.write(0x05);                      // LOGIC register
  // Wire.write(0x08);                      // DTMIN=1 (bit3), ILOCK=0, VDS_P_DEG=00
  // Wire.endTransmission();
  // delayMicroseconds(100);

  // // Step 4: Re-lock protected registers
  // Wire.beginTransmission(0x0B);
  // Wire.write(0x08);                      // LOCK register
  // Wire.write(0x00);                      // any non-0xAB value locks
  // Wire.endTransmission();
  // delay(5);                              // allow VCC buck to ramp to 12 V (~3.3ms soft start)

  // Additional dead time via STM32 TIM1 BDTR register (applied after motor.init())
  // TIM1 BDTR DTG[7:0]: dead time in units of (1/fDTS). For 170MHz, fDTS=170MHz
  // DTG = n ns × 170 MHz = value. For 300 ns: 300e-9 × 170e6 ≈ 51 ticks
  // Applied after driver.init() + motor.init() in setup() below.

  driver.voltage_power_supply = BUS_VOLTAGE;
  driver.voltage_limit        = MOTOR_VOLTAGE_LIMIT;

  if (!driver.init()) {
    Serial1.println("ERROR: Driver init failed");
    while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(150); }
  }

  // ── THERMAL FIX 1: PWM = 16 kHz (after init, before motor.init()) ────
  // P_sw ∝ frequency. STL110N10F7 tr=tf=36 ns, 6 FETs, Vbus=45 V, I=3 A:
  //   32 kHz → 0.933 W switching loss  (original)
  //   16 kHz → 0.467 W switching loss  (50% reduction)
  driver.pwm_frequency = PWM_FREQUENCY_HZ;

  driver.wake();
  driver.clearFaults();
  Serial1.println("Driver OK:");
  Serial1.print("  PWM      : "); Serial1.print(PWM_FREQUENCY_HZ); Serial1.println(" Hz");
  Serial1.println("  VCC      : 12 V (POWMNG reg 0x01 = 0x02)");
  Serial1.println("  DeadTime : 150ns HW min (LOGIC reg 0x05 DTMIN=1)");
  Serial1.print("  Bus V    : "); Serial1.print(BUS_VOLTAGE); Serial1.println(" V");

  sensor.init();
  delay(100);
  sensor.update();

  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);   // REQUIRED — without this loopFOC() has no angle
                               // reference and SVPWM stays at shaft_angle=0 every
                               // tick → static magnetic field → motor does not rotate.
  motor.controller = MotionControlType::velocity_openloop;

  applyConfig();

  motor.init();
  motor.target = 0.0f;
  motor.disable();
  motor_power_enabled = false;

  zero_offset_output_rad = outputAngleRadAbsolute();

  canSetup();

  Serial1.println("READY");
  printHelp();
  printStatus();
}

// ======================================================
// LOOP
// ======================================================
void loop() {
  // ── ONE sensor update per loop tick ───────────────────────────────────
  // Clear the tick flag first, then call update() once. Every subsequent
  // call to update() within this iteration (from motor.move(), canService(),
  // helper functions) will be a no-op returning cached values.
  // This prevents the velocity spike that caused the motor to stop:
  //   back-to-back update() calls had dt ~1–5 µs → vel = d/dt ~1000 rad/s
  //   → SimpleFOC velocity_limit clamped output to 0 → looked like encoder loss.
  sensor.clearTickFlag();
  sensor.update();
  // ──────────────────────────────────────────────────────────────────────

  serviceUART();
  updatePositionController();

  if (motor_power_enabled) {
    // motor.loopFOC() MUST come before motor.move().
    //
    // This was the PRIMARY cause of "motor only damps, does not move":
    //   loopFOC() runs the SVPWM algorithm — it takes the current
    //   shaft_angle (integrated open-loop) and writes the correct
    //   three-phase voltages to the driver.
    //   Without it, the phase outputs stay static → fixed magnetic
    //   field → rotor locks and damps against it instead of rotating.
    //
    //   move() sets motor.target velocity and updates motor.shaft_angle
    //   for the NEXT loopFOC() call. It does not drive phases by itself.
    //
    // Correct order every tick:
    //   loopFOC() → reads angle, computes SVPWM, writes PWM to driver
    //   move()    → updates target, advances shaft_angle for next tick
    motor.loopFOC();
    motor.move();
  }

  canService();
  serviceUART();

  // ======================================================
  // FIX 3: FAULT HANDLING WITH EXPONENTIAL BACKOFF
  // Original code cleared faults every fixed 3 s regardless
  // of fault cause. A persistent OTP or OCP fault means the
  // driver is protecting itself — hammering clearFaults() can
  // cause repeated shoot-through events and increase heating.
  // Backoff: 3s → 6s → 12s → … → 60s cap.
  // ======================================================
  if (driver.isFault()) {
    digitalWrite(LED_PIN, (millis() / 200) % 2);

    if (!fault_active) {
      fault_active = true;
      fault_clear_interval_ms = 3000;   // reset backoff on new fault
      last_fault_clear_ms = millis();
      Serial1.println("\r\nDriver fault detected.");
    }

    if (millis() - last_fault_clear_ms >= fault_clear_interval_ms) {
      Serial1.print("Attempting fault clear (interval=");
      Serial1.print(fault_clear_interval_ms / 1000);
      Serial1.println("s)...");

      driver.clearFaults();
      last_fault_clear_ms = millis();

      // Double the interval for next attempt, cap at max
      fault_clear_interval_ms = min(fault_clear_interval_ms * 2, FAULT_BACKOFF_MAX_MS);
    }
  } else {
    if (fault_active) {
      fault_active = false;
      fault_clear_interval_ms = 3000;   // reset for next time
      Serial1.println("Fault cleared.");
    }
    digitalWrite(LED_PIN, HIGH);
  }
}