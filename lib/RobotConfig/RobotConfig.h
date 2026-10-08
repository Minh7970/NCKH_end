#pragma once
#include <Arduino.h>

// Set these for the local Wi-Fi network; blank SSID enables fallback AP mode.
constexpr char WIFI_SSID[] = "Nha Tro235 L2";
constexpr char WIFI_PASSWORD[] = "TH909002@*@";
constexpr char WIFI_AP_SSID[] = "ESP32S3-FireRobot";
constexpr char WIFI_AP_PASSWORD[] = "12345678";
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;

/*
 * Fire Robot - ESP32 DevKitC V4
 * Modular PlatformIO configuration.
 *
 * NOTE:
 * HC-SR04 cannot reliably measure a 1 cm gap. Its practical minimum
 * distance is around 2-3 cm and the robot chassis/turning error is larger.
 * Therefore TARGET_WALL_GAP_CM is kept as a mapping target only.
 * PHYSICAL_STOP_CM must be >= 5 cm for a safer prototype.
 */

// L298N left
constexpr uint8_t PIN_ENA = 25;
constexpr uint8_t PIN_IN1 = 26;
constexpr uint8_t PIN_IN2 = 27;

// L298N right
constexpr uint8_t PIN_ENB = 14;
constexpr uint8_t PIN_IN3 = 16;
constexpr uint8_t PIN_IN4 = 17;
 
// Flame module analogue AO outputs: LEFT AO -> GPIO32, RIGHT AO -> GPIO33.
constexpr uint8_t PIN_FLAME_LEFT   = 32;
constexpr uint8_t PIN_FLAME_RIGHT  = 33;
// Keep the digital implementation available, but use the original analogue
// fixed-angle/flicker detector for the current wiring.
constexpr bool FLAME_USE_DIGITAL_OUTPUT = false;
constexpr bool FLAME_DIGITAL_ACTIVE_LOW = true;

// MQ2 input is retained only as a disconnected legacy definition. GPIO35 is
// input-only and must never be used as a servo signal.
constexpr uint8_t PIN_MQ2 = 35;
// The pump tube is fixed straight ahead at the flame-servo 0-degree heading.
// There is no nozzle servo in the current hardware revision.
constexpr uint8_t PUMP_FIXED_HEADING_DEG = 0;
constexpr uint8_t FLAME_ALIGNMENT_TOLERANCE_DEG = 5;
// MQ-2 is no longer installed in this hardware revision.
constexpr bool MQ2_SENSOR_ENABLED = false;
constexpr bool MQ2_SIGNAL_ACTIVE_HIGH = true;
constexpr uint16_t MQ2_DETECT_DELTA = 350;
constexpr uint8_t MQ2_CONFIRM_SAMPLES = 10;
constexpr uint32_t MQ2_WARMUP_MS = 60000;
// Enable MQ-2 standby only when the sensor is installed and enabled above.
constexpr bool MQ2_SLEEP_MODE_ENABLED = true;
constexpr uint32_t MQ2_SLEEP_INTERVAL_MS = 500;
// After all reachable FREE cells are covered, wake briefly once per second
// for safety/telemetry checks and then return to light sleep.
constexpr uint32_t COVERAGE_SLEEP_INTERVAL_MS = 1000;

/*  
 * A real 3-5 um MWIR detector with an onboard band-pass filter must drive
 * this pin. It is a digital confirmation signal, not an ordinary flame
 * sensor input. GPIO36 is input-only and needs an external 10k pulldown.
 */
constexpr uint8_t PIN_MWIR_CONFIRM = 36;
constexpr bool MWIR_SENSOR_ENABLED = false;
constexpr bool MWIR_CONFIRM_ACTIVE_HIGH = true;
// Current hardware confirms fire using only the two analogue flame channels.
constexpr bool REQUIRE_MWIR_CONFIRMATION = false;
// Set true only when gas/smoke confirmation is mandatory for your test area.
constexpr bool REQUIRE_GAS_CONFIRMATION = false;
static_assert(!REQUIRE_MWIR_CONFIRMATION || MWIR_SENSOR_ENABLED,
              "MWIR confirmation requires an installed MWIR sensor");
static_assert(!REQUIRE_GAS_CONFIRMATION || MQ2_SENSOR_ENABLED,
              "Gas confirmation requires an installed MQ-2 sensor");

// MPU6050 relative heading sensor. GPIO21/22 are already used by the scanning
// ultrasonic sensor, therefore this project uses a separate I2C bus mapping.
// Wiring: VCC->3V3, GND->GND, SDA->GPIO23, SCL->GPIO5, AD0->GND.
constexpr bool MPU6050_ENABLED = true;
constexpr uint8_t PIN_MPU_SDA = 23;
constexpr uint8_t PIN_MPU_SCL = 5;
constexpr uint8_t MPU6050_I2C_ADDRESS = 0x68;
constexpr uint32_t MPU6050_I2C_FREQUENCY = 400000;
constexpr uint16_t MPU6050_CALIBRATION_SAMPLES = 120;
constexpr uint32_t MPU6050_READ_PERIOD_MS = 20;
constexpr float MPU6050_GYRO_SCALE_DPS = 131.0f; // +/-250 dps
// Use -1 only when the module is mounted upside down and heading correction
// moves farther away from a straight line.
constexpr int8_t MPU6050_YAW_SIGN = 1;
constexpr float MPU6050_GYRO_DEADBAND_DPS = 0.45f;
constexpr uint8_t MPU6050_MAX_READ_FAILURES = 5;
constexpr uint32_t MPU6050_FRESH_MS = 150;

// Relative-yaw control. MPU6050 has no magnetometer, so NORTH is the robot's
// startup direction; the gyro then preserves relative 90/180-degree headings.
constexpr float MPU_TURN_90_STOP_DEG = 82.0f;
constexpr float MPU_TURN_180_STOP_DEG = 172.0f;
constexpr uint32_t MPU_TURN_MIN_MS = 100;
constexpr uint32_t MPU_TURN_MAX_90_MS = 2500;
constexpr float MPU_HEADING_HOLD_DEADBAND_DEG = 1.0f;
constexpr float MPU_HEADING_HOLD_KP = 2.0f;
constexpr int16_t MPU_HEADING_HOLD_MAX_PWM = 35;
constexpr float MPU_FIRE_RETURN_TOLERANCE_DEG = 3.0f;
constexpr uint32_t MPU_FIRE_RETURN_MAX_MS = 5000;

// HC-SR04
constexpr uint8_t PIN_TRIG = 21;
constexpr uint8_t PIN_ECHO = 22;
constexpr uint32_t ULTRASONIC_TIMEOUT_US = 20000;

// SA2: fixed forward-facing SRF05 for live obstacle avoidance.
constexpr uint8_t PIN_SA2_TRIG = 4;
constexpr uint8_t PIN_SA2_ECHO = 13;

// Pump / scan servo
constexpr uint8_t PIN_PUMP  = 18;
constexpr uint8_t PIN_SERVO = 19;

// Flame scanning servos (mỗi servo mang 1 cảm biến lửa trái/phải, quét đối xứng
// để mở rộng vùng phát hiện; đã bỏ cảm biến lửa ở giữa)
constexpr uint8_t PIN_FLAME_SERVO_LEFT  = 2;
constexpr uint8_t PIN_FLAME_SERVO_RIGHT = 15;

// Flame servo scan configuration
constexpr uint8_t FLAME_SERVO_MIN_DEG    = 0;
constexpr uint8_t FLAME_SERVO_MAX_DEG    = 180;
constexpr uint8_t FLAME_SERVO_CENTER_DEG = 90;
constexpr uint8_t FLAME_SERVO_STEP_DEG   = 5;
constexpr uint32_t FLAME_SERVO_INTERVAL_MS = 100;
// Directional aim positions; tune FRONT/REAR for the actual brackets.
constexpr uint8_t FLAME_AIM_LEFT_DEG = 0;
constexpr uint8_t FLAME_AIM_RIGHT_DEG = 180;
constexpr uint8_t FLAME_AIM_FRONT_DEG = 90;
constexpr uint8_t FLAME_AIM_REAR_DEG = 0;

// Positional SG90 servo carrying the SRF04.
constexpr uint8_t SERVO_SCAN_MIN_DEG = 0;
constexpr uint8_t SERVO_SCAN_MAX_DEG = 180;
constexpr uint8_t SERVO_SCAN_CENTER_DEG = 90;
constexpr uint8_t SERVO_SCAN_STEP_DEG = 5;
// A 5-degree SG90 move only needs a short settling interval. Keep a separate
// longer delay for the first 90 -> 0 degree move after startup/manual mode.
constexpr uint32_t SERVO_SCAN_SETTLE_MS = 35;
constexpr uint32_t SERVO_SCAN_START_SETTLE_MS = 200;
constexpr uint32_t SERVO_SCAN_END_PAUSE_MS = 80;
static_assert((SERVO_SCAN_MAX_DEG - SERVO_SCAN_MIN_DEG) % SERVO_SCAN_STEP_DEG == 0,
			  "Servo scan range must be divisible by the angle step");

constexpr bool PUMP_ACTIVE_HIGH   = true;

// Bounds checking
constexpr uint16_t FLAME_ADC_MIN = 0;
constexpr uint16_t FLAME_ADC_MAX = 4095;
constexpr uint16_t FLAME_SIGNAL_MIN = 0;
// Direction classification accepts the full 12-bit analogue signal.
constexpr uint16_t FLAME_SIGNAL_MAX = 4095;
constexpr uint16_t FLAME_AMBIENT_MIN = 100;
constexpr uint16_t FLAME_AMBIENT_MAX = 3500;

// Pump safety
constexpr uint32_t PUMP_MAX_SPRAY_TIME_MS = 10000;  // 10s tối đa
constexpr uint32_t PUMP_COOLDOWN_MS = 500;          // Cooldown 500ms
constexpr uint8_t PUMP_MAX_CYCLES_PER_SESSION = 3;  // Tối đa 3 chu kỳ
// Spray as soon as the filtered flame detector confirms a fire. Set false if
// the robot must approach until FLAME_NEAR_DELTA is reached before spraying.
constexpr bool PUMP_ON_CONFIRMED_FIRE = true;

// Task priorities
#define TASK_PRIORITY_SENSOR 4
#define TASK_PRIORITY_FIRE_SAFETY 5
// L298N PWM configuration, matching the working standalone sketch.
constexpr uint32_t MOTOR_PWM_FREQ = 15000;
constexpr uint8_t MOTOR_PWM_BITS = 8;
// Keep motor PWM away from the low-numbered LEDC channels commonly allocated
// by ESP32Servo on Arduino-ESP32 core 2.x.
constexpr uint8_t MOTOR_PWM_CHANNEL_LEFT = 6;
constexpr uint8_t MOTOR_PWM_CHANNEL_RIGHT = 7;
// Hard cap applied after per-wheel trim: 80% of the 8-bit PWM range.
constexpr uint8_t MOTOR_MAX_PWM_PERCENT = 85;
constexpr uint8_t MOTOR_PWM_LIMIT =
  (uint16_t)255 * MOTOR_MAX_PWM_PERCENT / 100;
static_assert(MOTOR_MAX_PWM_PERCENT <= 100,
              "Motor PWM limit must be at most 100 percent");

// Motor speed settings (PWM 0..255), grouped for simple tuning like the .ino.
constexpr uint8_t MOTOR_SPEED_MANUAL = 160;
constexpr uint8_t MOTOR_SPEED_SEARCH = 140;
constexpr uint8_t MOTOR_SPEED_AUTO   = 140;
constexpr uint8_t MOTOR_SPEED_TURN   = MOTOR_PWM_LIMIT;
constexpr uint8_t MOTOR_SPEED_STEER  = 130;
constexpr uint8_t MOTOR_SPEED_FIRE   = MOTOR_PWM_LIMIT;

// Start both sides without amplification. The previous 180% values forced
// every autonomous command into the 80% hard cap, leaving no room to balance
// a faster side during straight-line calibration.
constexpr uint8_t MOTOR_TRIM_LEFT_PCT  = 100;
constexpr uint8_t MOTOR_TRIM_RIGHT_PCT = 100;
constexpr uint8_t MOTOR_MIN_START_PWM  = 0;
constexpr uint8_t ARC_INNER_PCT        = 35;

// Existing module names mapped to the easy-to-tune settings above.
constexpr uint8_t PWM_EXPLORE = MOTOR_SPEED_SEARCH;
constexpr uint8_t PWM_PATROL  = MOTOR_SPEED_AUTO;
constexpr uint8_t PWM_TURN    = MOTOR_SPEED_TURN;
constexpr uint8_t PWM_FIRE    = MOTOR_SPEED_FIRE;

/*
 * Flame validation for ESP32's 12-bit ADC (0..4095).
 * Most analog flame modules become LOWER when IR intensity rises.  Detection
 * is based on a drop from the learned ambient value plus a flame-like flicker
 * signal, so steady sunlight/lamps are rejected more reliably.
 */
constexpr bool FLAME_SIGNAL_ACTIVE_LOW = true;
constexpr uint16_t FLAME_DETECT_DELTA = 450;
constexpr uint16_t FLAME_NEAR_DELTA = 1100;
// Absolute and relative AC components must both look like flame flicker.
constexpr uint16_t FLAME_FLICKER_DELTA = 80;
constexpr uint8_t FLAME_FLICKER_MIN_PERCENT = 5;
constexpr uint16_t FLAME_FLICKER_STEP_DELTA = 20;
constexpr uint8_t FLAME_FLICKER_MIN_REVERSALS = 3;
constexpr uint8_t FLAME_FLICKER_MIN_VARIATION_MULTIPLIER = 2;
// Direct sunlight can drive a near-IR phototransistor against an ADC rail.
constexpr uint16_t FLAME_ADC_SATURATION_MARGIN = 40;
constexpr uint8_t FLAME_MAX_SATURATED_PERCENT = 50;
// When a strong IR change is found, stop both flame servos at that angle and
// verify temporal flicker without servo motion contaminating the samples.
constexpr uint32_t FLAME_VERIFY_SETTLE_MS = 100;
constexpr uint32_t FLAME_VERIFY_TIMEOUT_MS = 1800;
constexpr uint32_t FLAME_VERIFY_REJECT_COOLDOWN_MS = 600;
constexpr uint8_t FLAME_VERIFY_MIN_SAMPLES = 12;
constexpr uint8_t FLAME_LOST_SAMPLES = 8;
// During synchronized body/servo alignment the sensor view changes by design,
// so the analogue signal can disappear briefly even though the same flame is
// still being handled. Keep the confirmed state through the longest 180-degree
// turn, then return to the normal short loss filter once alignment is complete.
constexpr uint8_t FLAME_ALIGNMENT_LOST_SAMPLES = 80;
constexpr uint16_t FIRE_DIRECTION_MARGIN = 80;
constexpr bool SENSOR_DEBUG = false;
constexpr uint32_t SENSOR_DEBUG_PERIOD_MS = 1000;
constexpr bool SA2_DEBUG = true;
constexpr uint32_t SA2_DEBUG_PERIOD_MS = 250;

// Mapping
constexpr uint8_t MAP_W = 40;
constexpr uint8_t MAP_H = 40;
constexpr uint8_t MAP_CELL_CM = 5;
constexpr uint8_t START_X = MAP_W / 2;
constexpr uint8_t START_Y = MAP_H / 2;
// Two matching ultrasonic observations are required before changing a cell.
constexpr uint8_t DYNAMIC_OBSTACLE_CONFIRMATIONS = 2;
constexpr uint8_t DYNAMIC_FREE_CONFIRMATIONS = 2;
// Inflate a confirmed echo by 10 cm so a large surface is represented as a
// no-go footprint instead of a one-cell-thick point on the map.
constexpr uint8_t MAP_OBSTACLE_INFLATION_CELLS = 2;
constexpr uint32_t COVERAGE_RESCAN_MS = 60000;

// Mapping target vs live obstacle avoidance.
constexpr uint8_t TARGET_WALL_GAP_CM = 25;
constexpr uint8_t PHYSICAL_STOP_CM   = 15;
constexpr uint8_t OBSTACLE_MAP_CM    = 25;
// SA2 is the close-range avoidance trigger requested for the chassis.
// The comparison is inclusive, so a valid reading of exactly 15 cm also stops.
constexpr uint8_t FORWARD_CLEARANCE_CM = PHYSICAL_STOP_CM;
constexpr uint8_t SA2_OBSTACLE_DISTANCE_CM = PHYSICAL_STOP_CM;
// false: no echo means "not detected" and does not immobilize the robot.
// A valid SA2 echo at or below the threshold still stops immediately.
constexpr bool SA2_STOP_ON_INVALID_READING = false;
constexpr uint8_t RADAR_FORWARD_CONE_DEG = 20;
constexpr uint8_t RADAR_EMERGENCY_STOP_CM = PHYSICAL_STOP_CM;
constexpr uint32_t RADAR_FORWARD_FRESH_MS = 1500;
// Do not write moving scans into the persistent occupancy map. Without an IMU
// or wheel encoders, the pose cannot be trusted while the chassis is moving.
constexpr bool RADAR_MAP_ONLY_WHEN_STOPPED = true;
// A timeout/no-echo normally means that no reflecting surface was found in
// range. Do not assume the whole ray is clear: release only the immediately
// adjacent 5 cm cell so exploration can start in a wide/open area. Set false
// if a missing radar echo must stop the robot instead.
constexpr bool RADAR_NO_ECHO_CLEARS_ADJACENT_CELL = true;
constexpr uint16_t RADAR_NO_ECHO_CLEAR_CM = OBSTACLE_MAP_CM + MAP_CELL_CM;
// Backtrack 25 cm first, evaluate alternate routes, then extend one verified
// path cell at a time up to 50 cm when no escape direction is available.
constexpr uint8_t AVOID_BACKTRACK_MIN_CELLS = 5;
constexpr uint8_t AVOID_BACKTRACK_MAX_CELLS = 10;
constexpr uint8_t AVOID_ROUTE_LOOKAHEAD_CELLS = 10;
static_assert(AVOID_BACKTRACK_MIN_CELLS <= AVOID_BACKTRACK_MAX_CELLS,
              "Minimum avoidance backtrack must not exceed maximum");
constexpr uint32_t SA2_AVOID_BACK_MS = 580;
constexpr uint32_t SA2_AVOID_TURN_90_MS = 575;
constexpr uint32_t SA2_AVOID_FORWARD_MS = 1300;
constexpr uint8_t SA2_OBSTACLE_CONFIRM_SAMPLES = 3;
constexpr uint8_t SA2_CRITICAL_STOP_CM = 5;
constexpr uint32_t ULTRASONIC_SETTLE_AFTER_TURN_MS = 100;

// AUTO_CRUISE pose estimate interval for one 5 cm cell; motor does not stop here.
// Calibrate this time against a measured 5 cm forward run at MOTOR_SPEED_AUTO.
// Conservative starting calibration after the log showed 39 cells being
// counted in only 6.0 seconds. Measure a real 5 cm run and tune this value.
constexpr uint32_t AUTO_POSE_CELL_INTERVAL_MS = 400;
constexpr uint32_t TURN_90_MS = 400;
constexpr uint32_t TURN_SETTLE_MS = 130;
constexpr uint32_t SCAN_BODY_TURN_MS = 2 * TURN_90_MS;
constexpr uint8_t SCAN_CYCLES_PER_POSE = 2;
constexpr uint8_t MAX_TURNS_WITHOUT_FORWARD = 2;

// First-run exploration
constexpr uint16_t MAX_EXPLORE_STEPS = 180;
constexpr uint16_t MAX_NO_NEW_CELL = 45;

// Sensors / RTOS
constexpr uint32_t ULTRASONIC_PERIOD_MS = 60;
constexpr uint32_t MAP_SAVE_PERIOD_MS = 5000;
constexpr uint32_t SENSOR_HEARTBEAT_TIMEOUT_MS = 700;
// Keep the robot still while the flame sensors learn the ambient IR level.
constexpr uint32_t FIRE_ARM_DELAY_MS = 2500;
constexpr uint8_t FIRE_CONFIRM_SAMPLES = 8;

// Bounded mapping scan driven by the top-mounted positional servo.
// Two samples plus the mapper's cross-sweep confirmation retain noise
// rejection while making each radar step substantially faster.
constexpr uint8_t SCAN_SAMPLES_PER_ANGLE = 2;
// A terminal echo seen by both samples at an angle is committed as a confirmed
// obstacle when the complete 0 -> 180 -> 0 sweep is published.
constexpr uint8_t RADAR_OBSTACLE_MIN_VALID_SAMPLES = 2;
static_assert(RADAR_OBSTACLE_MIN_VALID_SAMPLES <= SCAN_SAMPLES_PER_ANGLE,
              "Radar obstacle confirmation cannot exceed samples per angle");
constexpr uint16_t MAX_RANGE_CM = 250;

// EEPROM
constexpr uint16_t EEPROM_MAGIC = 0xA65A;
// ESP32 EEPROM is flash emulation and must be initialized with its capacity.
constexpr uint16_t EEPROM_SIZE = MAP_W * MAP_H + 9;
// Rebuild once after flashing this firmware so a stale map from earlier test
// runs cannot mark the robot's startup corridor as permanently blocked.
constexpr uint8_t EEPROM_VERSION = 9;
