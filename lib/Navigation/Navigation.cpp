#include "Navigation.h"
#include "RobotConfig.h"
#include "SensorManager.h"
#include "ServoScanner.h"
#include "FlameServoScanner.h"
#include "ScanMapper.h"
#include "WebControl.h"
#include "MotorControl.h"
#include "FireControl.h"
#include "MapMemory.h"
#include "PowerManager.h"

#include <Arduino.h>
#include <string.h>

namespace {

constexpr uint8_t NO_HEADING = 0xFF;

Pose p = {START_X, START_Y, NORTH};
Navigation::Mode current = Navigation::EXPLORE;

// --------------------------------------------------------------------------
// Motion: no IMU. All turns/moves are timed.
// --------------------------------------------------------------------------
enum Motion {
  IDLE,
  AUTO_CRUISE,
  TURN_LEFT,
  TURN_RIGHT,
  TURN_BACK,
  BACK_CELL,      // NEW: backup when stuck
  AVOID_FORWARD   // NEW: escape from obstacle
};

enum AvoidPhase {
  AVOID_NONE,
  AVOID_ALIGNING_BACKTRACK,
  AVOID_BACKING,
  AVOID_TURNING,
  AVOID_ESCAPING
};

Motion motion = IDLE;
AvoidPhase avoidPhase = AVOID_NONE;
uint32_t motionStart = 0;
uint32_t lastMotionFinish = 0;
float turnStartYaw = 0.0f;
bool turnImuValid = false;
float forwardTargetYaw = 0.0f;
bool forwardImuValid = false;

// Recent cell-by-cell route. It is intentionally kept in RAM: after a reset
// the physical robot position is not known, so replaying a persisted route
// would be unsafe. During one run it lets avoidance reverse over real cells
// instead of guessing a new pose from a long blind backup.
constexpr uint16_t PATH_HISTORY_CAPACITY = 256;
Pose pathHistory[PATH_HISTORY_CAPACITY];
uint16_t pathHistoryCount = 0;
uint8_t avoidBacktrackRemaining = 0;
uint8_t avoidBacktrackReserve = 0;
uint8_t avoidBacktrackedCells = 0;
bool backingAlongHistory = false;
// Number of logical map cells left in the current straight free corridor.
// Pose/history still update at each cell boundary while the motors keep moving.
uint16_t cruiseCellsRemaining = 0;

// --------------------------------------------------------------------------
// Pure coverage navigation: S-pattern + BFS.
// --------------------------------------------------------------------------
bool coverageVisited[MAP_H][MAP_W];
bool searchSeen[MAP_H][MAP_W];
uint8_t searchFirstHeading[MAP_H][MAP_W];
uint16_t searchQueue[MAP_W * MAP_H];

Heading sweepHeading = EAST;
Heading laneHeading = NORTH;
uint8_t pendingHeading = NO_HEADING;

bool coverageComplete = false;

// --------------------------------------------------------------------------
// NEW: Background radar scan (continuous while moving)
// --------------------------------------------------------------------------
bool backgroundScanActive = false;
uint8_t scanCycle = 0;
Heading scanStartHeading = NORTH;
uint32_t lastScanTime = 0;
bool initialMapReady = false;
bool initialScanTurnActive = false;
uint32_t initialScanTurnStartMs = 0;
float initialScanTurnStartYaw = 0.0f;
bool initialScanTurnImuValid = false;
uint16_t forwardRadarDistanceCm = 0;
uint32_t forwardRadarMs = 0;
bool forwardRadarValid = false;
uint32_t lastSafetyLogMs = 0;

// --------------------------------------------------------------------------
// NEW: Local 3x3 obstacle map for SA2 avoidance
// --------------------------------------------------------------------------
enum LocalCellState {
  LC_UNKNOWN = 0,
  LC_FREE = 1,
  LC_BLOCKED = 2
};

LocalCellState localMap[3][3];  // [row][col], robot at [1][1]
uint32_t localMapLastUpdate = 0;
constexpr uint32_t LOCAL_MAP_FORGET_MS = 6000;  // Forget after 6s

// --------------------------------------------------------------------------
// NEW: Stuck detection (SA2 distance not changing)
// --------------------------------------------------------------------------
uint16_t sa2History[5];  // Last 5 SA2 readings
uint8_t sa2HistoryIndex = 0;
uint32_t stuckStartTime = 0;
constexpr uint16_t STUCK_TOLERANCE_CM = 5;  // 5cm tolerance
constexpr uint8_t STUCK_READINGS = 5;       // 5 readings
constexpr uint32_t STUCK_TIMEOUT_MS = 3000; // If stuck for 3s, backup

bool controlWasManual = false;
uint8_t consecutiveTurnsWithoutForward = 0;
bool waitingForStationarySweep = false;
uint8_t sa2CloseSampleCount = 0;
uint32_t lastSa2DecisionSampleMs = 0;
float fireEntryYaw = 0.0f;
bool fireEntryImuValid = false;
bool fireReturnActive = false;
uint32_t fireReturnStartMs = 0;

// Forward declarations used by the local avoidance selector.
bool inside(int16_t x, int16_t y);
void advance(int16_t& x, int16_t& y, Heading h);
void requestStationarySweep(const __FlashStringHelper* reason);

bool imuReadingFresh(const SensorSnapshot& s) {
  return MPU6050_ENABLED && s.imuValid && s.imuMs > 0 &&
    millis() - s.imuMs <= MPU6050_FRESH_MS;
}

float yawProgressDeg(float startYaw, const SensorSnapshot& s) {
  return fabsf(s.yawDeg - startYaw);
}

void captureTurnYaw() {
  const SensorSnapshot s = SensorManager::get();
  turnStartYaw = s.yawDeg;
  turnImuValid = imuReadingFresh(s);
}

void captureForwardYaw() {
  const SensorSnapshot s = SensorManager::get();
  forwardTargetYaw = s.yawDeg;
  forwardImuValid = imuReadingFresh(s);
}

void maintainForwardHeading(const SensorSnapshot& s) {
  const uint8_t basePwm = current == Navigation::PATROL
    ? PWM_PATROL : PWM_EXPLORE;
  if (!forwardImuValid || !imuReadingFresh(s)) {
    MotorControl::forward(basePwm);
    return;
  }

  float errorDeg = s.yawDeg - forwardTargetYaw;
  if (fabsf(errorDeg) <= MPU_HEADING_HOLD_DEADBAND_DEG) errorDeg = 0.0f;
  const int correction = constrain(
    (int)lroundf(errorDeg * MPU_HEADING_HOLD_KP),
    -(int)MPU_HEADING_HOLD_MAX_PWM,
    (int)MPU_HEADING_HOLD_MAX_PWM);
  MotorControl::forwardCorrected(basePwm, correction);
}

// --------------------------------------------------------------------------
// Local map utilities
// --------------------------------------------------------------------------
void resetLocalMap() {
  memset(localMap, LC_UNKNOWN, sizeof(localMap));
  localMapLastUpdate = millis();
  Serial.println(F("[LOCALMAP] RESET"));
}

void initLocalMap() {
  resetLocalMap();
}

void markLocalObstacle(Heading heading) {
  // Map heading to local coordinates
  // Robot at [1][1], heading determines relative position
  int8_t dx = 0, dy = 0;
  if (heading == NORTH) dy = -1;
  else if (heading == SOUTH) dy = 1;
  else if (heading == EAST) dx = 1;
  else if (heading == WEST) dx = -1;
  
  int8_t localX = 1 + dx;
  int8_t localY = 1 + dy;
  
  if (localX >= 0 && localX < 3 && localY >= 0 && localY < 3) {
    localMap[localY][localX] = LC_BLOCKED;
    localMapLastUpdate = millis();
    
    Serial.print(F("[LOCALMAP] BLOCKED "));
    Serial.print((int)heading);
    Serial.print(F(" at ["));
    Serial.print(localX);
    Serial.print(F(","));
    Serial.print(localY);
    Serial.println(F("]"));
  }
}

void markLocalFree(Heading heading) {
  int8_t dx = 0, dy = 0;
  if (heading == NORTH) dy = -1;
  else if (heading == SOUTH) dy = 1;
  else if (heading == EAST) dx = 1;
  else if (heading == WEST) dx = -1;
  
  int8_t localX = 1 + dx;
  int8_t localY = 1 + dy;
  
  if (localX >= 0 && localX < 3 && localY >= 0 && localY < 3) {
    localMap[localY][localX] = LC_FREE;
    localMapLastUpdate = millis();
  }
}

bool isLocalFree(Heading heading) {
  int8_t dx = 0, dy = 0;
  if (heading == NORTH) dy = -1;
  else if (heading == SOUTH) dy = 1;
  else if (heading == EAST) dx = 1;
  else if (heading == WEST) dx = -1;
  
  int8_t localX = 1 + dx;
  int8_t localY = 1 + dy;
  
  if (localX < 0 || localX >= 3 || localY < 0 || localY >= 3) {
    return false;  // Out of bounds
  }
  
  return localMap[localY][localX] == LC_FREE;
}

bool isLocalBlocked(Heading heading) {
  int8_t dx = 0, dy = 0;
  if (heading == NORTH) dy = -1;
  else if (heading == SOUTH) dy = 1;
  else if (heading == EAST) dx = 1;
  else if (heading == WEST) dx = -1;
  
  int8_t localX = 1 + dx;
  int8_t localY = 1 + dy;
  
  if (localX < 0 || localX >= 3 || localY < 0 || localY >= 3) {
    return true;  // Out of bounds = blocked
  }
  
  return localMap[localY][localX] == LC_BLOCKED;
}

int16_t scoreAvoidDirection(Heading heading) {
  if (isLocalBlocked(heading)) return -1;

  int16_t x = p.x;
  int16_t y = p.y;
  int16_t score = 0;
  uint8_t openCells = 0;

  for (uint8_t step = 0; step < AVOID_ROUTE_LOOKAHEAD_CELLS; ++step) {
    advance(x, y, heading);
    if (!inside(x, y) || MapMemory::isBlocked(x, y)) break;

    ++openCells;
    score += MapMemory::get(x, y) == CELL_FREE ? 3 : 2;
    if (!coverageVisited[y][x]) score += 3;
  }

  if (openCells == 0) return -1;

  // Prefer a new side route over continuing backward on the already covered
  // path when both routes have a similar amount of clearance.
  if (heading == (Heading)((p.heading + 2) % 4)) {
    score = score > 4 ? score - 4 : 0;
  }
  return score;
}

uint8_t chooseAvoidDirection() {
  const Heading candidates[3] = {
    (Heading)((p.heading + 3) % 4),
    (Heading)((p.heading + 1) % 4),
    (Heading)((p.heading + 2) % 4)
  };

  int16_t bestScore = -1;
  uint8_t bestHeading = NO_HEADING;
  for (uint8_t i = 0; i < 3; ++i) {
    const int16_t score = scoreAvoidDirection(candidates[i]);
    Serial.print(F("[NAV] ESCAPE EVAL heading="));
    Serial.print((int)candidates[i]);
    Serial.print(F(" score="));
    Serial.println(score);

    if (score > bestScore) {
      bestScore = score;
      bestHeading = (uint8_t)candidates[i];
    }
  }
  return bestHeading;
}

// --------------------------------------------------------------------------
// Stuck detection
// --------------------------------------------------------------------------
void updateSA2History(uint16_t distance) {
  sa2History[sa2HistoryIndex] = distance;
  sa2HistoryIndex = (sa2HistoryIndex + 1) % STUCK_READINGS;
}

bool isStuck() {
  if (sa2History[0] == 0) return false;  // Not enough data
  
  uint16_t minDist = sa2History[0];
  uint16_t maxDist = sa2History[0];
  
  for (uint8_t i = 1; i < STUCK_READINGS; ++i) {
    if (sa2History[i] < minDist) minDist = sa2History[i];
    if (sa2History[i] > maxDist) maxDist = sa2History[i];
  }
  
  return (maxDist - minDist) < STUCK_TOLERANCE_CM;
}

// --------------------------------------------------------------------------
// Navigation utilities
// --------------------------------------------------------------------------
bool inside(int16_t x, int16_t y) {
  return x >= 0 && x < MAP_W && y >= 0 && y < MAP_H;
}

Heading backOf(Heading h) {
  return (Heading)((h + 2) % 4);
}

void advance(int16_t& x, int16_t& y, Heading h) {
  if (h == NORTH) --y;
  else if (h == SOUTH) ++y;
  else if (h == EAST) ++x;
  else --x;
}

bool sameCell(const Pose& a, const Pose& b) {
  return a.x == b.x && a.y == b.y;
}

void resetTravelPath() {
  pathHistoryCount = 1;
  pathHistory[0] = p;
}

void recordTravelPose() {
  if (pathHistoryCount > 0 && sameCell(pathHistory[pathHistoryCount - 1], p))
    return;

  if (pathHistoryCount >= PATH_HISTORY_CAPACITY) {
    memmove(&pathHistory[0], &pathHistory[1],
            sizeof(pathHistory[0]) * (PATH_HISTORY_CAPACITY - 1));
    pathHistoryCount = PATH_HISTORY_CAPACITY - 1;
  }
  pathHistory[pathHistoryCount++] = p;
}

uint8_t availableBacktrackCells() {
  if (pathHistoryCount < 2 ||
      !sameCell(pathHistory[pathHistoryCount - 1], p)) return 0;

  // Validate every recorded edge. This permits a 5-10 cell retrace even when
  // the recorded path contains one or more corners.
  uint8_t available = 0;
  for (int32_t i = (int32_t)pathHistoryCount - 1;
       i > 0 && available < AVOID_BACKTRACK_MAX_CELLS; --i) {
    int16_t expectedX = pathHistory[i - 1].x;
    int16_t expectedY = pathHistory[i - 1].y;
    advance(expectedX, expectedY, pathHistory[i].heading);
    if (expectedX != pathHistory[i].x || expectedY != pathHistory[i].y) break;
    ++available;
  }
  return available;
}

bool restorePreviousTravelCell() {
  if (pathHistoryCount < 2) return false;

  int16_t expectedX = p.x;
  int16_t expectedY = p.y;
  advance(expectedX, expectedY, backOf(p.heading));
  const Pose& previous = pathHistory[pathHistoryCount - 2];
  if (previous.x != expectedX || previous.y != expectedY) return false;

  --pathHistoryCount;
  p.x = previous.x;
  p.y = previous.y;
  // Keep the chassis heading: the robot physically drove backward.
  return true;
}

bool cellIsUncovered(Heading h) {
  int16_t x = p.x;
  int16_t y = p.y;
  advance(x, y, h);

  return inside(x, y) &&
         MapMemory::get(x, y) == CELL_FREE &&
         !coverageVisited[y][x];
}

uint16_t freeCorridorLength(Heading h, uint16_t& unvisitedCells) {
  int16_t x = p.x;
  int16_t y = p.y;
  uint16_t length = 0;
  unvisitedCells = 0;

  for (uint16_t step = 0; step < MAP_W + MAP_H; ++step) {
    advance(x, y, h);
    if (!inside(x, y) || MapMemory::isBlocked(x, y)) break;
    if (MapMemory::get(x, y) != CELL_FREE) break;

    ++length;
    if (!coverageVisited[y][x]) ++unvisitedCells;
  }

  return length;
}

void markCurrentVisited() {
  if (!inside(p.x, p.y)) return;
  coverageVisited[p.y][p.x] = true;
  MapMemory::set(p.x, p.y, CELL_FREE);
}

void resetCoverage() {
  memset(coverageVisited, 0, sizeof(coverageVisited));
  memset(searchSeen, 0, sizeof(searchSeen));
  memset(searchFirstHeading, NO_HEADING, sizeof(searchFirstHeading));

  pendingHeading = NO_HEADING;
  cruiseCellsRemaining = 0;
  coverageComplete = false;
  // Begin each coverage pass in the direction the chassis is already facing.
  // This avoids an unnecessary 90-degree turn when the path ahead is clear.
  sweepHeading = p.heading;
  laneHeading = (Heading)((p.heading + 1) % 4);

  markCurrentVisited();
}

// --------------------------------------------------------------------------
// BFS fallback
// --------------------------------------------------------------------------
uint8_t findNearestUncoveredHeading() {
  memset(searchSeen, 0, sizeof(searchSeen));
  memset(searchFirstHeading, NO_HEADING, sizeof(searchFirstHeading));

  if (!inside(p.x, p.y)) return NO_HEADING;

  const uint16_t start = (uint16_t)(p.y * MAP_W + p.x);
  uint16_t head = 0;
  uint16_t tail = 0;

  searchQueue[tail++] = start;
  searchSeen[p.y][p.x] = true;

  const Heading order[4] = {
    sweepHeading,
    laneHeading,
    backOf(sweepHeading),
    backOf(laneHeading)
  };

  while (head < tail) {
    const uint16_t packed = searchQueue[head++];
    const int16_t x = packed % MAP_W;
    const int16_t y = packed / MAP_W;

    for (uint8_t i = 0; i < 4; ++i) {
      const Heading h = order[i];
      int16_t nx = x;
      int16_t ny = y;
      advance(nx, ny, h);

      if (!inside(nx, ny)) continue;
      if (searchSeen[ny][nx]) continue;
      // Coverage is a route over radar-confirmed free cells only. Unknown
      // cells are explored by the radar and become eligible after a sweep
      // commit; they are never entered speculatively by the planner.
      if (MapMemory::get(nx, ny) != CELL_FREE) continue;

      searchSeen[ny][nx] = true;

      const uint8_t firstHeading =
        (packed == start)
          ? (uint8_t)h
          : searchFirstHeading[y][x];

      searchFirstHeading[ny][nx] = firstHeading;

      if (!coverageVisited[ny][nx]) {
        return firstHeading;
      }

      if (tail < MAP_W * MAP_H) {
        searchQueue[tail++] = (uint16_t)(ny * MAP_W + nx);
      }
    }
  }

  return NO_HEADING;
}

// --------------------------------------------------------------------------
// Coverage decision
// --------------------------------------------------------------------------
uint8_t chooseCoverageHeading() {
  // Prefer the longest uninterrupted corridor already confirmed free by SRF04.
  // Among equal-length corridors, prefer one containing more unvisited cells.
  const Heading routeOrder[4] = {
    sweepHeading, laneHeading, backOf(sweepHeading), backOf(laneHeading)
  };
  uint8_t bestHeading = NO_HEADING;
  uint16_t bestLength = 0;
  uint16_t bestUnvisited = 0;

  for (uint8_t i = 0; i < 4; ++i) {
    uint16_t unvisited = 0;
    const uint16_t length = freeCorridorLength(routeOrder[i], unvisited);
    if (unvisited > 0 &&
        (length > bestLength ||
         (length == bestLength && unvisited > bestUnvisited))) {
      bestHeading = (uint8_t)routeOrder[i];
      bestLength = length;
      bestUnvisited = unvisited;
    }
  }
  if (bestHeading != NO_HEADING) return bestHeading;

  // No known-free corridor remains; use the S-pattern, then BFS to reach every
  // radar-confirmed free cell that is still unvisited.
  if (cellIsUncovered(sweepHeading)) return sweepHeading;

  if (cellIsUncovered(laneHeading)) {
    sweepHeading = backOf(sweepHeading);
    return laneHeading;
  }

  if (cellIsUncovered(backOf(laneHeading))) {
    laneHeading = backOf(laneHeading);
    sweepHeading = backOf(sweepHeading);
    return laneHeading;
  }

  return findNearestUncoveredHeading();
}

// --------------------------------------------------------------------------
// Motor motion
// --------------------------------------------------------------------------
void startForward() {
  if (cruiseCellsRemaining == 0) {
    uint16_t unvisitedCells = 0;
    cruiseCellsRemaining = freeCorridorLength(p.heading, unvisitedCells);
    // Unknown space is explored conservatively one cell at a time. Cells that
    // are already confirmed free are merged into one uninterrupted motor run.
    if (cruiseCellsRemaining == 0) {
      Serial.println(F("[NAV] NO CONFIRMED FREE CORRIDOR; WAIT FOR RADAR SWEEP"));
      motion = IDLE;
      pendingHeading = NO_HEADING;
      return;
    }
  }
  motion = AUTO_CRUISE;
  motionStart = millis();
  captureForwardYaw();
  MotorControl::forward(
    current == Navigation::PATROL ? PWM_PATROL : PWM_EXPLORE
  );

  Serial.print(F("[NAV] FREE CORRIDOR cells="));
  Serial.print(cruiseCellsRemaining);
  Serial.print(F(" PWM="));
  Serial.println(current == Navigation::PATROL ? PWM_PATROL : PWM_EXPLORE);
}

void startBack() {
  motion = BACK_CELL;
  motionStart = millis();
  MotorControl::backward(
    current == Navigation::PATROL ? PWM_PATROL : PWM_EXPLORE
  );
  Serial.println(F("[NAV] BACK UP (STUCK)"));
}

void startLeft() {
  motion = TURN_LEFT;
  motionStart = millis();
  captureTurnYaw();
  MotorControl::left(PWM_TURN);
  Serial.printf("[NAV] TURN LEFT 90 IMU=%d\n", turnImuValid ? 1 : 0);
}

void startRight() {
  motion = TURN_RIGHT;
  motionStart = millis();
  captureTurnYaw();
  MotorControl::right(PWM_TURN);
  Serial.printf("[NAV] TURN RIGHT 90 IMU=%d\n", turnImuValid ? 1 : 0);
}

void startTurnBack() {
  motion = TURN_BACK;
  motionStart = millis();
  captureTurnYaw();
  MotorControl::right(PWM_TURN);
  Serial.printf("[NAV] TURN BACK 180 IMU=%d\n", turnImuValid ? 1 : 0);
}

void startAvoidForward() {
  avoidPhase = AVOID_ESCAPING;
  motion = AVOID_FORWARD;
  motionStart = millis();
  captureForwardYaw();
  MotorControl::forward(PWM_EXPLORE);
  Serial.println(F("[NAV] AVOID FORWARD (escape)"));
}

void startAvoidBack() {
  avoidPhase = AVOID_BACKING;
  backingAlongHistory = avoidBacktrackRemaining > 0;
  motion = BACK_CELL;
  motionStart = millis();
  MotorControl::backward(current == Navigation::PATROL ? PWM_PATROL : PWM_EXPLORE);
  Serial.println(backingAlongHistory
    ? F("[NAV] AVOID: RETRACE PREVIOUS CELL")
    : F("[NAV] AVOID: CONTROLLED BACKUP"));
}

void turnToward(Heading target) {
  const uint8_t delta = (uint8_t)((target + 4 - p.heading) % 4);

  if (delta == 1) startRight();
  else if (delta == 3) startLeft();
  else if (delta == 2) startTurnBack();
}

void continueAvoidBacktrack() {
  if (avoidBacktrackRemaining == 0 || pathHistoryCount == 0) return;

  // A stored heading is the direction used to enter that cell. Face it, then
  // drive backward over the same physical edge. Repeat after a recorded corner.
  const Heading retraceHeading = pathHistory[pathHistoryCount - 1].heading;
  if (p.heading != retraceHeading) {
    avoidPhase = AVOID_ALIGNING_BACKTRACK;
    turnToward(retraceHeading);
  } else {
    startAvoidBack();
  }
}

bool startBestAvoidEscape() {
  const uint8_t avoidHeading = chooseAvoidDirection();
  if (avoidHeading == NO_HEADING) return false;

  Serial.print(F("[NAV] ESCAPE SELECT heading="));
  Serial.println(avoidHeading);
  avoidPhase = AVOID_TURNING;
  if ((Heading)avoidHeading == p.heading) startAvoidForward();
  else turnToward((Heading)avoidHeading);
  return true;
}

// --------------------------------------------------------------------------
// Motion finish
// --------------------------------------------------------------------------
void finishMotion() {
  const Motion completedMotion = motion;
  MotorControl::stop();

  if (completedMotion == BACK_CELL) {
    // Prefer the recorded route. Only fall back to dead reckoning when there
    // is no matching previous cell (for example immediately after boot).
    const bool restored = backingAlongHistory && restorePreviousTravelCell();
    if (!restored) {
      Heading back = backOf(p.heading);
      int16_t nextX = p.x;
      int16_t nextY = p.y;
      advance(nextX, nextY, back);
      if (inside(nextX, nextY) && !MapMemory::isBlocked(nextX, nextY)) {
        p.x = nextX;
        p.y = nextY;
      }
    }
    backingAlongHistory = false;
  } else if (completedMotion == TURN_LEFT) {
    MapMemory::rotateLeft(p);
    if (consecutiveTurnsWithoutForward < 255) ++consecutiveTurnsWithoutForward;
  } else if (completedMotion == TURN_RIGHT) {
    MapMemory::rotateRight(p);
    if (consecutiveTurnsWithoutForward < 255) ++consecutiveTurnsWithoutForward;
  } else if (completedMotion == TURN_BACK) {
    MapMemory::rotateBack(p);
    if (consecutiveTurnsWithoutForward < 255) ++consecutiveTurnsWithoutForward;
  } else if (completedMotion == AVOID_FORWARD) {
    // Avoid: move forward
    MapMemory::updatePoseForward(p);
    markCurrentVisited();
    recordTravelPose();
    consecutiveTurnsWithoutForward = 0;
  }

  motion = IDLE;
  lastMotionFinish = millis();

  if (avoidPhase == AVOID_ALIGNING_BACKTRACK &&
      (completedMotion == TURN_LEFT || completedMotion == TURN_RIGHT ||
       completedMotion == TURN_BACK)) {
    startAvoidBack();
  } else if (avoidPhase == AVOID_BACKING && completedMotion == BACK_CELL) {
    ++avoidBacktrackedCells;
    if (avoidBacktrackRemaining > 0) --avoidBacktrackRemaining;
    if (avoidBacktrackRemaining > 0) {
      continueAvoidBacktrack();
      return;
    }

    // The 3x3 local map belongs to the old pose. After a long retrace, assess
    // the new pose from the persistent map instead of using stale local cells.
    resetLocalMap();
    Serial.print(F("[NAV] BACKTRACKED cells="));
    Serial.println(avoidBacktrackedCells);

    if (startBestAvoidEscape()) return;

    // No side is open after the first five cells. Retreat one more verified
    // cell and score the routes again, stopping after ten total cells.
    if (avoidBacktrackReserve > 0) {
      --avoidBacktrackReserve;
      avoidBacktrackRemaining = 1;
      Serial.println(F("[NAV] NO ESCAPE; EXTEND BACKTRACK BY 1 CELL"));
      continueAvoidBacktrack();
      return;
    }

    avoidPhase = AVOID_NONE;
    pendingHeading = NO_HEADING;
    Serial.println(F("[NAV] NO SAFE ESCAPE AFTER MAX BACKTRACK"));
  } else if (avoidPhase == AVOID_TURNING &&
             (completedMotion == TURN_LEFT || completedMotion == TURN_RIGHT ||
              completedMotion == TURN_BACK)) {
    startAvoidForward();
  } else if (avoidPhase == AVOID_ESCAPING && completedMotion == AVOID_FORWARD) {
    avoidPhase = AVOID_NONE;
    avoidBacktrackReserve = 0;
    pendingHeading = NO_HEADING;
    cruiseCellsRemaining = 0;
    // Preserve coverageVisited so avoidance does not make the robot cover all
    // previously completed cells again.
    requestStationarySweep(F("after obstacle avoidance"));
  }

  Serial.print(F("[NAV] POSE x="));
  Serial.print(p.x);
  Serial.print(F(" y="));
  Serial.print(p.y);
  Serial.print(F(" heading="));
  Serial.println((int)p.heading);
}

// --------------------------------------------------------------------------
// Background radar scan (continuous while moving)
// --------------------------------------------------------------------------
void startBackgroundScan() {
  backgroundScanActive = true;
  scanStartHeading = p.heading;
  lastScanTime = millis();

  SensorManager::setUltrasonicScanMode(true);
  ScanMapper::beginSweep();
  ServoScanner::startScan();

  Serial.println(F("[RADAR] BACKGROUND SCAN START (CONTINUOUS)"));
}

void requestStationarySweep(const __FlashStringHelper* reason) {
  MotorControl::stop();
  motion = IDLE;
  pendingHeading = NO_HEADING;
  cruiseCellsRemaining = 0;
  waitingForStationarySweep = true;
  backgroundScanActive = true;
  scanStartHeading = p.heading;
  lastScanTime = millis();

  SensorManager::setUltrasonicScanMode(true);
  ServoScanner::stop();
  ScanMapper::beginSweep();
  ServoScanner::startScan();

  Serial.print(F("[RADAR] STATIONARY SWEEP REQUIRED: "));
  Serial.println(reason);
}

void runBackgroundScan() {
  ServoScanner::update();

  if (!ServoScanner::measurementDue()) return;

  uint32_t sum = 0;
  uint8_t validSamples = 0;

  for (uint8_t i = 0; i < SCAN_SAMPLES_PER_ANGLE; ++i) {
    const uint16_t distance = SensorManager::readUltrasonicNow();

    if (distance > 0) {
      sum += distance;
      ++validSamples;
    }

    delay(5);
  }

  const uint16_t distanceCm =
    validSamples > 0 ? (uint16_t)(sum / validSamples) : 0;

  const float worldAngleDeg =
    (float)p.heading * 90.0f +
    (float)ServoScanner::angle() -
    (float)SERVO_SCAN_CENTER_DEG;

  const int16_t relativeAngle =
    (int16_t)ServoScanner::angle() - (int16_t)SERVO_SCAN_CENTER_DEG;
  if (relativeAngle >= -(int16_t)RADAR_FORWARD_CONE_DEG &&
      relativeAngle <= (int16_t)RADAR_FORWARD_CONE_DEG) {
    forwardRadarDistanceCm = distanceCm;
    forwardRadarValid = validSamples > 0 && distanceCm > 0;
    forwardRadarMs = millis();
  }

  // With no IMU/encoders, only a stopped chassis has a map-worthy pose.
  const bool headingStable =
    motion != TURN_LEFT && motion != TURN_RIGHT && motion != TURN_BACK &&
    !initialScanTurnActive;
  const bool integrateIntoMap = RADAR_MAP_ONLY_WHEN_STOPPED
    ? (motion == IDLE && !initialScanTurnActive)
    : headingStable;

  ScanMapper::addReading(
    p,
    worldAngleDeg,
    distanceCm,
    validSamples,
    SCAN_SAMPLES_PER_ANGLE,
    integrateIntoMap
  );

  ServoScanner::measurementTaken();

  if (ServoScanner::cycleFinished()) {
    ScanMapper::finishSweep(p, (uint8_t)(scanCycle + 1));
    scanCycle++;
    lastScanTime = millis();
    // The sweep has just published new FREE/OBSTACLE cells. Re-run coverage
    // selection so newly discovered free cells are eventually visited.
    coverageComplete = false;
    if (motion == IDLE) pendingHeading = NO_HEADING;

    if (waitingForStationarySweep) {
      waitingForStationarySweep = false;
      consecutiveTurnsWithoutForward = 0;
      Serial.println(F("[RADAR] STATIONARY MAP READY; NAVIGATION RELEASED"));
    }

    if (!initialMapReady && scanCycle < SCAN_CYCLES_PER_POSE) {
      backgroundScanActive = false;
      initialScanTurnActive = true;
      initialScanTurnStartMs = millis();
      const SensorSnapshot imu = SensorManager::get();
      initialScanTurnStartYaw = imu.yawDeg;
      initialScanTurnImuValid = imuReadingFresh(imu);
      MotorControl::right(PWM_TURN);
      Serial.printf("[RADAR] INITIAL BODY TURN 180 START IMU=%d\n",
                    initialScanTurnImuValid ? 1 : 0);
      return;
    }

    if (!initialMapReady) {
      initialMapReady = true;
      // The chassis is now facing the rear-scan heading. Start the coverage
      // lanes from this real heading instead of turning 180 degrees back to
      // the heading stored before the initial mapping sequence.
      sweepHeading = p.heading;
      laneHeading = (Heading)((p.heading + 1) % 4);
      pendingHeading = NO_HEADING;
      Serial.println(F("[RADAR] FRONT+REAR INITIAL MAP READY"));
    }
    
    Serial.print(F("[RADAR] CYCLE "));
    Serial.print(scanCycle);
    Serial.println(F(" COMPLETE"));
    
    // Start next scan immediately
    ServoScanner::startScan();
    ScanMapper::beginSweep();
  }
}

// --------------------------------------------------------------------------
// SA2 obstacle avoidance with local 3x3 map
// --------------------------------------------------------------------------
bool sa2ReadingFresh(const SensorSnapshot& s) {
  return s.sa2Valid &&
    millis() - s.sa2Ms <= 2 * ULTRASONIC_PERIOD_MS + 100;
}

bool sa2ObstacleConfirmed(const SensorSnapshot& s) {
  if (!sa2ReadingFresh(s) || s.sa2DistanceCm <= 1) {
    sa2CloseSampleCount = 0;
    return false;
  }

  if (s.sa2Ms != lastSa2DecisionSampleMs) {
    lastSa2DecisionSampleMs = s.sa2Ms;
    if (s.sa2DistanceCm <= SA2_OBSTACLE_DISTANCE_CM) {
      if (sa2CloseSampleCount < SA2_OBSTACLE_CONFIRM_SAMPLES)
        ++sa2CloseSampleCount;
    } else {
      sa2CloseSampleCount = 0;
    }
  }

  return s.sa2DistanceCm <= SA2_CRITICAL_STOP_CM ||
    sa2CloseSampleCount >= SA2_OBSTACLE_CONFIRM_SAMPLES;
}

bool forwardRadarBlocked() {
  return forwardRadarValid &&
    millis() - forwardRadarMs <= RADAR_FORWARD_FRESH_MS &&
    forwardRadarDistanceCm > 1 &&
    forwardRadarDistanceCm <= RADAR_EMERGENCY_STOP_CM;
}

void handleForwardObstacle(uint16_t distanceCm, const __FlashStringHelper* source) {
  if (avoidPhase == AVOID_ESCAPING) avoidPhase = AVOID_NONE;
  if (avoidPhase != AVOID_NONE) return;

  MotorControl::stop();
  motion = IDLE;
  cruiseCellsRemaining = 0;

  // Mark obstacle in local map
  markLocalObstacle(p.heading);
  
  // Mark in global map
  MapMemory::markObstacleAhead(p, distanceCm);
  int16_t x = p.x;
  int16_t y = p.y;
  advance(x, y, p.heading);
  if (inside(x, y)) {
    MapMemory::set(x, y, CELL_OBSTACLE);
  }

  Serial.print(F("["));
  Serial.print(source);
  Serial.print(F("] OBSTACLE AT "));
  Serial.print(distanceCm);
  Serial.println(F("cm"));

  pendingHeading = NO_HEADING;

  // First retrace five verified cells (or all available history when shorter).
  // Keep up to five more verified cells in reserve if route evaluation at the
  // first position finds no safe escape.
  const uint8_t availableBacktrack = availableBacktrackCells();
  avoidBacktrackRemaining = availableBacktrack > AVOID_BACKTRACK_MIN_CELLS
    ? AVOID_BACKTRACK_MIN_CELLS : availableBacktrack;
  avoidBacktrackReserve = availableBacktrack - avoidBacktrackRemaining;
  avoidBacktrackedCells = 0;

  Serial.print(F("[NAV] BACKTRACK PLAN initial="));
  Serial.print(avoidBacktrackRemaining);
  Serial.print(F(" reserve="));
  Serial.println(avoidBacktrackReserve);

  if (avoidBacktrackRemaining == 0) {
    // There is no verified cell behind us (typically at startup). Do not make
    // a blind reverse; turn toward a locally unblocked direction instead.
    if (!startBestAvoidEscape()) {
      avoidPhase = AVOID_NONE;
      Serial.println(F("[NAV] NO VERIFIED BACKTRACK OR SAFE ESCAPE"));
    }
  } else {
    continueAvoidBacktrack();
  }
}

// --------------------------------------------------------------------------
// Stuck recovery
// --------------------------------------------------------------------------
void handleStuck() {
  MotorControl::stop();
  motion = IDLE;
  cruiseCellsRemaining = 0;
  
  Serial.println(F("[STUCK] BACKING UP AND REPLANNING"));
  
  // Backup one cell
  startBack();
  
  // Reset stuck detection
  memset(sa2History, 0, sizeof(sa2History));
  sa2HistoryIndex = 0;
  stuckStartTime = 0;
  
  // Replan after backup
  pendingHeading = NO_HEADING;
}

// --------------------------------------------------------------------------
// Main coverage logic
// --------------------------------------------------------------------------
void runCoverage(const SensorSnapshot& s) {
  if (waitingForStationarySweep) {
    MotorControl::stop();
    return;
  }

  // Forget old local map
  if (millis() - localMapLastUpdate > LOCAL_MAP_FORGET_MS) {
    resetLocalMap();
  }

  if (avoidPhase == AVOID_NONE && forwardRadarBlocked()) {
    handleForwardObstacle(forwardRadarDistanceCm, F("RADAR"));
    return;
  }

  // Only a fresh, valid close echo is an obstacle. With the configured
  // no-echo policy, radar/map protection remains active if SA2 has no return.
  const bool sa2Fresh = sa2ReadingFresh(s);
  if (!sa2Fresh) {
    if (millis() - lastSafetyLogMs >= 1000) {
      lastSafetyLogMs = millis();
      Serial.println(SA2_STOP_ON_INVALID_READING
        ? F("[SAFETY] SA2 invalid/stale; motors stopped")
        : F("[SA2] no valid echo; continuing with radar/map protection"));
    }
    if (SA2_STOP_ON_INVALID_READING) {
      MotorControl::stop();
      return;
    }
  }

  if (avoidPhase == AVOID_NONE && sa2ObstacleConfirmed(s)) {
    handleForwardObstacle(s.sa2DistanceCm, F("SA2"));
    return;
  }
  
  // Reset stuck if no obstacle
  if (sa2Fresh && s.sa2DistanceCm > SA2_OBSTACLE_DISTANCE_CM) {
    stuckStartTime = 0;
    markLocalFree(p.heading);

    // A fresh SA2 return beyond the stop distance proves that the immediately
    // adjacent cell is clear. Repair a stale persisted obstacle at that cell
    // so an old EEPROM map cannot keep the robot stationary after restart.
    int16_t clearX = p.x;
    int16_t clearY = p.y;
    advance(clearX, clearY, p.heading);
    if (inside(clearX, clearY)) {
      MapMemory::set(clearX, clearY, CELL_FREE);
    }
  }

  // Decide next heading
  if (pendingHeading == NO_HEADING) {
    pendingHeading = chooseCoverageHeading();

    if (pendingHeading == NO_HEADING) {
      coverageComplete = true;
      MapMemory::save();
      MotorControl::stop();
      Serial.println(F("[NAV] COVERAGE COMPLETE"));
      PowerManager::enterCoverageSleep();
      return;
    }
  }

  const Heading target = (Heading)pendingHeading;

  if (p.heading != target) {
    if (consecutiveTurnsWithoutForward >= MAX_TURNS_WITHOUT_FORWARD) {
      requestStationarySweep(F("turn-loop guard"));
      return;
    }
    turnToward(target);
    return;
  }

  // Check if next cell is blocked in global map
  int16_t nextX = p.x;
  int16_t nextY = p.y;
  advance(nextX, nextY, target);

  if (!inside(nextX, nextY) ||
      MapMemory::get(nextX, nextY) != CELL_FREE) {
    Serial.println(F("[NAV] NEXT CELL NOT CONFIRMED FREE -> WAIT/REPLAN"));
    pendingHeading = NO_HEADING;
    cruiseCellsRemaining = 0;
    return;
  }

  // Move forward
  startForward();
}

} // namespace

namespace Navigation {

void begin() {
  p = {START_X, START_Y, NORTH};
  current = MapMemory::isLoaded() ? PATROL : EXPLORE;

  motion = IDLE;
  motionStart = millis();
  lastMotionFinish = millis();
  turnImuValid = false;
  forwardImuValid = false;
  initialScanTurnImuValid = false;

  backgroundScanActive = false;
  scanCycle = 0;
  controlWasManual = false;
  // Establish the first map from two stationary sweeps (front, then rear)
  // before dead-reckoned motion can change the assumed pose.
  initialMapReady = false;
  initialScanTurnActive = false;
  consecutiveTurnsWithoutForward = 0;
  waitingForStationarySweep = false;
  sa2CloseSampleCount = 0;
  lastSa2DecisionSampleMs = 0;
  fireEntryImuValid = false;
  fireReturnActive = false;
  fireReturnStartMs = 0;
  avoidBacktrackRemaining = 0;
  avoidBacktrackReserve = 0;
  avoidBacktrackedCells = 0;
  backingAlongHistory = false;
  forwardRadarValid = false;
  forwardRadarMs = 0;

  resetCoverage();
  resetTravelPath();
  initLocalMap();
  ServoScanner::begin();

  Serial.println(F("[NAV] CONTINUOUS NAVIGATION"));
  Serial.println(F("[NAV] STATIONARY RADAR MAP + SA2 3x3 AVOIDANCE"));
  Serial.println(F("[NAV] WAITING FOR FRONT+REAR INITIAL MAP"));
}

void firstRunRoomScan() {
  // Navigation remains stopped until front and rear stationary sweeps finish.
  startBackgroundScan();
}

void update() {
  const SensorSnapshot s = SensorManager::get();

  // In MQ-2 standby the chassis must remain completely still. Once gas is
  // confirmed PowerManager clears this gate and normal patrol/fire logic
  // resumes on the next navigation tick.
  if (PowerManager::sleeping()) {
    MotorControl::stop();
    MotorControl::setPump(false);
    motion = IDLE;
    avoidPhase = AVOID_NONE;
    pendingHeading = NO_HEADING;
    cruiseCellsRemaining = 0;
    return;
  }

  if (!s.ready) {
    MotorControl::stop();
    MotorControl::setPump(false);
    motion = IDLE;
    avoidPhase = AVOID_NONE;
    pendingHeading = NO_HEADING;
    cruiseCellsRemaining = 0;
    return;
  }

  // Fire has priority
  if (s.fireDetected) {
    if (current != FIRE) {
      fireEntryImuValid = imuReadingFresh(s);
      // If fire interrupts a turn, return to the yaw at which that turn
      // started because the discrete map heading has not been committed yet.
      // During a straight run, return to its held heading for the same reason.
      if (fireEntryImuValid && initialScanTurnActive &&
          initialScanTurnImuValid) {
        fireEntryYaw = initialScanTurnStartYaw;
      } else if (fireEntryImuValid &&
          (motion == TURN_LEFT || motion == TURN_RIGHT || motion == TURN_BACK) &&
          turnImuValid) {
        fireEntryYaw = turnStartYaw;
      } else if (fireEntryImuValid &&
                 (motion == AUTO_CRUISE || motion == AVOID_FORWARD) &&
                 forwardImuValid) {
        fireEntryYaw = forwardTargetYaw;
      } else {
        fireEntryYaw = s.yawDeg;
      }
      Serial.printf("[FIRE] navigation heading saved yaw=%.1f IMU=%d\n",
                    fireEntryYaw, fireEntryImuValid ? 1 : 0);
    }
    fireReturnActive = false;
    current = FIRE;
    motion = IDLE;
    avoidPhase = AVOID_NONE;
    pendingHeading = NO_HEADING;
    cruiseCellsRemaining = 0;
    backgroundScanActive = false;
    initialScanTurnActive = false;
    if (!initialMapReady) scanCycle = 0;

    SensorManager::setUltrasonicScanMode(false);
    ServoScanner::stop();

    FireControl::update(
    true,
    s.fireNear || PUMP_ON_CONFIRMED_FIRE,
    s.flameLeftSignal,
    s.flameRightSignal,
    FlameServoScanner::held()
      ? FlameServoScanner::heldAngle() : FlameServoScanner::leftAngle()
    );
    return;
  }

  // Manual mode
  if (!WebControl::automaticMode()) {
    if (!controlWasManual) {
      controlWasManual = true;
      motion = IDLE;
      avoidPhase = AVOID_NONE;
      pendingHeading = NO_HEADING;
      cruiseCellsRemaining = 0;
      backgroundScanActive = false;
      initialScanTurnActive = false;
      if (!initialMapReady) scanCycle = 0;

      SensorManager::setUltrasonicScanMode(false);
      ServoScanner::stop();
      MotorControl::stop();
    }
    return;
  }

  // Recover from fire mode
  if (current == FIRE) {
    FireControl::update(false, false, 0, 0);

    if (fireEntryImuValid && imuReadingFresh(s)) {
      const float returnError = fireEntryYaw - s.yawDeg;
      if (!fireReturnActive &&
          fabsf(returnError) > MPU_FIRE_RETURN_TOLERANCE_DEG) {
        fireReturnActive = true;
        fireReturnStartMs = millis();
        Serial.printf("[FIRE] returning to saved heading error=%.1f deg\n",
                      returnError);
      }

      if (fireReturnActive &&
          fabsf(returnError) > MPU_FIRE_RETURN_TOLERANCE_DEG &&
          millis() - fireReturnStartMs < MPU_FIRE_RETURN_MAX_MS) {
        if (returnError > 0.0f) MotorControl::left(PWM_FIRE);
        else MotorControl::right(PWM_FIRE);
        return;
      }
    }

    MotorControl::stop();
    if (fireReturnActive) {
      Serial.printf("[FIRE] navigation heading restored yaw=%.1f\n", s.yawDeg);
    }
    fireReturnActive = false;
    current = MapMemory::isLoaded() ? PATROL : EXPLORE;
    controlWasManual = false;
    lastMotionFinish = millis();
    if (initialMapReady) requestStationarySweep(F("after fire heading restore"));
    return;
  }

  if (controlWasManual) {
    controlWasManual = false;
    // Manual motion is not odometry-tracked; do not backtrack through a route
    // whose physical coordinates can no longer be guaranteed.
    resetTravelPath();
    lastMotionFinish = millis();
    return;
  }

  if (initialScanTurnActive) {
    const uint32_t elapsed = millis() - initialScanTurnStartMs;
    const bool imuFresh = initialScanTurnImuValid && imuReadingFresh(s);
    const bool imuReached = imuFresh && elapsed >= MPU_TURN_MIN_MS &&
      yawProgressDeg(initialScanTurnStartYaw, s) >= MPU_TURN_180_STOP_DEG;
    const uint32_t timeout = imuFresh
      ? 2 * MPU_TURN_MAX_90_MS : SCAN_BODY_TURN_MS;
    if (imuReached || elapsed >= timeout) {
      MotorControl::stop();
      MapMemory::rotateBack(p);
      initialScanTurnActive = false;
      initialScanTurnImuValid = false;
      scanStartHeading = p.heading;
      startBackgroundScan();
      Serial.printf("[RADAR] INITIAL BODY TURN 180 COMPLETE yaw=%.1f source=%s\n",
                    yawProgressDeg(initialScanTurnStartYaw, s),
                    imuReached ? "IMU" : "TIME");
    } else {
      // Keep asserting the turn command. Previously the generic
      // !initialMapReady branch stopped the motors immediately after the
      // first command, so the pose changed but the chassis did not.
      MotorControl::right(PWM_TURN);
    }
    return;
  }

  if (backgroundScanActive) {
    runBackgroundScan();
  } else {
    startBackgroundScan();
  }

  // The first completed sweep can start the initial 180-degree body turn.
  // Do not fall through and cancel that command in the same update tick.
  if (initialScanTurnActive) return;

  if (motion == AUTO_CRUISE || motion == AVOID_FORWARD) {
    if (forwardRadarBlocked()) {
      handleForwardObstacle(forwardRadarDistanceCm, F("RADAR"));
      return;
    }

    if (!sa2ReadingFresh(s)) {
      if (millis() - lastSafetyLogMs >= 1000) {
        lastSafetyLogMs = millis();
        Serial.println(SA2_STOP_ON_INVALID_READING
          ? F("[SAFETY] SA2 lost during forward motion; stopped")
          : F("[SA2] no valid echo while moving; radar/map remains active"));
      }
      if (SA2_STOP_ON_INVALID_READING) {
        MotorControl::stop();
        motion = IDLE;
        avoidPhase = AVOID_NONE;
        pendingHeading = NO_HEADING;
        cruiseCellsRemaining = 0;
        return;
      }
    }

    if (sa2ObstacleConfirmed(s)) {
      handleForwardObstacle(s.sa2DistanceCm, F("SA2"));
      return;
    }

    maintainForwardHeading(s);
  }

  // Finish timed motion
  if (motion != IDLE) {
    if (motion == AUTO_CRUISE) {
      if (millis() - motionStart >= AUTO_POSE_CELL_INTERVAL_MS) {
        int16_t nextX = p.x;
        int16_t nextY = p.y;
        advance(nextX, nextY, p.heading);

        if (!inside(nextX, nextY) || MapMemory::isBlocked(nextX, nextY)) {
          Serial.println(F("[NAV] CRUISE PATH BLOCKED -> REPLAN"));
          requestStationarySweep(F("cruise path blocked"));
          return;
        }

        MapMemory::updatePoseForward(p);
        markCurrentVisited();
        recordTravelPose();
        consecutiveTurnsWithoutForward = 0;

        if (cruiseCellsRemaining > 0) --cruiseCellsRemaining;
        if (cruiseCellsRemaining == 0) {
          Serial.println(F("[NAV] FREE CORRIDOR COMPLETE -> STATIONARY RESCAN"));
          requestStationarySweep(F("free corridor complete"));
          return;
        }

        motionStart += AUTO_POSE_CELL_INTERVAL_MS;
      }
      // Keep the forward PWM active; the cell timer updates pose only.
      return;
    }

    uint32_t required = motion == TURN_BACK ? 2 * TURN_90_MS : TURN_90_MS;
    if (motion == BACK_CELL) {
      required = avoidPhase == AVOID_BACKING
        ? (backingAlongHistory ? AUTO_POSE_CELL_INTERVAL_MS : SA2_AVOID_BACK_MS)
        : AUTO_POSE_CELL_INTERVAL_MS;
    } else if (motion == AVOID_FORWARD) {
      // Escape exactly one calibrated cell so the stored route and physical
      // position remain consistent; the planner decides the following cell.
      required = AUTO_POSE_CELL_INTERVAL_MS;
    } else if ((avoidPhase == AVOID_TURNING ||
                avoidPhase == AVOID_ALIGNING_BACKTRACK) &&
               (motion == TURN_LEFT || motion == TURN_RIGHT || motion == TURN_BACK)) {
      required = motion == TURN_BACK
        ? 2 * SA2_AVOID_TURN_90_MS : SA2_AVOID_TURN_90_MS;
    }

    bool motionFinished = millis() - motionStart >= required;
    bool turnImuReached = false;
    const bool isTurn = motion == TURN_LEFT || motion == TURN_RIGHT ||
      motion == TURN_BACK;
    if (isTurn && turnImuValid && imuReadingFresh(s)) {
      const float target = motion == TURN_BACK
        ? MPU_TURN_180_STOP_DEG : MPU_TURN_90_STOP_DEG;
      const uint32_t elapsed = millis() - motionStart;
      turnImuReached = elapsed >= MPU_TURN_MIN_MS &&
        yawProgressDeg(turnStartYaw, s) >= target;
      motionFinished = turnImuReached ||
        elapsed >= (motion == TURN_BACK
          ? 2 * MPU_TURN_MAX_90_MS : MPU_TURN_MAX_90_MS);
    }

    if (motionFinished) {
      if (isTurn) {
        Serial.printf("[NAV] TURN COMPLETE yaw=%.1f source=%s\n",
                      yawProgressDeg(turnStartYaw, s),
                      turnImuReached ? "IMU" : "TIME");
      }
      finishMotion();
    }
    return;
  }

  if (!initialMapReady) {
    MotorControl::stop();
    return;
  }

  // Run coverage with continuous scanning
  runCoverage(s);
}

Pose pose() {
  return p;
}

Mode mode() {
  return current;
}

void forceStop() {
  motion = IDLE;
  pendingHeading = NO_HEADING;
  cruiseCellsRemaining = 0;
  backgroundScanActive = false;

  SensorManager::setUltrasonicScanMode(false);
  ServoScanner::stop();
  MotorControl::stop();
}

} // namespace Navigation
