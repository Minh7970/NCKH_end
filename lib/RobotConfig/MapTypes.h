#pragma once
#include <Arduino.h>

enum CellType : uint8_t {
  CELL_UNKNOWN = 0,
  CELL_FREE = 1,
  CELL_OBSTACLE = 2
};

enum Heading : uint8_t {
  NORTH = 0,
  EAST = 1,
  SOUTH = 2,
  WEST = 3
};

struct Pose {
  int16_t x;
  int16_t y;
  Heading heading;
};
