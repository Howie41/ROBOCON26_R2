#pragma once

#include <cstdint>

namespace merlin_map {

enum class Heading : uint8_t {
  PosX = 0,
  PosY = 1,
  NegX = 2,
  NegY = 3,
};

struct Cell {
  uint8_t row{0};
  uint8_t col{0};
  int16_t center_x{0};
  int16_t center_y{0};
  int16_t height_mm{0};
};

struct Debug {
  bool cell_valid{false};
  uint8_t row{0};
  uint8_t col{0};
  int16_t height_mm{0};
  uint8_t heading{0};
  int16_t matched_center_x{0};
  int16_t matched_center_y{0};
  int16_t query_x{0};
  int16_t query_y{0};
  int32_t nearest_dist_sq{0};
};

void init();

Heading heading();
void setHeading(Heading heading);
void rotateCcw90();
void rotateCw90();

bool identifyCurrentCell(int16_t x, int16_t y);
void invalidateCurrentCell();

bool tryGetCurrentCell(Cell *out_cell);
bool tryGetNeighborCell(bool forward, Cell *out_cell);

const Debug &debug();

}  // namespace merlin_map
