#include "merlin_map.h"

#include <array>

namespace merlin_map {
namespace {

constexpr int16_t kAnchorX = 3255;
constexpr int16_t kAnchorY = 1770;
constexpr int16_t kPitchMm = 1200;
constexpr int32_t kIdentifyMaxDistSq = 1000 * 1000;

constexpr int16_t kHeightTable[4][3] = {
    {400, 200, 400},
    {200, 400, 600},
    {400, 600, 400},
    {200, 400, 200},
};

constexpr Cell makeCell(uint8_t row, uint8_t col) {
  return Cell{
      row,
      col,
      static_cast<int16_t>(kAnchorX + static_cast<int16_t>(row - 1U) * kPitchMm),
      static_cast<int16_t>(kAnchorY + static_cast<int16_t>(2 - static_cast<int>(col)) *
                           kPitchMm),
      kHeightTable[row - 1U][col - 1U],
  };
}

constexpr std::array<Cell, 12> kCells = {
    makeCell(1, 1), makeCell(1, 2), makeCell(1, 3),
    makeCell(2, 1), makeCell(2, 2), makeCell(2, 3),
    makeCell(3, 1), makeCell(3, 2), makeCell(3, 3),
    makeCell(4, 1), makeCell(4, 2), makeCell(4, 3),
};

Cell g_current_cell{};
bool g_current_cell_valid = false;
Heading g_heading = Heading::PosX;
Debug g_debug{};

const Cell *findCell(uint8_t row, uint8_t col) {
  for (const auto &cell : kCells) {
    if (cell.row == row && cell.col == col) {
      return &cell;
    }
  }
  return nullptr;
}

void clearCurrentCell() {
  g_current_cell = Cell{};
  g_current_cell_valid = false;
  g_debug.cell_valid = false;
  g_debug.row = 0;
  g_debug.col = 0;
  g_debug.height_mm = 0;
}

}  // namespace

void init() {
  g_heading = Heading::PosX;
  g_debug = Debug{};
  g_debug.heading = static_cast<uint8_t>(g_heading);
  clearCurrentCell();
}

Heading heading() { return g_heading; }

void setHeading(Heading heading_in) {
  g_heading = heading_in;
  g_debug.heading = static_cast<uint8_t>(g_heading);
}

void rotateCcw90() {
  switch (g_heading) {
    case Heading::PosX:
      setHeading(Heading::PosY);
      break;
    case Heading::PosY:
      setHeading(Heading::NegX);
      break;
    case Heading::NegX:
      setHeading(Heading::NegY);
      break;
    case Heading::NegY:
      setHeading(Heading::PosX);
      break;
  }
}

void rotateCw90() {
  switch (g_heading) {
    case Heading::PosX:
      setHeading(Heading::NegY);
      break;
    case Heading::NegY:
      setHeading(Heading::NegX);
      break;
    case Heading::NegX:
      setHeading(Heading::PosY);
      break;
    case Heading::PosY:
      setHeading(Heading::PosX);
      break;
  }
}

bool identifyCurrentCell(int16_t x, int16_t y) {
  g_debug.query_x = x;
  g_debug.query_y = y;
  g_debug.heading = static_cast<uint8_t>(g_heading);

  const Cell *nearest_cell = nullptr;
  int32_t nearest_dist_sq = 0;

  for (const auto &cell : kCells) {
    const int32_t dx = static_cast<int32_t>(x) - cell.center_x;
    const int32_t dy = static_cast<int32_t>(y) - cell.center_y;
    const int32_t dist_sq = dx * dx + dy * dy;
    if (nearest_cell == nullptr || dist_sq < nearest_dist_sq) {
      nearest_cell = &cell;
      nearest_dist_sq = dist_sq;
    }
  }

  if (nearest_cell == nullptr) {
    clearCurrentCell();
    g_debug.nearest_dist_sq = 0;
    g_debug.matched_center_x = 0;
    g_debug.matched_center_y = 0;
    return false;
  }

  g_debug.nearest_dist_sq = nearest_dist_sq;
  g_debug.matched_center_x = nearest_cell->center_x;
  g_debug.matched_center_y = nearest_cell->center_y;

  if (nearest_dist_sq > kIdentifyMaxDistSq) {
    clearCurrentCell();
    return false;
  }

  g_current_cell = *nearest_cell;
  g_current_cell_valid = true;
  g_debug.cell_valid = true;
  g_debug.row = nearest_cell->row;
  g_debug.col = nearest_cell->col;
  g_debug.height_mm = nearest_cell->height_mm;
  return true;
}

void invalidateCurrentCell() { clearCurrentCell(); }

bool tryGetCurrentCell(Cell *out_cell) {
  if (!g_current_cell_valid || out_cell == nullptr) {
    return false;
  }
  *out_cell = g_current_cell;
  return true;
}

bool tryGetNeighborCell(bool forward, Cell *out_cell) {
  if (!g_current_cell_valid || out_cell == nullptr) {
    return false;
  }

  int next_row = g_current_cell.row;
  int next_col = g_current_cell.col;

  switch (g_heading) {
    case Heading::PosX:
      next_row += forward ? 1 : -1;
      break;
    case Heading::NegX:
      next_row += forward ? -1 : 1;
      break;
    case Heading::PosY:
      next_col += forward ? -1 : 1;
      break;
    case Heading::NegY:
      next_col += forward ? 1 : -1;
      break;
  }

  if (next_row < 1 || next_row > 4 || next_col < 1 || next_col > 3) {
    return false;
  }

  const Cell *cell = findCell(static_cast<uint8_t>(next_row),
                              static_cast<uint8_t>(next_col));
  if (cell == nullptr) {
    return false;
  }

  *out_cell = *cell;
  return true;
}

const Debug &debug() { return g_debug; }

}  // namespace merlin_map
