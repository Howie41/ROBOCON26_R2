#include "merlin_map.h"

#include <atomic>

#include "state_machine_task.h"

extern std::atomic<area_type> g_config_area_type;

namespace merlin_map {
MerlinLayout g_blue_layout{
    50,
    70,
    {3399, 4599, 5799, 6999},
    {0, 0, 0, 0},
    {0, 0, 0},
    {2726, 1526, 326},
    {
        {400, 200, 400},
        {200, 400, 600},
        {400, 600, 400},
        {200, 400, 200},
    },
    {},
    {
        MerlinPose{2150, 2726+70, 0},
        MerlinPose{2150, 1526+70, 0},
        MerlinPose{2150, 326+70, 0},
    },
    MerlinPose{2150, 1526, 0},
    MerlinPose{2509, 1526, 0},
    MerlinPose{3119, 1526, 0},
};

MerlinLayout g_red_layout{
    50,
    -84,
    {3399, 4599, 5799, 6999},
    {0, 0, 0, 0},
    {0, 0, 0},
    {-326, -1526, -2726},
    {
        {400, 200, 400},
        {600, 400, 200},
        {400, 600, 400},
        {200, 400, 200},
    },
    {},
    {
        MerlinPose{2150, -326-84, 0},
        MerlinPose{2150, -1526-84, 0},
        MerlinPose{2150, -2726-84, 0},
    },
    MerlinPose{2150, -1526, 0},
    MerlinPose{2509, -1526, 0},
    MerlinPose{3119, -1526, 0},
};

namespace {

constexpr int32_t kIdentifyMaxDistSq = 1000 * 1000;

Cell g_current_cell{};
bool g_current_cell_valid = false;
Heading g_heading = Heading::PosX;
Debug g_debug{};

Cell makeCellFromLayout(const MerlinLayout &layout, uint8_t row, uint8_t col) {
  const uint8_t row_idx = static_cast<uint8_t>(row - 1U);
  const uint8_t col_idx = static_cast<uint8_t>(col - 1U);
  const CellTrim &trim = layout.cell_trim[row_idx][col_idx];

  const int32_t center_x =
      static_cast<int32_t>(layout.global_dx) +
      static_cast<int32_t>(layout.row_x[row_idx]) +
      static_cast<int32_t>(layout.col_dx[col_idx]) +
      static_cast<int32_t>(trim.dx);
  const int32_t center_y =
      static_cast<int32_t>(layout.global_dy) +
      static_cast<int32_t>(layout.col_y[col_idx]) +
      static_cast<int32_t>(layout.row_dy[row_idx]) +
      static_cast<int32_t>(trim.dy);

  return Cell{
      row,
      col,
      static_cast<int16_t>(center_x),
      static_cast<int16_t>(center_y),
      layout.height_mm[row_idx][col_idx],
  };
}

MerlinPose applyGlobalOffset(const MerlinLayout &layout,
                             const MerlinPose &pose) {
  return MerlinPose{
      static_cast<int16_t>(pose.x + layout.global_dx),
      static_cast<int16_t>(pose.y + layout.global_dy),
      pose.yaw,
  };
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

const MerlinLayout &blueLayout() { return g_blue_layout; }

const MerlinLayout &redLayout() { return g_red_layout; }

const MerlinLayout &currentLayout() {
  return (g_config_area_type.load() == area_type::red) ? g_red_layout
                                                        : g_blue_layout;
}

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

  const MerlinLayout &layout = currentLayout();
  Cell nearest_cell{};
  bool has_nearest = false;
  int32_t nearest_dist_sq = 0;

  for (uint8_t row = 1U; row <= 4U; ++row) {
    for (uint8_t col = 1U; col <= 3U; ++col) {
      const Cell cell = makeCellFromLayout(layout, row, col);
      const int32_t dx = static_cast<int32_t>(x) - cell.center_x;
      const int32_t dy = static_cast<int32_t>(y) - cell.center_y;
      const int32_t dist_sq = dx * dx + dy * dy;
      if (!has_nearest || dist_sq < nearest_dist_sq) {
        nearest_cell = cell;
        has_nearest = true;
        nearest_dist_sq = dist_sq;
      }
    }
  }

  if (!has_nearest) {
    clearCurrentCell();
    g_debug.nearest_dist_sq = 0;
    g_debug.matched_center_x = 0;
    g_debug.matched_center_y = 0;
    return false;
  }

  g_debug.nearest_dist_sq = nearest_dist_sq;
  g_debug.matched_center_x = nearest_cell.center_x;
  g_debug.matched_center_y = nearest_cell.center_y;

  if (nearest_dist_sq > kIdentifyMaxDistSq) {
    clearCurrentCell();
    return false;
  }

  g_current_cell = nearest_cell;
  g_current_cell_valid = true;
  g_debug.cell_valid = true;
  g_debug.row = nearest_cell.row;
  g_debug.col = nearest_cell.col;
  g_debug.height_mm = nearest_cell.height_mm;
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

  return tryGetCellByRowCol(static_cast<uint8_t>(next_row),
                            static_cast<uint8_t>(next_col), out_cell);
}

bool tryGetCellByRowCol(uint8_t row, uint8_t col, Cell *out_cell) {
  if (out_cell == nullptr || row < 1U || row > 4U || col < 1U || col > 3U) {
    return false;
  }

  *out_cell = makeCellFromLayout(currentLayout(), row, col);
  return true;
}

MerlinPose entryPose(uint8_t col) {
  if (col < 1U || col > 3U) {
    return MerlinPose{};
  }

  return applyGlobalOffset(currentLayout(), currentLayout().entry_pose[col - 1U]);
}

MerlinPose stairFrontPose() {
  return applyGlobalOffset(currentLayout(), currentLayout().stair_front_pose);
}

MerlinPose stairClosePose() {
  return applyGlobalOffset(currentLayout(), currentLayout().stair_close_pose);
}

MerlinPose stairHighDrivePose() {
  return applyGlobalOffset(currentLayout(), currentLayout().stair_high_drive_pose);
}

MerlinPose stairCenterPose() {
  Cell center_cell{};
  if (!tryGetCellByRowCol(1U, 2U, &center_cell)) {
    return MerlinPose{};
  }

  return MerlinPose{center_cell.center_x, center_cell.center_y, 0};
}

const Debug &debug() { return g_debug; }

}  // namespace merlin_map
