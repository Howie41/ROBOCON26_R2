#pragma once

#include <cstdint>

namespace merlin_map {

struct MerlinPose {
  int16_t x{0};
  int16_t y{0};
  int16_t yaw{0};
};

struct CellTrim {
  int16_t dx{0};
  int16_t dy{0};
};

struct MerlinLayout {
  int16_t global_dx{0};
  int16_t global_dy{0};

  int16_t row_x[4]{};
  int16_t row_dy[4]{};
  int16_t col_dx[3]{};
  int16_t col_y[3]{};

  int16_t height_mm[4][3]{};
  CellTrim cell_trim[4][3]{};

  MerlinPose entry_pose[3]{};
  MerlinPose stair_front_pose{};
  MerlinPose stair_close_pose{};
  MerlinPose stair_high_drive_pose{};
};

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

extern MerlinLayout g_blue_layout;
extern MerlinLayout g_red_layout;

void init();

Heading heading();
void setHeading(Heading heading);
void rotateCcw90();
void rotateCw90();

const MerlinLayout &blueLayout();
const MerlinLayout &redLayout();
const MerlinLayout &currentLayout();

bool identifyCurrentCell(int16_t x, int16_t y);
void invalidateCurrentCell();

bool tryGetCurrentCell(Cell *out_cell);
bool tryGetNeighborCell(bool forward, Cell *out_cell);
bool tryGetCellByRowCol(uint8_t row, uint8_t col, Cell *out_cell);

MerlinPose entryPose(uint8_t col);
MerlinPose stairFrontPose();
MerlinPose stairClosePose();
MerlinPose stairHighDrivePose();
MerlinPose stairCenterPose();

const Debug &debug();

}  // namespace merlin_map
