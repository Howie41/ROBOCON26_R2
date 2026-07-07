#pragma once

#include <cstdint>

enum class StairAssistLaser1State : uint8_t {
  Invalid = 0,
  Far,
  NearStair,
  EdgeOpen,
};

enum class StairAssistLaser2State : uint8_t {
  Invalid = 0,
  GroundNormal,
  HighSuspended,
  StepContact,
};

enum class StairAssistMode : uint8_t {
  ClimbUp = 0,
  Descend,
};

enum class StairAssistLaser3Profile : uint8_t {
  Center = 0,
  Side,
};

struct StairAssistDebug {
  bool enabled{false};
  bool auto_lower_enabled{false};
  bool laser1_fresh{false};
  bool laser2_fresh{false};
  bool laser3_fresh{false};
  uint8_t assist_mode{static_cast<uint8_t>(StairAssistMode::ClimbUp)};

  int32_t laser1_mm{0};
  int32_t laser2_mm{0};
  int32_t laser3_mm{0};

  uint32_t laser1_frame_count{0};
  uint32_t laser2_frame_count{0};
  uint32_t laser3_frame_count{0};

  uint8_t laser1_state{static_cast<uint8_t>(StairAssistLaser1State::Invalid)};
  uint8_t laser2_state{static_cast<uint8_t>(StairAssistLaser2State::Invalid)};
  uint8_t laser3_state{static_cast<uint8_t>(StairAssistLaser1State::Invalid)};
  uint8_t laser3_profile{static_cast<uint8_t>(StairAssistLaser3Profile::Center)};

  uint8_t laser1_near_count{0};
  uint8_t laser1_edge_count{0};
  uint8_t laser3_near_count{0};
  uint8_t laser3_edge_count{0};
  uint8_t laser1_auto_lower_count{0};
  uint8_t laser1_descend_ready_count{0};
  uint8_t laser1_descend_lower_count{0};
  uint8_t laser2_climb_high_count{0};
  uint8_t laser2_descend_lower_count{0};
  uint8_t laser2_ground_count{0};
  uint8_t laser2_high_count{0};
  uint8_t laser2_step_count{0};
  bool front_photogate_blocked{false};
  bool front_photogate_unblocked{false};
  bool front_photogate_blocked_edge{false};
  bool front_photogate_unblocked_edge{false};
  bool front_photogate_descend_lower_latched{false};
  bool rear_photogate_blocked{false};
  bool rear_photogate_unblocked{false};
  bool rear_photogate_blocked_edge{false};
  bool rear_photogate_unblocked_edge{false};
  bool rear_photogate_climb_lower_latched{false};
  bool rear_photogate_descend_high_latched{false};

  bool saw_laser2_high_for_climb{false};
  bool saw_laser2_close_for_descend{false};

  bool suggest_climb_up{false};
  bool suggest_descend_high{false};
  bool suggest_descend_edge_ready{false};
  bool should_lower_after_climb{false};
  bool should_lower_after_descend{false};

  int32_t laser3_near_min_used_mm{0};
  int32_t laser3_near_max_used_mm{0};
};

void stairAssistInit();
void stairAssistSetEnabled(bool enabled);
bool stairAssistEnabled();
void stairAssistSetMode(StairAssistMode mode);
StairAssistMode stairAssistMode();
void stairAssistSetLaser3Profile(StairAssistLaser3Profile profile);
StairAssistLaser3Profile stairAssistLaser3Profile();
void stairAssistSetAutoLowerEnabled(bool enabled);
bool stairAssistAutoLowerEnabled();
void stairAssistUpdate();
void stairAssistResetProgress();

StairAssistLaser1State stairAssistLaser1State();
StairAssistLaser2State stairAssistLaser2State();

bool stairAssistSuggestClimbUp();
bool stairAssistSuggestDescendHighMode();
bool stairAssistSuggestDescendEdgeReady();
bool stairAssistSuggestGoToEdgeHigh();
bool stairAssistSuggestGoToEdgeLow();
bool stairAssistShouldLowerAfterClimbAdvance();
bool stairAssistShouldLowerAfterDescendRetreat();

const StairAssistDebug &stairAssistDebug();

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint8_t g_stair_front_descend_lower_latched;
extern volatile uint8_t g_stair_laser2_go_to_edge_low_ready;
extern volatile uint8_t g_stair_laser2_descend_lower_ready;
extern volatile uint8_t g_stair_go_to_edge_low_ready;
extern volatile uint8_t g_stair_should_lower_after_descend;

#ifdef __cplusplus
}
#endif
