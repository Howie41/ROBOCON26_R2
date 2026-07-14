#include "stair_assist.h"

#include "cmsis_os2.h"
#include "com_config.h"
#include "photogate.hpp"

extern "C" {

volatile uint8_t g_stair_front_descend_lower_latched = 0U;
volatile uint8_t g_stair_laser2_go_to_edge_low_ready = 0U;
volatile uint8_t g_stair_laser2_descend_lower_ready = 0U;
volatile uint8_t g_stair_go_to_edge_low_ready = 0U;
volatile uint8_t g_stair_should_lower_after_descend = 0U;

}

namespace {

constexpr uint8_t kStableFrames = 1;
constexpr uint32_t kLaserDataTimeoutMs = 500;
constexpr uint32_t kFrontPhotogateUnblockedHoldMs = 3;

// Laser1 and laser3 are both front-facing stair lasers.
// Laser1 keeps its own center/side thresholds so field tuning does not affect
// laser3.
constexpr int32_t kLaser1AutoLowerMinMm = 950;
constexpr int32_t kLaser1AutoLowerMaxMm = 1120;
constexpr int32_t kLaser1DescendLowerMinMm = 1650;
constexpr int32_t kLaser1DescendLowerMaxMm = 1700;
constexpr int32_t kLaser1EdgeMinMm = 800;

// Laser3 is another front-facing stair laser. It uses the same state logic as
// laser1, but keeps its own thresholds so field tuning does not affect laser1.
constexpr int32_t kLaser3EdgeMinMm = 800;

constexpr int32_t kLaser2ClimbHighMinMm = 170;
constexpr int32_t kLaser2ClimbHighMaxMm = 240;
constexpr int32_t kLaser2GoToEdgeLowMinMm = 500;
constexpr int32_t kLaser2GoToEdgeLowMaxMm = 700;
constexpr int32_t kLaser2DescendLowerMinMm = 280;
constexpr int32_t kLaser2DescendLowerMaxMm = 500;

// Laser2 is mounted on the front leg and uses the user's current estimates:
// ground ~600 mm, high suspended ~1000 mm, step contact ~300 mm.
constexpr int32_t kLaser2StepMinMm = 0;
constexpr int32_t kLaser2StepMaxMm = 449;
constexpr int32_t kLaser2GroundMinMm = 450;
constexpr int32_t kLaser2GroundMaxMm = 800;
constexpr int32_t kLaser2HighMinMm = 801;
constexpr int32_t kLaser2HighMaxMm = 1400;

struct SensorFrameTrack {
  uint32_t last_frame_count{0};
  uint32_t last_update_tick{0};
};

StairAssistDebug g_debug{};
SensorFrameTrack g_laser1_track{};
SensorFrameTrack g_laser2_track{};
SensorFrameTrack g_laser3_track{};
StairAssistLaser1State g_laser1_state{StairAssistLaser1State::Invalid};
StairAssistLaser2State g_laser2_state{StairAssistLaser2State::Invalid};
StairAssistLaser1State g_laser3_state{StairAssistLaser1State::Invalid};
StairAssistMode g_mode{StairAssistMode::ClimbUp};
StairAssistLaser3Profile g_laser3_profile{StairAssistLaser3Profile::Center};

bool g_enabled{false};
bool g_auto_lower_enabled{false};
bool g_saw_laser2_high_for_climb{false};
bool g_saw_laser2_close_for_descend{false};
bool g_front_photogate_descend_lower_latched{false};
bool g_rear_photogate_climb_lower_latched{false};
bool g_rear_photogate_descend_high_latched{false};
uint32_t g_front_photogate_unblocked_since_tick{0};
bool g_front_photogate_unblocked_hold_ready{false};

volatile int32_t g_laser1_center_near_min_mm = 400;
volatile int32_t g_laser1_center_near_max_mm = 435;
volatile int32_t g_laser1_side_near_min_mm = 400;
volatile int32_t g_laser1_side_near_max_mm = 435;
volatile int32_t g_laser1_center_go_edge_min_mm = 400;
volatile int32_t g_laser1_center_go_edge_max_mm = 423;
volatile int32_t g_laser1_side_go_edge_min_mm = 430;
volatile int32_t g_laser1_side_go_edge_max_mm = 504;

volatile int32_t g_laser3_center_near_min_mm = 400;
volatile int32_t g_laser3_center_near_max_mm = 470;
volatile int32_t g_laser3_side_near_min_mm = 400;
volatile int32_t g_laser3_side_near_max_mm = 510;
volatile int32_t g_laser3_center_go_edge_min_mm = 400;
volatile int32_t g_laser3_center_go_edge_max_mm = 435;
volatile int32_t g_laser3_side_go_edge_min_mm = 400;
volatile int32_t g_laser3_side_go_edge_max_mm = 510;

template <typename T>
void saturatingIncrement(T &value) {
  if (value < static_cast<T>(255)) {
    ++value;
  }
}

template <typename T>
void clearIfNotMatch(T &value, bool matched) {
  if (!matched) {
    value = 0;
  }
}

bool inRangeInclusive(int32_t value, int32_t min_value, int32_t max_value) {
  return value >= min_value && value <= max_value;
}

int32_t laser1NearMinMm() {
  return (g_laser3_profile == StairAssistLaser3Profile::Center)
             ? g_laser1_center_near_min_mm
             : g_laser1_side_near_min_mm;
}

int32_t laser1NearMaxMm() {
  return (g_laser3_profile == StairAssistLaser3Profile::Center)
             ? g_laser1_center_near_max_mm
             : g_laser1_side_near_max_mm;
}

int32_t laser1GoEdgeMinMm() {
  return (g_laser3_profile == StairAssistLaser3Profile::Center)
             ? g_laser1_center_go_edge_min_mm
             : g_laser1_side_go_edge_min_mm;
}

int32_t laser1GoEdgeMaxMm() {
  return (g_laser3_profile == StairAssistLaser3Profile::Center)
             ? g_laser1_center_go_edge_max_mm
             : g_laser1_side_go_edge_max_mm;
}

int32_t laser3NearMinMm() {
  return (g_laser3_profile == StairAssistLaser3Profile::Center)
             ? g_laser3_center_near_min_mm
             : g_laser3_side_near_min_mm;
}

int32_t laser3NearMaxMm() {
  return (g_laser3_profile == StairAssistLaser3Profile::Center)
             ? g_laser3_center_near_max_mm
             : g_laser3_side_near_max_mm;
}

int32_t laser3GoEdgeMinMm() {
  return (g_laser3_profile == StairAssistLaser3Profile::Center)
             ? g_laser3_center_go_edge_min_mm
             : g_laser3_side_go_edge_min_mm;
}

int32_t laser3GoEdgeMaxMm() {
  return (g_laser3_profile == StairAssistLaser3Profile::Center)
             ? g_laser3_center_go_edge_max_mm
             : g_laser3_side_go_edge_max_mm;
}

bool frameIsFresh(uint32_t now_tick, const SensorFrameTrack &track,
                  const LaserMeasure::MeasureResult &result) {
  if (!result.valid || result.is_error || track.last_frame_count == 0) {
    return false;
  }

  return (now_tick - track.last_update_tick) <= kLaserDataTimeoutMs;
}

uint8_t toDebugState(StairAssistLaser1State state) {
  return static_cast<uint8_t>(state);
}

uint8_t toDebugState(StairAssistLaser2State state) {
  return static_cast<uint8_t>(state);
}

void updateFrameTrack(uint32_t now_tick, SensorFrameTrack &track,
                      const LaserMeasure::MeasureResult &result) {
  if (result.frame_count != track.last_frame_count) {
    track.last_frame_count = result.frame_count;
    track.last_update_tick = now_tick;
  }
}

StairAssistLaser1State classifyLaser1(int32_t distance_mm, bool fresh) {
  if (!fresh) {
    return StairAssistLaser1State::Invalid;
  }

  if (distance_mm >= kLaser1EdgeMinMm) {
    return StairAssistLaser1State::EdgeOpen;
  }

  if (inRangeInclusive(distance_mm, laser1NearMinMm(), laser1NearMaxMm())) {
    return StairAssistLaser1State::NearStair;
  }

  return StairAssistLaser1State::Far;
}

StairAssistLaser1State classifyLaser3(int32_t distance_mm, bool fresh) {
  if (!fresh) {
    return StairAssistLaser1State::Invalid;
  }

  if (distance_mm >= kLaser3EdgeMinMm) {
    return StairAssistLaser1State::EdgeOpen;
  }

  if (inRangeInclusive(distance_mm, laser3NearMinMm(), laser3NearMaxMm())) {
    return StairAssistLaser1State::NearStair;
  }

  return StairAssistLaser1State::Far;
}

StairAssistLaser2State classifyLaser2(int32_t distance_mm, bool fresh) {
  if (!fresh) {
    return StairAssistLaser2State::Invalid;
  }

  if (inRangeInclusive(distance_mm, kLaser2StepMinMm, kLaser2StepMaxMm)) {
    return StairAssistLaser2State::StepContact;
  }

  if (inRangeInclusive(distance_mm, kLaser2GroundMinMm, kLaser2GroundMaxMm)) {
    return StairAssistLaser2State::GroundNormal;
  }

  if (inRangeInclusive(distance_mm, kLaser2HighMinMm, kLaser2HighMaxMm)) {
    return StairAssistLaser2State::HighSuspended;
  }

  return StairAssistLaser2State::Invalid;
}

void updateLaser1Judge(uint32_t now_tick) {
  const auto &result = laser1.latestResult();
  updateFrameTrack(now_tick, g_laser1_track, result);

  g_debug.laser1_mm = result.distance_mm;
  g_debug.laser1_frame_count = result.frame_count;
  g_debug.laser1_fresh = frameIsFresh(now_tick, g_laser1_track, result);
  g_debug.laser1_near_min_used_mm = laser1NearMinMm();
  g_debug.laser1_near_max_used_mm = laser1NearMaxMm();
  g_debug.laser1_go_edge_min_used_mm = laser1GoEdgeMinMm();
  g_debug.laser1_go_edge_max_used_mm = laser1GoEdgeMaxMm();

  g_laser1_state = classifyLaser1(result.distance_mm, g_debug.laser1_fresh);
  g_debug.laser1_state = toDebugState(g_laser1_state);

  const bool near_match = g_laser1_state == StairAssistLaser1State::NearStair;
  const bool edge_match = g_laser1_state == StairAssistLaser1State::EdgeOpen;
  const bool go_edge_match =
      g_debug.laser1_fresh &&
      inRangeInclusive(result.distance_mm, laser1GoEdgeMinMm(),
                       laser1GoEdgeMaxMm());
  const bool auto_lower_match =
      inRangeInclusive(result.distance_mm, kLaser1AutoLowerMinMm,
                       kLaser1AutoLowerMaxMm) &&
      g_debug.laser1_fresh;
  const bool descend_lower_match =
      inRangeInclusive(result.distance_mm, kLaser1DescendLowerMinMm,
                       kLaser1DescendLowerMaxMm) &&
      g_debug.laser1_fresh;

  if (near_match) {
    saturatingIncrement(g_debug.laser1_near_count);
  }
  clearIfNotMatch(g_debug.laser1_near_count, near_match);

  if (edge_match) {
    saturatingIncrement(g_debug.laser1_edge_count);
  }
  clearIfNotMatch(g_debug.laser1_edge_count, edge_match);

  if (go_edge_match) {
    saturatingIncrement(g_debug.laser1_go_edge_count);
  }
  clearIfNotMatch(g_debug.laser1_go_edge_count, go_edge_match);

  if (auto_lower_match) {
    saturatingIncrement(g_debug.laser1_auto_lower_count);
  }
  clearIfNotMatch(g_debug.laser1_auto_lower_count, auto_lower_match);

  if (auto_lower_match) {
    saturatingIncrement(g_debug.laser1_descend_ready_count);
  }
  clearIfNotMatch(g_debug.laser1_descend_ready_count, auto_lower_match);

  if (descend_lower_match) {
    saturatingIncrement(g_debug.laser1_descend_lower_count);
  }
  clearIfNotMatch(g_debug.laser1_descend_lower_count, descend_lower_match);
}

void updateLaser3Judge(uint32_t now_tick) {
  const auto &result = laser3.latestResult();
  if (result.frame_count != g_laser3_track.last_frame_count) {
    g_laser3_track.last_frame_count = result.frame_count;
    g_laser3_track.last_update_tick = now_tick;
  }

  g_debug.laser3_mm = result.distance_mm;
  g_debug.laser3_frame_count = result.frame_count;
  g_debug.laser3_fresh =
      result.valid && !result.is_error && g_laser3_track.last_frame_count != 0 &&
      ((now_tick - g_laser3_track.last_update_tick) <= kLaserDataTimeoutMs);
  g_debug.laser3_profile = static_cast<uint8_t>(g_laser3_profile);
  g_debug.laser3_near_min_used_mm = laser3NearMinMm();
  g_debug.laser3_near_max_used_mm = laser3NearMaxMm();
  g_debug.laser3_go_edge_min_used_mm = laser3GoEdgeMinMm();
  g_debug.laser3_go_edge_max_used_mm = laser3GoEdgeMaxMm();

  g_laser3_state = classifyLaser3(result.distance_mm, g_debug.laser3_fresh);
  g_debug.laser3_state = toDebugState(g_laser3_state);

  const bool near_match = g_laser3_state == StairAssistLaser1State::NearStair;
  const bool edge_match = g_laser3_state == StairAssistLaser1State::EdgeOpen;
  const bool go_edge_match =
      g_debug.laser3_fresh &&
      inRangeInclusive(result.distance_mm, laser3GoEdgeMinMm(),
                       laser3GoEdgeMaxMm());

  if (near_match) {
    saturatingIncrement(g_debug.laser3_near_count);
  }
  clearIfNotMatch(g_debug.laser3_near_count, near_match);

  if (edge_match) {
    saturatingIncrement(g_debug.laser3_edge_count);
  }
  clearIfNotMatch(g_debug.laser3_edge_count, edge_match);

  if (go_edge_match) {
    saturatingIncrement(g_debug.laser3_go_edge_count);
  }
  clearIfNotMatch(g_debug.laser3_go_edge_count, go_edge_match);
}

void updateLaser2Judge(uint32_t now_tick) {
  const auto &result = laser2.latestResult();
  updateFrameTrack(now_tick, g_laser2_track, result);

  g_debug.laser2_mm = result.distance_mm;
  g_debug.laser2_frame_count = result.frame_count;
  g_debug.laser2_fresh = frameIsFresh(now_tick, g_laser2_track, result);

  g_laser2_state = classifyLaser2(result.distance_mm, g_debug.laser2_fresh);
  g_debug.laser2_state = toDebugState(g_laser2_state);

  const bool ground_match = g_laser2_state == StairAssistLaser2State::GroundNormal;
  const bool high_match = g_laser2_state == StairAssistLaser2State::HighSuspended;
  const bool step_match = g_laser2_state == StairAssistLaser2State::StepContact;
  const bool climb_high_match =
      inRangeInclusive(result.distance_mm, kLaser2ClimbHighMinMm,
                       kLaser2ClimbHighMaxMm) &&
      g_debug.laser2_fresh;
  const bool descend_lower_match =
      inRangeInclusive(result.distance_mm, kLaser2DescendLowerMinMm,
                       kLaser2DescendLowerMaxMm) &&
      g_debug.laser2_fresh;

  if (ground_match) {
    saturatingIncrement(g_debug.laser2_ground_count);
  }
  clearIfNotMatch(g_debug.laser2_ground_count, ground_match);

  if (high_match) {
    saturatingIncrement(g_debug.laser2_high_count);
  }
  clearIfNotMatch(g_debug.laser2_high_count, high_match);

  if (step_match) {
    saturatingIncrement(g_debug.laser2_step_count);
  }
  clearIfNotMatch(g_debug.laser2_step_count, step_match);

  if (climb_high_match) {
    saturatingIncrement(g_debug.laser2_climb_high_count);
  }
  clearIfNotMatch(g_debug.laser2_climb_high_count, climb_high_match);

  if (descend_lower_match) {
    saturatingIncrement(g_debug.laser2_descend_lower_count);
  }
  clearIfNotMatch(g_debug.laser2_descend_lower_count, descend_lower_match);
}

void updateRearPhotogateJudge(uint32_t now_tick) {
  g_debug.front_photogate_blocked =
      photogate::blocked(photogate::GateId::Front);
  g_debug.front_photogate_unblocked =
      photogate::unblocked(photogate::GateId::Front);

  const bool front_blocked_edge =
      photogate::consumeBlockedEdge(photogate::GateId::Front);
  const bool front_unblocked_edge =
      photogate::consumeUnblockedEdge(photogate::GateId::Front);

  g_debug.front_photogate_blocked_edge = front_blocked_edge;
  g_debug.front_photogate_unblocked_edge = front_unblocked_edge;

  if (front_unblocked_edge) {
    g_front_photogate_descend_lower_latched = true;
  }

  if (g_debug.front_photogate_unblocked) {
    if (g_front_photogate_unblocked_since_tick == 0U) {
      g_front_photogate_unblocked_since_tick = now_tick;
    }
    g_front_photogate_unblocked_hold_ready =
        (now_tick - g_front_photogate_unblocked_since_tick) >=
        kFrontPhotogateUnblockedHoldMs;
  } else {
    g_front_photogate_unblocked_since_tick = 0U;
    g_front_photogate_unblocked_hold_ready = false;
  }

  g_debug.front_photogate_descend_lower_latched =
      g_front_photogate_descend_lower_latched;
  g_stair_front_descend_lower_latched =
      g_front_photogate_descend_lower_latched ? 1U : 0U;

  g_debug.rear_photogate_blocked =
      photogate::blocked(photogate::GateId::Rear);
  g_debug.rear_photogate_unblocked =
      photogate::unblocked(photogate::GateId::Rear);

  const bool blocked_edge =
      photogate::consumeBlockedEdge(photogate::GateId::Rear);
  const bool unblocked_edge =
      photogate::consumeUnblockedEdge(photogate::GateId::Rear);

  g_debug.rear_photogate_blocked_edge = blocked_edge;
  g_debug.rear_photogate_unblocked_edge = unblocked_edge;

  if (blocked_edge) {
    g_rear_photogate_climb_lower_latched = true;
  }
  if (unblocked_edge) {
    g_rear_photogate_descend_high_latched = true;
  }

  g_debug.rear_photogate_climb_lower_latched =
      g_rear_photogate_climb_lower_latched;
  g_debug.rear_photogate_descend_high_latched =
      g_rear_photogate_descend_high_latched;
}

void updateDecisionFlags() {
  g_debug.enabled = g_enabled;
  g_debug.assist_mode = static_cast<uint8_t>(g_mode);

  if (!g_enabled) {
    stairAssistResetProgress();
    g_debug.suggest_climb_up = false;
    g_debug.suggest_descend_high = false;
    g_debug.suggest_descend_edge_ready = false;
    g_debug.should_lower_after_climb = false;
    g_debug.should_lower_after_descend = false;
    g_stair_laser2_go_to_edge_low_ready = 0U;
    g_stair_laser2_descend_lower_ready = 0U;
    g_stair_go_to_edge_low_ready = 0U;
    g_stair_should_lower_after_descend = 0U;
    return;
  }

  g_debug.suggest_climb_up =
      (g_mode == StairAssistMode::ClimbUp) &&
      ((g_debug.laser1_near_count >= kStableFrames) ||
       (g_debug.laser3_near_count >= kStableFrames) ||
       (g_debug.laser2_climb_high_count >= kStableFrames));
  g_debug.suggest_descend_high =
      (g_mode == StairAssistMode::Descend) &&
      // Edge remains the preferred trigger, but a sustained unblocked level is
      // also accepted so slow descend motions do not miss the single transition.
      (g_rear_photogate_descend_high_latched ||
       g_debug.rear_photogate_unblocked);
  g_debug.suggest_descend_edge_ready =
      g_debug.laser1_edge_count >= kStableFrames;
  g_debug.should_lower_after_climb =
      g_auto_lower_enabled &&
      (g_mode == StairAssistMode::ClimbUp) &&
      // Edge remains the preferred trigger, but a sustained blocked level is
      // also accepted so slow climb motions do not miss the single transition.
      (g_rear_photogate_climb_lower_latched ||
       g_debug.rear_photogate_blocked);

  if (g_debug.laser2_high_count >= kStableFrames) {
    g_saw_laser2_high_for_climb = true;
  }
  if (g_debug.laser2_ground_count >= kStableFrames ||
      g_debug.laser2_step_count >= kStableFrames) {
    g_saw_laser2_close_for_descend = true;
  }

  g_debug.saw_laser2_high_for_climb = g_saw_laser2_high_for_climb;
  g_debug.saw_laser2_close_for_descend = g_saw_laser2_close_for_descend;

  const bool laser2_go_to_edge_low_ready =
      g_debug.laser2_fresh &&
      inRangeInclusive(g_debug.laser2_mm, kLaser2GoToEdgeLowMinMm,
                       kLaser2GoToEdgeLowMaxMm);
  const bool laser2_descend_lower_ready =
      g_debug.laser2_descend_lower_count >= kStableFrames;
  const bool front_go_to_edge_low_ready =
      g_front_photogate_unblocked_hold_ready;

  g_stair_laser2_go_to_edge_low_ready = laser2_go_to_edge_low_ready ? 1U : 0U;
  g_stair_laser2_descend_lower_ready =
      laser2_descend_lower_ready ? 1U : 0U;
  g_stair_go_to_edge_low_ready =
      (laser2_go_to_edge_low_ready || front_go_to_edge_low_ready) ? 1U : 0U;

  g_debug.should_lower_after_descend =
      g_auto_lower_enabled &&
      (g_mode == StairAssistMode::Descend) &&
      laser2_descend_lower_ready;
  g_stair_should_lower_after_descend =
      g_debug.should_lower_after_descend ? 1U : 0U;
}

}  // namespace

void stairAssistInit() {
  g_enabled = false;
  g_auto_lower_enabled = false;
  g_mode = StairAssistMode::ClimbUp;
  g_laser3_profile = StairAssistLaser3Profile::Center;
  g_saw_laser2_high_for_climb = false;
  g_saw_laser2_close_for_descend = false;
  g_front_photogate_descend_lower_latched = false;
  g_rear_photogate_climb_lower_latched = false;
  g_rear_photogate_descend_high_latched = false;
  g_front_photogate_unblocked_since_tick = 0U;
  g_front_photogate_unblocked_hold_ready = false;
  g_laser1_track = {};
  g_laser2_track = {};
  g_laser3_track = {};
  g_laser1_state = StairAssistLaser1State::Invalid;
  g_laser2_state = StairAssistLaser2State::Invalid;
  g_laser3_state = StairAssistLaser1State::Invalid;
  g_stair_front_descend_lower_latched = 0U;
  g_stair_laser2_go_to_edge_low_ready = 0U;
  g_stair_laser2_descend_lower_ready = 0U;
  g_stair_go_to_edge_low_ready = 0U;
  g_stair_should_lower_after_descend = 0U;
  g_debug = {};
}

void stairAssistSetEnabled(bool enabled) {
  if (g_enabled == enabled) {
    return;
  }

  g_enabled = enabled;
  stairAssistResetProgress();
}

bool stairAssistEnabled() {
  return g_enabled;
}

void stairAssistSetMode(StairAssistMode mode) {
  if (g_mode == mode) {
    return;
  }

  g_mode = mode;
  stairAssistResetProgress();
}

StairAssistMode stairAssistMode() {
  return g_mode;
}

void stairAssistSetLaser3Profile(StairAssistLaser3Profile profile) {
  if (g_laser3_profile == profile) {
    return;
  }

  g_laser3_profile = profile;
  g_debug.laser3_profile = static_cast<uint8_t>(g_laser3_profile);
  g_debug.laser1_near_count = 0;
  g_debug.laser1_edge_count = 0;
  g_debug.laser1_go_edge_count = 0;
  g_debug.laser3_near_count = 0;
  g_debug.laser3_edge_count = 0;
  g_debug.laser3_go_edge_count = 0;
}

StairAssistLaser3Profile stairAssistLaser3Profile() {
  return g_laser3_profile;
}

void stairAssistSetAutoLowerEnabled(bool enabled) {
  g_auto_lower_enabled = enabled;
  g_debug.auto_lower_enabled = enabled;
  g_debug.laser1_auto_lower_count = 0;
  g_debug.laser1_descend_ready_count = 0;
  g_debug.laser1_descend_lower_count = 0;
  g_debug.laser1_go_edge_count = 0;
  g_debug.laser3_near_count = 0;
  g_debug.laser3_edge_count = 0;
  g_debug.laser3_go_edge_count = 0;
  g_debug.laser2_climb_high_count = 0;
  g_debug.laser2_descend_lower_count = 0;
  g_front_photogate_descend_lower_latched = false;
  g_rear_photogate_climb_lower_latched = false;
  g_rear_photogate_descend_high_latched = false;
  g_front_photogate_unblocked_since_tick = 0U;
  g_front_photogate_unblocked_hold_ready = false;
  g_debug.front_photogate_descend_lower_latched = false;
  g_debug.rear_photogate_climb_lower_latched = false;
  g_debug.rear_photogate_descend_high_latched = false;
  g_debug.should_lower_after_climb = false;
  g_debug.should_lower_after_descend = false;
  g_stair_front_descend_lower_latched = 0U;
  g_stair_laser2_go_to_edge_low_ready = 0U;
  g_stair_laser2_descend_lower_ready = 0U;
  g_stair_go_to_edge_low_ready = 0U;
  g_stair_should_lower_after_descend = 0U;
}

bool stairAssistAutoLowerEnabled() {
  return g_auto_lower_enabled;
}

void stairAssistUpdate() {
  const uint32_t now_tick = osKernelGetTickCount();
  updateLaser1Judge(now_tick);
  updateLaser3Judge(now_tick);
  updateLaser2Judge(now_tick);
  updateRearPhotogateJudge(now_tick);
  updateDecisionFlags();
}

void stairAssistResetProgress() {
  g_saw_laser2_high_for_climb = false;
  g_saw_laser2_close_for_descend = false;
  g_front_photogate_descend_lower_latched = false;
  g_rear_photogate_climb_lower_latched = false;
  g_rear_photogate_descend_high_latched = false;
  g_front_photogate_unblocked_since_tick = 0U;
  g_front_photogate_unblocked_hold_ready = false;
  g_debug.saw_laser2_high_for_climb = false;
  g_debug.saw_laser2_close_for_descend = false;
  g_debug.front_photogate_descend_lower_latched = false;
  g_debug.rear_photogate_climb_lower_latched = false;
  g_debug.rear_photogate_descend_high_latched = false;
  g_debug.suggest_climb_up = false;
  g_debug.suggest_descend_high = false;
  g_debug.laser1_auto_lower_count = 0;
  g_debug.laser1_descend_ready_count = 0;
  g_debug.laser1_descend_lower_count = 0;
  g_debug.laser1_go_edge_count = 0;
  g_debug.laser3_near_count = 0;
  g_debug.laser3_edge_count = 0;
  g_debug.laser3_go_edge_count = 0;
  g_debug.laser2_climb_high_count = 0;
  g_debug.laser2_descend_lower_count = 0;
  g_debug.should_lower_after_climb = false;
  g_debug.should_lower_after_descend = false;
  g_stair_front_descend_lower_latched = 0U;
  g_stair_laser2_go_to_edge_low_ready = 0U;
  g_stair_laser2_descend_lower_ready = 0U;
  g_stair_go_to_edge_low_ready = 0U;
  g_stair_should_lower_after_descend = 0U;
}

StairAssistLaser1State stairAssistLaser1State() {
  return g_laser1_state;
}

StairAssistLaser2State stairAssistLaser2State() {
  return g_laser2_state;
}

bool stairAssistSuggestClimbUp() {
  return g_debug.suggest_climb_up;
}

bool stairAssistSuggestDescendHighMode() {
  return g_debug.suggest_descend_high;
}

bool stairAssistSuggestDescendEdgeReady() {
  return g_debug.suggest_descend_edge_ready;
}

bool stairAssistSuggestGoToEdgeHigh() {
  return g_enabled &&
         ((g_debug.laser1_go_edge_count >= kStableFrames) ||
          (g_debug.laser3_go_edge_count >= kStableFrames));
}

bool stairAssistSuggestGoToEdgeLow() {
  return g_enabled &&
         ((g_stair_laser2_go_to_edge_low_ready != 0U) ||
          g_front_photogate_unblocked_hold_ready);
}

bool stairAssistShouldLowerAfterClimbAdvance() {
  return g_debug.should_lower_after_climb;
}

bool stairAssistShouldLowerAfterDescendRetreat() {
  return g_debug.should_lower_after_descend;
}

const StairAssistDebug &stairAssistDebug() {
  return g_debug;
}
