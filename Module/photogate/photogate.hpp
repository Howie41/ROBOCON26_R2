#pragma once

#include <cstdint>

namespace photogate {

enum class GateId : uint8_t {
  Front = 0,
  Rear = 1,
};

struct GateDebugState {
  bool raw_blocked{false};
  bool blocked{false};
  bool blocked_edge{false};
  bool unblocked_edge{false};
  uint8_t stable_count{0};
};

struct DebugState {
  GateDebugState front{};
  GateDebugState rear{};
};

void init();
void update();

bool blocked(GateId id);
bool unblocked(GateId id);

bool blockedEdge(GateId id);
bool unblockedEdge(GateId id);

const DebugState& debugState();

}  // namespace photogate

extern "C" {

extern volatile uint8_t g_photogate_front_blocked;
extern volatile uint8_t g_photogate_front_unblocked;
extern volatile uint8_t g_photogate_front_blocked_edge;
extern volatile uint8_t g_photogate_front_unblocked_edge;

extern volatile uint8_t g_photogate_rear_blocked;
extern volatile uint8_t g_photogate_rear_unblocked;
extern volatile uint8_t g_photogate_rear_blocked_edge;
extern volatile uint8_t g_photogate_rear_unblocked_edge;

}
