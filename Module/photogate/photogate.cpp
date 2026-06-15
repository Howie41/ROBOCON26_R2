#include "photogate.hpp"

#include "main.h"

extern "C" {

volatile uint8_t g_photogate_front_blocked = 0;
volatile uint8_t g_photogate_front_unblocked = 0;
volatile uint8_t g_photogate_front_blocked_edge = 0;
volatile uint8_t g_photogate_front_unblocked_edge = 0;

volatile uint8_t g_photogate_rear_blocked = 0;
volatile uint8_t g_photogate_rear_unblocked = 0;
volatile uint8_t g_photogate_rear_blocked_edge = 0;
volatile uint8_t g_photogate_rear_unblocked_edge = 0;

}

namespace photogate {
namespace {

constexpr uint16_t kFrontPin = GPIO_PIN_14;
constexpr uint16_t kRearPin = GPIO_PIN_15;

// E3ZG/E3Z-D61 is typically used as an NPN open-collector output.
// With the current GPIO pull-up configuration, an active sensor output pulls
// the line low. If field wiring or Light-ON/Dark-ON selection is reversed,
// only this level mapping needs to change.
constexpr GPIO_PinState kActiveLevel = GPIO_PIN_RESET;
constexpr bool kActiveMeansBlocked = true;
constexpr uint8_t kStableSamplesRequired = 3;

struct GateRuntimeState {
  bool raw_blocked{false};
  bool blocked{false};
  bool pending_blocked{false};
  bool blocked_edge{false};
  bool unblocked_edge{false};
  uint8_t stable_count{0};
};

DebugState g_debug_state{};
GateRuntimeState g_front_state{};
GateRuntimeState g_rear_state{};

bool rawActive(GPIO_TypeDef* port, uint16_t pin) {
  return HAL_GPIO_ReadPin(port, pin) == kActiveLevel;
}

GPIO_TypeDef* frontPort() { return GPIOD; }

GPIO_TypeDef* rearPort() { return GPIOD; }

bool rawBlocked(GPIO_TypeDef* port, uint16_t pin) {
  const bool active = rawActive(port, pin);
  return kActiveMeansBlocked ? active : !active;
}

void syncDebugState(const GateRuntimeState& src, GateDebugState* dst) {
  dst->raw_blocked = src.raw_blocked;
  dst->blocked = src.blocked;
  dst->blocked_edge = src.blocked_edge;
  dst->unblocked_edge = src.unblocked_edge;
  dst->stable_count = src.stable_count;
}

void syncGlobalDebugMirrors() {
  g_photogate_front_blocked = g_front_state.blocked ? 1U : 0U;
  g_photogate_front_unblocked = g_front_state.blocked ? 0U : 1U;
  g_photogate_front_blocked_edge = g_front_state.blocked_edge ? 1U : 0U;
  g_photogate_front_unblocked_edge = g_front_state.unblocked_edge ? 1U : 0U;

  g_photogate_rear_blocked = g_rear_state.blocked ? 1U : 0U;
  g_photogate_rear_unblocked = g_rear_state.blocked ? 0U : 1U;
  g_photogate_rear_blocked_edge = g_rear_state.blocked_edge ? 1U : 0U;
  g_photogate_rear_unblocked_edge = g_rear_state.unblocked_edge ? 1U : 0U;
}

void initGateState(GateRuntimeState* state, GPIO_TypeDef* port, uint16_t pin) {
  const bool blocked_now = rawBlocked(port, pin);
  state->raw_blocked = blocked_now;
  state->blocked = blocked_now;
  state->pending_blocked = blocked_now;
  state->blocked_edge = false;
  state->unblocked_edge = false;
  state->stable_count = 0;
}

void updateGateState(GateRuntimeState* state, GPIO_TypeDef* port, uint16_t pin) {
  const bool raw_blocked_now = rawBlocked(port, pin);

  state->blocked_edge = false;
  state->unblocked_edge = false;
  state->raw_blocked = raw_blocked_now;

  if (raw_blocked_now == state->blocked) {
    state->pending_blocked = raw_blocked_now;
    state->stable_count = 0;
    return;
  }

  if (raw_blocked_now != state->pending_blocked) {
    state->pending_blocked = raw_blocked_now;
    state->stable_count = 1;
    return;
  }

  if (state->stable_count < kStableSamplesRequired) {
    ++state->stable_count;
  }

  if (state->stable_count < kStableSamplesRequired) {
    return;
  }

  const bool last_blocked = state->blocked;
  state->blocked = state->pending_blocked;
  state->blocked_edge = !last_blocked && state->blocked;
  state->unblocked_edge = last_blocked && !state->blocked;
  state->stable_count = 0;
}

GateRuntimeState& gateState(GateId id) {
  return (id == GateId::Front) ? g_front_state : g_rear_state;
}

}  // namespace

void init() {
  initGateState(&g_front_state, frontPort(), kFrontPin);
  initGateState(&g_rear_state, rearPort(), kRearPin);
  syncDebugState(g_front_state, &g_debug_state.front);
  syncDebugState(g_rear_state, &g_debug_state.rear);
  syncGlobalDebugMirrors();
}

void update() {
  updateGateState(&g_front_state, frontPort(), kFrontPin);
  updateGateState(&g_rear_state, rearPort(), kRearPin);
  syncDebugState(g_front_state, &g_debug_state.front);
  syncDebugState(g_rear_state, &g_debug_state.rear);
  syncGlobalDebugMirrors();
}

bool blocked(GateId id) { return gateState(id).blocked; }

bool unblocked(GateId id) { return !gateState(id).blocked; }

bool blockedEdge(GateId id) { return gateState(id).blocked_edge; }

bool unblockedEdge(GateId id) { return gateState(id).unblocked_edge; }

const DebugState& debugState() { return g_debug_state; }

}  // namespace photogate
