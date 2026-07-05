/**
 * @file control_task.h
 * @author 大帅将军
 * @brief
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "control_task.h"

#include "NavProtocol.hpp"
#include "chassis_task.h"
#include "lift_task.h"
#include "merlin_map/merlin_map.h"
#include "pid_controller.h"
#include "state_machine_task.h"
#include "stair_assist.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "tracking.h"
#include "waypoint_navigator.hpp"

extern PID_t pid_x;
extern PID_t pid_y;
extern PID_t pid_yaw;

osThreadId_t ControlTaskHandle;

TypedTopicPublisher<pub_chassis_cmd> chassis_data_pub("chassis_cmd");
pub_chassis_cmd chassis_cmd{};

TypedTopicPublisher<pub_lift_cmd> lift_data_pub("lift_cmd");
pub_lift_cmd lift_cmd{};

TypedTopicPublisher<pub_arm_cmd> arm_data_pub("arm_cmd");
pub_arm_cmd arm_cmd{};

static TypedTopicSubscriber<pub_Xbox_Data> control_xbox_sub("xbox", 8);
pub_Xbox_Data control_xbox_cmd{};

static bool xbox_mode_last = false;
static bool xbox_view_last = false;
static bool xbox_lb_last = false;
static bool xbox_rb_last = false;
static bool xbox_x_last = false;
static bool xbox_y_last = false;
static bool xbox_a_last = false;
static bool xbox_b_last = false;
static bool xbox_rt_last = false;
static bool xbox_ls_last = false;
static bool xbox_rs_last = false;
static bool xbox_y_last_for_stair = false;
static bool xbox_a_last_for_stair = false;
static bool xbox_lt_pressed_last = false;
static TickType_t xbox_x_press_tick = 0;
static TickType_t xbox_y_press_tick = 0;
static TickType_t xbox_a_press_tick = 0;
static TickType_t xbox_b_press_tick = 0;
static TickType_t xbox_lt_press_tick = 0;
static bool stair_assist_high_request_latched = false;
static bool stair_assist_low_request_latched = false;

constexpr TickType_t kLiftTapTimeout = pdMS_TO_TICKS(300);
constexpr uint16_t kLtTriggerThreshold = 800U;
constexpr TickType_t kLtLongPressThreshold = pdMS_TO_TICKS(800);
constexpr uint16_t kR1TriggerThreshold = 1000U;

namespace {

enum class LtAction : uint8_t {
  None = 0,
  GoToEdge,
  ReturnToCenter,
};

}  // namespace

void Xbox_Data_Process() {
  if (ABS(control_xbox_cmd.joyLVert - 32767) > 2300) {
    chassis_cmd.linear_x_ =
        -(int)(control_xbox_cmd.joyLVert - 32767) / 32767.0f * MAX_VELOCITY;
  } else {
    chassis_cmd.linear_x_ = 0.0f;
  }

  if (ABS(control_xbox_cmd.joyLHori - 32767) > 2300) {
    chassis_cmd.linear_y_ =
        (int)(control_xbox_cmd.joyLHori - 32767) / 32767.0f * MAX_VELOCITY;
  } else {
    chassis_cmd.linear_y_ = 0.0f;
  }

  if (ABS(control_xbox_cmd.joyRHori - 32767) > 2300) {
    chassis_cmd.omega_ =
        -(int)(control_xbox_cmd.joyRHori - 32767) / 32767.0f *
        MAX_ROTATION_VELOCITY;
  } else {
    chassis_cmd.omega_ = 0.0f;
  }
}

void Lift_Data_Process() {
  if (control_xbox_cmd.btnY && !xbox_y_last) {
    xbox_y_press_tick = xTaskGetTickCount();
  }

  lift_cmd.request_high = false;
  if (!control_xbox_cmd.btnY && xbox_y_last) {
    if ((xTaskGetTickCount() - xbox_y_press_tick) < kLiftTapTimeout) {
      lift_cmd.request_high = true;
    }
  }
  xbox_y_last = control_xbox_cmd.btnY;

  if (control_xbox_cmd.btnA && !xbox_a_last) {
    xbox_a_press_tick = xTaskGetTickCount();
  }

  lift_cmd.request_low = false;
  if (!control_xbox_cmd.btnA && xbox_a_last) {
    if ((xTaskGetTickCount() - xbox_a_press_tick) < kLiftTapTimeout) {
      lift_cmd.request_low = true;
    }
  }
  xbox_a_last = control_xbox_cmd.btnA;

  lift_cmd.lift_up = control_xbox_cmd.btnY;
  lift_cmd.lift_down = control_xbox_cmd.btnA;

  if (ABS(control_xbox_cmd.joyRVert - 32767) > 2000) {
    lift_cmd.lift_2006_input =
        (int)(control_xbox_cmd.joyRVert - 32767) / 32767.0f *
        MAX_LIFT_VELOCITY;
  } else {
    lift_cmd.lift_2006_input = 0.0f;
  }
}

static void Lift_2006_Axis_Process() {
  if (ABS(control_xbox_cmd.joyRVert - 32767) > 2000) {
    lift_cmd.lift_2006_input =
        (int)(control_xbox_cmd.joyRVert - 32767) / 32767.0f *
        MAX_LIFT_VELOCITY;
  } else {
    lift_cmd.lift_2006_input = 0.0f;
  }
}

static void Arm_Data_Process() {
  if (control_xbox_cmd.btnX && !xbox_x_last) {
    xbox_x_press_tick = xTaskGetTickCount();
  }

  arm_cmd.update = false;
  if (!control_xbox_cmd.btnX && xbox_x_last) {
    if ((xTaskGetTickCount() - xbox_x_press_tick) < kLiftTapTimeout) {
      arm_cmd.update = true;
    }
  }
  xbox_x_last = control_xbox_cmd.btnX;

  if (control_xbox_cmd.btnB && !xbox_b_last) {
    xbox_b_press_tick = xTaskGetTickCount();
  }

  arm_cmd.fetch = false;
  if (!control_xbox_cmd.btnB && xbox_b_last) {
    if ((xTaskGetTickCount() - xbox_b_press_tick) < kLiftTapTimeout) {
      arm_cmd.fetch = true;
    }
  }
  xbox_b_last = control_xbox_cmd.btnB;
}

static bool consumeModeSwitch(bool current_state) {
  const bool rising_edge = current_state && !xbox_mode_last;
  xbox_mode_last = current_state;
  return rising_edge;
}

static bool consumeButtonRisingEdge(bool current_state, bool *last_state) {
  const bool rising_edge = current_state && !(*last_state);
  *last_state = current_state;
  return rising_edge;
}

static bool consumeTriggerRisingEdge(uint16_t current_value,
                                     uint16_t threshold,
                                     bool *last_state) {
  const bool current_state = current_value > threshold;
  const bool rising_edge = current_state && !(*last_state);
  *last_state = current_state;
  return rising_edge;
}

static LtAction consumeLtAction(uint16_t current_value) {
  const bool pressed = current_value > kLtTriggerThreshold;

  if (pressed && !xbox_lt_pressed_last) {
    xbox_lt_press_tick = xTaskGetTickCount();
  }

  LtAction action = LtAction::None;
  if (!pressed && xbox_lt_pressed_last) {
    const TickType_t hold_ticks = xTaskGetTickCount() - xbox_lt_press_tick;
    action = (hold_ticks >= kLtLongPressThreshold) ? LtAction::ReturnToCenter
                                                   : LtAction::GoToEdge;
  }

  xbox_lt_pressed_last = pressed;
  return action;
}

static bool manualActionReady() {
  return !nav_control::auto_enabled &&
         !nav_control::high_mode_active &&
         (stairWaypointStep() == 0U);
}

static bool manualLift2006Ready() {
  return !nav_control::auto_enabled &&
         nav_control::high_mode_active &&
         (stairWaypointStep() == 0U);
}

static void updateStairAssistSwitch() {
  if (consumeButtonRisingEdge(control_xbox_cmd.btnLS, &xbox_ls_last)) {
    const StairAssistMode next_mode =
        (stairAssistMode() == StairAssistMode::ClimbUp)
            ? StairAssistMode::Descend
            : StairAssistMode::ClimbUp;
    stairAssistSetMode(next_mode);
    stair_assist_high_request_latched = false;
    stair_assist_low_request_latched = false;
  }

  if (consumeButtonRisingEdge(control_xbox_cmd.btnRS, &xbox_rs_last)) {
    const bool next_enabled = !stairAssistEnabled();
    stairAssistSetEnabled(next_enabled);
    stairAssistSetAutoLowerEnabled(next_enabled);
    stair_assist_high_request_latched = false;
    stair_assist_low_request_latched = false;
  }
}

static void applyManualStairAssist() {
  stairAssistUpdate();

  if (highModeActive()) {
    stair_assist_high_request_latched = false;
  } else {
    stair_assist_low_request_latched = false;
  }

  if (!stairAssistEnabled()) {
    stair_assist_high_request_latched = false;
    stair_assist_low_request_latched = false;
    return;
  }

  if (nav_control::auto_enabled) {
    return;
  }

  if (!highModeActive()) {
    const bool should_request_high =
        (stairAssistMode() == StairAssistMode::ClimbUp)
            ? stairAssistSuggestClimbUp()
            : stairAssistSuggestDescendHighMode();

    if (!stair_assist_high_request_latched && should_request_high) {
      lift_cmd.request_high = true;
      stair_assist_high_request_latched = true;
      stair_assist_low_request_latched = false;
    }
    return;
  }

  if (!stairAssistAutoLowerEnabled()) {
    return;
  }

  if (stair_assist_low_request_latched) {
    return;
  }

  const bool should_request_low =
      (stairAssistMode() == StairAssistMode::ClimbUp)
          ? stairAssistShouldLowerAfterClimbAdvance()
          : stairAssistShouldLowerAfterDescendRetreat();

  if (should_request_low) {
    lift_cmd.request_low = true;
    stair_assist_low_request_latched = true;
  }
}

void controlInit() {
  if (!chassis_data_pub.IsValid()) {
    return;
  }
  if (!control_xbox_sub.IsValid()) {
    return;
  }
  if (!lift_data_pub.IsValid()) {
    return;
  }
  if (!arm_data_pub.IsValid()) {
    return;
  }

  stairAssistInit();
  merlin_map::init();
}

void controlTask(void *argument) {
  (void)argument;
  TickType_t currentTime = xTaskGetTickCount();

  controlInit();
  for (;;) {
    if (control_xbox_sub.TryGet(&control_xbox_cmd)) {
      updateStairAssistSwitch();

      if (consumeModeSwitch(control_xbox_cmd.btnXbox) && manualActionReady()) {
        stairWaypointGoToFront();
        vTaskDelayUntil(&currentTime, 5);
        continue;
      }

      if (consumeButtonRisingEdge(control_xbox_cmd.btnView, &xbox_view_last)) {
        merlin_map::identifyCurrentCell(nav_control::current_x,
                                        nav_control::current_y);
      }

      if (!nav_control::auto_enabled) {
        const bool stair_y_pressed =
            consumeButtonRisingEdge(control_xbox_cmd.btnY,
                                    &xbox_y_last_for_stair);
        const bool stair_a_pressed =
            consumeButtonRisingEdge(control_xbox_cmd.btnA,
                                    &xbox_a_last_for_stair);
        const LtAction lt_action = consumeLtAction(control_xbox_cmd.trigLT);
        const bool climb_r1_pressed =
            consumeTriggerRisingEdge(control_xbox_cmd.trigRT,
                                     kR1TriggerThreshold,
                                     &xbox_rt_last);

        if (stairWaypointArmed() && manualActionReady()) {
          if (stair_y_pressed) {
            chassis_action::start_climb_upstairs();
            vTaskDelayUntil(&currentTime, 5);
            continue;
          }
          if (stair_a_pressed) {
            chassis_action::start_climb_downstairs();
            vTaskDelayUntil(&currentTime, 5);
            continue;
          }
          if (lt_action == LtAction::GoToEdge) {
            chassis_action::start_go_to_edge();
            vTaskDelayUntil(&currentTime, 5);
            continue;
          }
          if (lt_action == LtAction::ReturnToCenter &&
              stairWaypointLevel() > 0U) {
            chassis_action::start_return_to_center();
            vTaskDelayUntil(&currentTime, 5);
            continue;
          }
        }

        if (climb_r1_pressed && manualActionReady()) {
          chassis_action::start_climb_R1();
          vTaskDelayUntil(&currentTime, 5);
          continue;
        }

        if (consumeButtonRisingEdge(control_xbox_cmd.btnLB, &xbox_lb_last) &&
            manualActionReady()) {
          chassis_action::turn_left_90_deg();
          vTaskDelayUntil(&currentTime, 5);
          continue;
        }

        if (consumeButtonRisingEdge(control_xbox_cmd.btnRB, &xbox_rb_last) &&
            manualActionReady()) {
          chassis_action::turn_right_90_deg();
          vTaskDelayUntil(&currentTime, 5);
          continue;
        }

        if (manualActionReady()) {
          Xbox_Data_Process();
          chassis_cmd.nav_mode_ = false;
          chassis_data_pub.Publish(chassis_cmd);
        }

        if (stairWaypointStep() == 0U) {
          if (!stairWaypointArmed()) {
            Lift_Data_Process();
          } else if (manualLift2006Ready()) {
            lift_cmd.request_high = false;
            lift_cmd.request_low = false;
            lift_cmd.lift_up = false;
            lift_cmd.lift_down = false;
            Lift_2006_Axis_Process();
          } else {
            lift_cmd.request_high = false;
            lift_cmd.request_low = false;
            lift_cmd.lift_up = false;
            lift_cmd.lift_down = false;
            lift_cmd.lift_2006_input = 0.0f;
          }
          applyManualStairAssist();
          lift_data_pub.Publish(lift_cmd);
        }

        Arm_Data_Process();
        arm_data_pub.Publish(arm_cmd);
      }
    }

    vTaskDelayUntil(&currentTime, 5);
  }
}
