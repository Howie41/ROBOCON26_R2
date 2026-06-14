/**
 * @file state_machine_task.cpp
 * @author zhy (Howie41)
 * @brief 状态机任务
 * @date 2026-05-24
 */

#include "cmsis_os2.h"

#include <atomic>
#include <cstdint>

#include "infrared_com.hpp"
#include "state_machine_task.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "waypoint_navigator.hpp"

osThreadId_t StateMachineTaskHandle;

static std::atomic<RobotState> current_state{RobotState::begin};

template <typename T>
void wait_until(T &&condition, uint32_t delay_ms = 100U) {
  while (!condition()) {
    osDelay(delay_ms);
  }
}

TypedTopicSubscriber<pub_infrared_msg> infrared_sub(
    InfraredModule::INFRARED_MSG_TOPIC, 1);
TypedTopicSubscriber<pub_qr_code_parsed> qr_code_sub("qr_code_parsed", 1);

void change_state_to(RobotState new_state) { current_state.store(new_state); }

RobotState get_current_state() { return current_state.load(); }

bool state_machine_idle() { return current_state.load() == RobotState::begin; }

void clean_previous_cmd() {
  pub_infrared_msg temp_im{};
  pub_qr_code_parsed temp_qr{};
  infrared_sub.TryGet(&temp_im);
  qr_code_sub.TryGet(&temp_qr);
}

uint8_t get_cmd_from_r1() {
  uint8_t cmd{0x00};
  pub_infrared_msg infrared_msg{.data = 0x00};
  pub_qr_code_parsed qr_code_msg{.data = 0x00};

  if (infrared_sub.TryGet(&infrared_msg)) {
    cmd = infrared_msg.data;
  }
  if (qr_code_sub.TryGet(&qr_code_msg)) {
    cmd = qr_code_msg.data;
  }
  return cmd;
}

void stateMachineTask(void *argument) {
  (void)argument;
  for (;;) {
    switch (current_state.load()) {
#ifdef MATCH_CWTY
      case RobotState::begin: {
        break;
      }

      case RobotState::go_to_SHR: {
        break;
      }

      case RobotState::go_to_stair_front: {
        stairWaypointGoToFront();
        change_state_to(RobotState::begin);
        break;
      }

      case RobotState::test_stair_up: {
        stairWaypointRunUp();
        change_state_to(RobotState::begin);
        break;
      }

      case RobotState::test_stair_down: {
        stairWaypointRunDown();
        change_state_to(RobotState::begin);
        break;
      }

      case RobotState::aim_at_weapon: {
        break;
      }

      case RobotState::catch_weapon: {
        break;
      }

      case RobotState::rotate_weapon_claw: {
        break;
      }

      case RobotState::wait_for_cmd: {
        clean_previous_cmd();
        wait_until([&]() -> bool {
          switch (get_cmd_from_r1()) {
            case 0x1A:
              change_state_to(RobotState::go_to_SHR);
              return true;
            case 0x1B:
              change_state_to(RobotState::go_to_MF);
              return true;
            default:
              return false;
          }
        });
        break;
      }

#elif MATCH_JGCB

#endif

      default:
        break;
    }

    osDelay(1);
  }
}
