#include "PCcom.hpp"
#include "NavProtocol.hpp"
#include "cmsis_os2.h"
#include "lift_task.h"
#include "field_waypoints.hpp"
#include "state_machine_task.h"
#include "waypoint_navigator.hpp"
#include "topic_pool.h"
#include "topics.hpp"
#include "waypoint_navigator.hpp"
#include "logger.hpp"
#include "robot_task.h"

#include <codecvt>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <sys/types.h>

namespace {

TypedTopicPublisher<pub_chassis_cmd> chassis_cmd_pub("chassis_cmd");
TypedTopicPublisher<pc_nav_event_t> pc_nav_event_pub("pc_nav_event_pub");

void applyNavTarget(int16_t x, int16_t y, int16_t yaw) {
  nav_control::target_x = x;
  nav_control::target_y = y;
  nav_control::target_yaw = yaw;
  nav_control::auto_enabled = true;
  nav_control::arrived = false;
  nav_control::target_active = true;
  nav_control::arrival_reported = false;
  nav_control::resetAllPIDs();
}

void handleEmergencyStop() {
  nav_control::auto_enabled = false;
  nav_control::arrived = false;
  nav_control::target_active = false;
  nav_control::arrival_reported = false;
  nav_control::target_x = nav_control::current_x;
  nav_control::target_y = nav_control::current_y;
  nav_control::target_yaw = nav_control::current_yaw;
  // change_state_to(RobotState::begin);

  pub_chassis_cmd stop_cmd{};
  stop_cmd.nav_mode_ = true;
  chassis_cmd_pub.Publish(stop_cmd);

  pc_nav_event_t evt{static_cast<uint16_t>(PcCmd::nav_stop_ok)};
  pc_nav_event_pub.Publish(evt);
}

}  // namespace

void PcCom::init() {
  manager_.set_send_function([this](const uint8_t *begin,
                                    const uint8_t *end) noexcept {
    const size_t len = static_cast<size_t>(end - begin);

    if (uart_ != nullptr) {
      uart_->write(begin, len);
    } else if (usb_ != nullptr) {
      usb_->WriteAsync(begin, len);
    }
  });

  manager_.set_receive_function(
      [this](Packet packet) noexcept { OnPacket(std::move(packet)); });
}

void PcCom::ProcessRx() {
  if (usb_ != nullptr) {
    UsbPort::Packet rx{};
    while (usb_->Read(rx)) {
      manager_.receive(rx.data, rx.data + rx.len);
    }
  }

  if (uart_ != nullptr) {
    UartPort::Packet rx{};
    while (uart_->Read(rx)) {
      manager_.receive(rx.data, rx.data + rx.len);
    }
  }
}

void PcCom::OnPacket(Packet packet) {
  switch (packet.code()) {
    case static_cast<uint16_t>(PcCmd::tail_claw_msg): {
      if (packet.body_size() != sizeof(tail_claw_msg)) {
        return;
      }
      tail_claw_msg msg{};
      std::memcpy(&msg, packet.body_data(), sizeof(tail_claw_msg));
      pc_tail_claw_pub_.Publish(msg);
      break;
    }
      /*case static_cast<uint16_t>(PcCmd::tail_claw_msg_weapon):{
      if(packet.body_size()!=sizeof(tail_claw_msg)){
        return;
      }
      tail_claw_msg msg{};
      std::memcpy(&msg,packet.body_data(),sizeof(tail_claw_msg));
      pc_tail_claw_pub_.Publish(msg);
      break;
    }*/
    // ---- 导航: 上位机上报当前位置 (0x0101) ----
    case static_cast<uint16_t>(PcCmd::nav_position):{
      if(packet.body_size()!=sizeof(pc_nav_position_t)){
        return;
      }
      pc_nav_position_t msg{};
      std::memcpy(&msg, packet.body_data(), sizeof(msg));
      nav_control::current_x = msg.x;
      nav_control::current_y = msg.y;
      nav_control::pc_reported_yaw = msg.yaw;
      nav_control::updatePositionTimestamp();
      break;
    }

    case static_cast<uint16_t>(PcCmd::nav_target): {
      if (packet.body_size() != sizeof(pc_nav_target_t)) {
        return;
      }
      pc_nav_target_t msg{};
      std::memcpy(&msg, packet.body_data(), sizeof(msg));
      applyNavTarget(msg.x, msg.y, msg.yaw);
      break;
    }

    // case static_cast<uint16_t>(PcCmd::nav_climb_up):
    //   if (state_machine_idle()) {
    //     change_state_to(RobotState::test_stair_up);
    //   }
    //   break;
    // case static_cast<uint16_t>(PcCmd::nav_climb_down):
    //   if (state_machine_idle()) {
    //     change_state_to(RobotState::test_stair_down);
    //   }
    //   break;
    case static_cast<uint16_t>(PcCmd::nav_enter_high):
      liftRequestHigh();
      break;
    case static_cast<uint16_t>(PcCmd::nav_enter_low):
      liftRequestLow();
      break;
    case static_cast<uint16_t>(PcCmd::nav_emergency_stop):
      handleEmergencyStop();
      break;

    case static_cast<uint16_t>(PcCmd::qr_code_parsed): {
      if (packet.body_size() != sizeof(pub_qr_code_parsed)) {
        return;
      }
      pub_qr_code_parsed qr_code_msg{};
      std::memcpy(&qr_code_msg, packet.body_data(), sizeof(qr_code_msg));
      pc_qr_code_pub_.Publish(qr_code_msg);
      break;
    }

    case static_cast<uint16_t>(PcCmd::startup_config): {
      if (packet.body_size() != sizeof(startup_config)) {
        return;
      }
      startup_config config{};
      std::memcpy(&config, packet.body_data(), sizeof(config));
      pc_startup_config_pub_.Publish(config);
      break;
    }

    default: {
      // 路径规划指令
      if (path_cmd::is_path_cmd(packet.code())) {
        path_cmd::code cmd = static_cast<path_cmd::code>(packet.code());
        pc_path_cmd_pub_.Publish(cmd);
      }
      break;
    }
  }
}

void PcCom::ProcessTx() {
  tail_claw_msg claw_msg{};
  if (pc_tail_claw_sub_.TryGet(&claw_msg)) {
    send(static_cast<uint16_t>(PcCmd::tail_claw_msg), claw_msg);
  }

  pc_nav_event_t nav_event{};
  if(pc_nav_event_sub_.TryGet(&nav_event)){
    send(nav_event.event_code, nav_event);
  }

  // 请求路径规划步骤
  uint16_t request_path_cmd{};
  if (pc_path_cmd_request_sub_.TryGet(&request_path_cmd)) {
    send(static_cast<uint16_t>(path_cmd::code::request), request_path_cmd);
  }
  tail_claw_msg claw_start_msg{};
  if (tail_claw_weapon_event_sub_.TryGet(&claw_start_msg)) {
      send(static_cast<uint16_t>(PcCmd::tail_claw_weapon_start), claw_start_msg);
  }
  // 回应上位机启动配置
  bool startup_config_ack{};
  if (pc_startup_config_ack_sub_.TryGet(&startup_config_ack)) {
    send(static_cast<uint16_t>(PcCmd::startup_config_ack), startup_config_ack);
  }
  LoggerQueue::message msg{};
  if (pc_log_queue_handle != nullptr &&
      osMessageQueueGet(pc_log_queue_handle, &msg, nullptr, 0U) == osOK) {
    char formatted[Logger::BUFFER_LENGTH];
    size_t n = msg.format_to(formatted, sizeof(formatted));
    if (n > 0) {
      send(static_cast<uint16_t>(PcCmd::log_message),
           reinterpret_cast<uint8_t*>(formatted), n);
    }
  }
  // 上位机显示屏幕信息
  screen_display_packet screen_msg{};
  if (pc_screen_display_sub_.TryGet(&screen_msg)) {
    send(static_cast<uint16_t>(PcCmd::screen_display), screen_msg);
  }
}

template <typename T>
bool PcCom::send(uint16_t code, const T &msg) {
  const uint8_t *begin = reinterpret_cast<const uint8_t *>(&msg);

  Packet packet{code, begin, begin + sizeof(T), gdut::build_packet};

  if (!packet) {
    return false;
  }

  manager_.send(packet);
  return true;
}

bool PcCom::send(uint16_t code) {
  const uint8_t *dummy = nullptr;
  Packet packet{code, dummy, dummy, gdut::build_packet};

  if (!packet) {
    return false;
  }

  manager_.send(packet);
  return true;
}

bool PcCom::send(uint16_t code, const uint8_t *data, size_t size) {
  Packet packet{code, data, data + size, gdut::build_packet};

  if (!packet) {
    return false;
  }

  manager_.send(packet);
  return true;
}
