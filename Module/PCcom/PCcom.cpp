#include "PCcom.hpp"
#include "NavProtocol.hpp"
#include "lift_task.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "waypoint_navigator.hpp"

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
  g_stair_ctx.active = false;
  g_stair_ctx.phase = 0;

  nav_control::auto_enabled = false;
  nav_control::arrived = false;
  nav_control::target_active = false;
  nav_control::arrival_reported = false;
  nav_control::target_x = nav_control::current_x;
  nav_control::target_y = nav_control::current_y;
  nav_control::target_yaw = nav_control::current_yaw;

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

    case static_cast<uint16_t>(PcCmd::nav_climb_up):
      stairSMStart(true);
      break;
    case static_cast<uint16_t>(PcCmd::nav_climb_down):
      stairSMStart(false);
      break;
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

    default:
      break;
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

  tail_claw_msg claw_start_msg{};
  if (tail_claw_weapon_event_sub_.TryGet(&claw_start_msg)) {
      send(static_cast<uint16_t>(PcCmd::tail_claw_weapon_start), claw_start_msg);
  }
  /*if (tail_claw_rod_event_sub_.TryGet(&claw_start_msg)) {
    bool ok=send(static_cast<uint16_t>(PcCmd::tail_claw_rod_start), claw_start_msg);
  }*/
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
