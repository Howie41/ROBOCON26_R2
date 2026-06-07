#pragma once

#include "UartPort.hpp"
#include "UsbPort.hpp"
#include "topic_pool.h"
#include "topics.hpp"
#include "transfer_protocol.hpp"
#include "verification_algorithm.hpp"

#include <cstdint>
#include "state_machine_task.h"

//消息类型
enum class PcCmd : uint16_t {
  tail_claw_msg = 0x0001,
  tail_claw_msg_flase = 0x0002,
  tail_claw_msg_success = 0x0003,
// ---- 上位机→下位机: 导航指令 (0x01xx) ----
  nav_position = 0x0101, // 上位机上报当前位置
  nav_target = 0x0102,// 上位机下发目标点
  nav_climb_up = 0x0103, // 上台阶指令
  nav_climb_down = 0x0104,// 下台阶指令
  nav_emergency_stop = 0x0105,// 紧急停止指令
  nav_enter_high = 0x0106,// 进入高位指令
  nav_enter_low = 0x0107,// 进入低位指令
//---- 下位机→上位机: 导航指令 (0x02xx)
  nav_arrived = 0x0201,// 到达目标点
  nav_high_enter = 0x0202,// 进入高位
  nav_high_exit = 0x0203,// 进入低位
  nav_climb_up_ok = 0x0204,// 上台阶完成
  nav_climb_down_ok = 0x0205,// 下台阶完成
  nav_stop_ok = 0x0206,// 紧急停止完成
  nav_stair_hd_start = 0x0207,// // 2006 高位驱动开始
  nav_stair_hd_done = 0x0208,//  2006 高位驱动到达
  nav_stair_cd_start = 0x0209, // 去中心驱动开始
  nav_stair_cd_done = 0x020A, // 去中心驱动到达

  qr_code_parsed = 0xAA00,// 上位机解析到二维码指令
};
class PcCom {
public:
  using Packet=gdut::data_packet<gdut::crc16_algorithm> ;
  using Manager=gdut::packet_manager<gdut::crc16_algorithm>;

  explicit PcCom(UsbPort &usb):usb_(&usb){}
  explicit PcCom(UartPort &uart):uart_(&uart){}
  

  void init();
  void ProcessTx();
  void ProcessRx();

private:
  void OnPacket(Packet packet);

  template<typename T>
  bool send(uint16_t code,const T &msg);
  bool send(uint16_t code);  // 无负载版本

private:
  Manager manager_;
  UartPort *uart_{nullptr};
  UsbPort *usb_={nullptr};

  // 示例: tail_claw 收发
  TypedTopicPublisher<tail_claw_msg> pc_tail_claw_pub_{"pc_tail_claw_pub"};
  TypedTopicSubscriber<tail_claw_msg> pc_tail_claw_sub_{"pc_tail_claw_sub", 8};
  
    // 导航事件订阅: lift_task / NavProtocol 发布事件 → ProcessTx 发送到上位机
    // 队列深度 4，足够缓冲连续事件
  TypedTopicSubscriber<pc_nav_event_t> pc_nav_event_sub_{"pc_nav_event_pub", 4};
  
    // 二维码解析
  TypedTopicPublisher<pub_qr_code_parsed> pc_qr_code_pub_{"qr_code_parsed"};

    // 请求路径规划动作
    TypedTopicSubscriber<bool> pc_path_cmd_request_sub_{"pc_path_cmd_request", 1};
    TypedTopicPublisher<PathCmd> pc_path_cmd_pub_{"pc_path_cmd"};
};
