/**
 * @file topic_pool.h
 * @author 大帅将军 ，Keten (2863861004@qq.com)
 * @brief
 * 模块所依赖的数据类型结构体，有些模块会依赖这些数据类型结构体进行数据传输，所以移植module层都
 *        必须携带这个包
 * @version 0.1
 * @date 2024-10-03
 *
 * @copyright Copyright (c) 2024
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once
#include "fdcan.h"
#include "usart.h"
#include <cstdint>
#include <stdbool.h>

#pragma pack(1)

typedef struct {
  UART_HandleTypeDef *huart; // 串口句柄
  uint16_t len;              // 数据长度
  void *data_addr; // 数据地址，使用时把地址赋值给这个指针，数值强转为uint8_t
} UART_TxMsg;

typedef struct {
  bool btnY;
  bool btnA;
  bool btnShare;
  bool btnView;
  bool btnMenu;
  bool btnXbox;
  bool btnLB;
  bool btnRB;
  bool btnLS;
  bool btnRS;
  uint16_t trigLT;
  uint16_t trigRT;
  bool btnDirUp;
  bool btnDirDown;
  bool btnDirLeft;
  bool btnDirRight;
  bool btnB;
  bool btnX;
  uint16_t joyLHori;
  uint16_t joyLVert;
  uint16_t joyRHori;
  uint16_t joyRVert;
  // 这里填写你需要传输的Xbox按键摇杆等数据
  // bool btnY;
  // bool btnY_last;
  //......

} pub_Xbox_Data;

// 底盘运动指令
typedef struct {
  float linear_x_;
  float linear_y_;
  float omega_;
  bool nav_mode_;
} pub_chassis_cmd;


typedef struct{
 bool lift_up;        //按住Y 持续上升
 bool lift_down;      //按住A 持续下降
 float lift_2006_input;

 bool request_high;  //按一下Y 请求升高到高位
 bool request_low;   //按一下A 请求降低到低位
} pub_lift_cmd;


typedef struct {
  float forward_speed;  // 前进速度 (RPM)，两轮同向
  float omega;          // 差速旋转 (RPM)，正值=左轮加速右轮减速
  bool active;          // true=2006自动导航激活
  bool request_lower;   // 到达目标后请求降回低位
} pub_high_nav_cmd;
//尾部夹爪控制指令
enum class TailClawMode : uint8_t {
  Manual = 0,            // 手动模式 根据Xbox输入控制夹爪移动和翻转
  AutoAlign,             // 自动对齐模式 根据Xbox输入控制夹爪移动和翻转
  Hold,                   // 锁定模式 锁定当前位置
  Disabled,               // 禁用模式 禁用所有功能
};
// 这个结构体既用作状态机发给tail_claw_task的命令，也用作上位机发给tail_claw_task的命令
typedef struct {
  TailClawMode mode;                 // 尾部夹爪模式
  bool set_roll_target;              // true=设置旋转目标
  float roll_target_deg;             // 旋转目标角度 (度)
  bool set_weapon_claw;               // true=设置武器夹爪
  bool weapon_claw_close;             // true=武器夹爪关闭

  bool set_air_pump;
  bool air_pump_on;
} pub_tail_claw_cmd;
// 上位机发给tail_claw_task的距离数据
struct tail_claw_msg {
   int16_t distance;
};

// ===== PcCom 二进制导航协议结构体 =====

// 上位机→下位机: 上报当前激光雷达坐标 (消息码 0x0101)
typedef struct {
  int16_t x;    // 世界坐标系 X (mm)
  int16_t y;    // 世界坐标系 Y (mm)
  int16_t yaw;  // 上位机计算的 yaw 角度 (度*100 或 度)，仅供记录，实际导航用陀螺仪
} pc_nav_position_t;

// 上位机→下位机: 下发目标点 (消息码 0x0102)
typedef struct {
  int16_t x;    // 目标 X (mm)
  int16_t y;    // 目标 Y (mm)
  int16_t yaw;  // 目标 yaw (度)
} pc_nav_target_t;

// 下位机→上位机: 事件通知 (消息码 0x0201~0x0205)
typedef struct {
  uint16_t event_code;  // 事件码，对应 PcCmd 中的 nav_* 枚举值
} pc_nav_event_t;

// 二维码扫描结果
typedef struct {
  uint8_t data;
} pub_qr_code_parsed; // topic = "qr_code_parsed"

#pragma pack()
