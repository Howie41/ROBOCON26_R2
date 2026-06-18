#pragma once

#include "UartPort.hpp"
#include "stm32h7xx_hal_def.h"

#include <cstddef>
#include <cstdint>

class LaserMeasure {
public:
  static constexpr uint8_t kDefaultAddress = 0x80;
  static constexpr std::size_t kMaxFrameSize = 16;

  struct MeasureResult {
    bool valid{false};// 是否有效（成功测距或明确错误）
    bool is_error{false};// 是否测距失败（有效但不是距离结果）
    int32_t distance_mm{0};// 测距结果，单位毫米，仅当 valid=true 且 is_error=false 时有效
    uint32_t frame_count{0};// 成功解析的帧计数
    char error_text[8]{};// 错误文本，仅当 valid=true 且 is_error=true 时有效
  };

  explicit LaserMeasure(UartPort &uart_port, uint8_t address = kDefaultAddress)
      : uart_port_(uart_port), address_(address) {} // 构造函数，接受 UART 端口和可选的设备地址

  HAL_StatusTypeDef init();// 初始化设备，配置 UART 等

  HAL_StatusTypeDef triggerSingleMeasure();// 触发一次测距，发送命令到设备

  bool processFrame(const uint8_t *data, std::size_t len);// 处理接收到的 UART 数据帧，解析测距结果或错误信息

  const MeasureResult &latestResult() const { return latest_result_; }// 获取最新的测距结果

  uint8_t address() const { return address_; }// 获取设备地址

private:
  static uint8_t checksum(const uint8_t *data, std::size_t len);// 计算校验和，供发送命令时使用
  bool parseDistancePayload(const uint8_t *payload, std::size_t len);// 解析测距结果的有效载荷，更新 latest_result_
  bool parseErrorPayload(const uint8_t *payload, std::size_t len);// 解析错误信息的有效载荷，更新 latest_result_
  void clearResultValidity();// 清除 latest_result_ 的有效性标志，准备接收新的测距结果

private:
  UartPort &uart_port_;// 引用 UART 端口对象，用于发送命令和接收数据
  uint8_t address_{kDefaultAddress};// 设备地址，默认为 0x80
  MeasureResult latest_result_{};// 存储最新的测距结果或错误信息
};
