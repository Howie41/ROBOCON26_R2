#pragma once

#include "UartPort.hpp"
#include "stm32h7xx_hal_def.h"

#include <cstddef>
#include <cstdint>

class SK60PlusLaser {
public:
  static constexpr uint8_t kDefaultAddress = 0x00;
  static constexpr std::size_t kMaxFrameSize = 13;

  struct MeasureResult {
    bool valid{false};
    bool is_error{false};
    int32_t distance_mm{0};
    uint32_t frame_count{0};
    char error_text[8]{};
  };

  explicit SK60PlusLaser(UartPort &uart_port, uint8_t address = kDefaultAddress)
      : uart_port_(uart_port), address_(address) {}

  HAL_StatusTypeDef init();
  HAL_StatusTypeDef triggerSingleMeasure();
  bool processFrame(const uint8_t *data, std::size_t len);

  const MeasureResult &latestResult() const { return latest_result_; }
  uint8_t address() const { return address_; }

private:
  static uint8_t checksum(const uint8_t *data, std::size_t len);
  bool parseMeasureResult(const uint8_t *data, std::size_t len);
  bool parseErrorResult(const uint8_t *data, std::size_t len);
  void clearResultValidity();

private:
  UartPort &uart_port_;
  uint8_t address_{kDefaultAddress};
  MeasureResult latest_result_{};
};
