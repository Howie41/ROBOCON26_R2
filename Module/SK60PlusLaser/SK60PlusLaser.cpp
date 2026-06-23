#include "SK60PlusLaser.hpp"

#include <cstdio>
#include <cstring>

HAL_StatusTypeDef SK60PlusLaser::init() {
  latest_result_ = {};
  return HAL_OK;
}

HAL_StatusTypeDef SK60PlusLaser::triggerSingleMeasure() {
  clearResultValidity();

  uint8_t cmd[9] = {
      0xAA,
      address_,
      0x00,
      0x20,
      0x00,
      0x01,
      0x00,
      0x00,
      0x00,
  };
  cmd[8] = checksum(cmd, 8);
  return uart_port_.writeDma(cmd, sizeof(cmd));
}

bool SK60PlusLaser::processFrame(const uint8_t *data, std::size_t len) {
  if (data == nullptr || len < 9 || len > kMaxFrameSize) {
    return false;
  }

  if (data[0] == 0xEE) {
    return parseErrorResult(data, len);
  }

  if (data[0] != 0xAA || data[1] != address_ || data[2] != 0x00 ||
      data[3] != 0x22) {
    return false;
  }

  if (checksum(data, len - 1) != data[len - 1]) {
    return false;
  }

  return parseMeasureResult(data, len);
}

uint8_t SK60PlusLaser::checksum(const uint8_t *data, std::size_t len) {
  uint32_t sum = 0;
  for (std::size_t i = 1; i < len; ++i) {
    sum += data[i];
  }
  return static_cast<uint8_t>(sum & 0xFFU);
}

bool SK60PlusLaser::parseMeasureResult(const uint8_t *data, std::size_t len) {
  if (data == nullptr || len < 13) {
    return false;
  }

  const uint32_t distance_u32 =
      (static_cast<uint32_t>(data[6]) << 24) |
      (static_cast<uint32_t>(data[7]) << 16) |
      (static_cast<uint32_t>(data[8]) << 8) |
      static_cast<uint32_t>(data[9]);
  const uint16_t quality =
      (static_cast<uint16_t>(data[10]) << 8) | static_cast<uint16_t>(data[11]);

  latest_result_.valid = true;
  latest_result_.is_error = false;
  latest_result_.distance_mm = static_cast<int32_t>(distance_u32);
  latest_result_.frame_count++;
  std::memset(latest_result_.error_text, 0, sizeof(latest_result_.error_text));
  (void)quality;
  return true;
}

bool SK60PlusLaser::parseErrorResult(const uint8_t *data, std::size_t len) {
  if (data == nullptr || len < 9) {
    return false;
  }

  clearResultValidity();
  latest_result_.valid = true;
  latest_result_.is_error = true;
  latest_result_.distance_mm = 0;
  latest_result_.frame_count++;
  std::snprintf(latest_result_.error_text, sizeof(latest_result_.error_text),
                "%02X%02X", data[6], data[7]);
  return true;
}

void SK60PlusLaser::clearResultValidity() {
  latest_result_.valid = false;
  latest_result_.is_error = false;
  std::memset(latest_result_.error_text, 0, sizeof(latest_result_.error_text));
}
