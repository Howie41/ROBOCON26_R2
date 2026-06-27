#include "LaserMeasure.hpp"

#include <cstring>

HAL_StatusTypeDef LaserMeasure::init() {
  latest_result_ = {};
  return HAL_OK;
}

HAL_StatusTypeDef LaserMeasure::triggerSingleMeasure() {
  clearResultValidity();

  uint8_t cmd[4] = {address_, 0x06, 0x02, 0x00};
  cmd[3] = checksum(cmd, 3);

  return uart_port_.writeDma(cmd, sizeof(cmd));
}

bool LaserMeasure::processFrame(const uint8_t *data, std::size_t len) {  
  if (data == nullptr || len < 5 || len > kMaxFrameSize) {
    return false;
  }

  if (data[0] != address_) {
    return false;
  }

  if (data[1] != 0x06 || data[2] != 0x82) {
    return false;
  }

  if (checksum(data, len - 1) != data[len - 1]) {
    return false;
  }

  const uint8_t *payload = data + 3;
  const std::size_t payload_len = len - 4;

  if (payload_len >= 3 && payload[0] == 'E' && payload[1] == 'R' &&
      payload[2] == 'R') {
    if (!parseErrorPayload(payload, payload_len)) {
      return false;
    }
  } else {
    if (!parseDistancePayload(payload, payload_len)) {
      return false;
    }
  }

  latest_result_.frame_count++;
  latest_result_.valid = true;
  return true;
}

uint8_t LaserMeasure::checksum(const uint8_t *data, std::size_t len) {
  uint32_t sum = 0;
  for (std::size_t i = 0; i < len; ++i) {
    sum += data[i];
  }
  return static_cast<uint8_t>(~sum + 1U);
}

bool LaserMeasure::parseDistancePayload(const uint8_t *payload,
                                        std::size_t len) {
  if (payload == nullptr || len == 0) {
    return false;
  }

  int32_t integer_part = 0;
  int32_t mm_part = 0;
  bool seen_dot = false;
  uint8_t frac_digits = 0;
  uint8_t round_digit = 0;

  for (std::size_t i = 0; i < len; ++i) {
    const uint8_t ch = payload[i];

    if (ch == '.') {
      if (seen_dot) {
        return false;
      }
      seen_dot = true;
      continue;
    }

    if (ch < '0' || ch > '9') {
      return false;
    }

    const int32_t digit = static_cast<int32_t>(ch - '0');

    if (!seen_dot) {
      integer_part = integer_part * 10 + digit;
      continue;
    }

    if (frac_digits < 3) {
      if (frac_digits == 0) {
        mm_part += digit * 100;
      } else if (frac_digits == 1) {
        mm_part += digit * 10;
      } else {
        mm_part += digit;
      }
    } else if (frac_digits == 3) {
      round_digit = static_cast<uint8_t>(digit);
    }

    frac_digits++;
  }

  int32_t distance_mm = integer_part * 1000 + mm_part;
  if (round_digit >= 5) {
    distance_mm += 1;
  }

  latest_result_.is_error = false;
  latest_result_.distance_mm = distance_mm;
  std::memset(latest_result_.error_text, 0, sizeof(latest_result_.error_text));
  return true;
}

bool LaserMeasure::parseErrorPayload(const uint8_t *payload, std::size_t len) {
  if (payload == nullptr || len < 3) {
    return false;
  }

  clearResultValidity();
  latest_result_.valid = true;
  latest_result_.is_error = true;

  const std::size_t copy_len =
      (len < sizeof(latest_result_.error_text) - 1)
          ? len
          : (sizeof(latest_result_.error_text) - 1);

  std::memcpy(latest_result_.error_text, payload, copy_len);
  latest_result_.error_text[copy_len] = '\0';
  return true;
}

void LaserMeasure::clearResultValidity() {
  latest_result_.valid = false;
  latest_result_.is_error = false;
  std::memset(latest_result_.error_text, 0, sizeof(latest_result_.error_text));
}
