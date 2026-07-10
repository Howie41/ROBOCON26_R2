#pragma once

class Hwt101IMU {
public:

    float decodeAngleDeg(uint8_t lo, uint8_t hi) {
        const int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo));
        return static_cast<float>(raw) / 32768.0f * 180.0f;
    }

    bool processByte(uint8_t byte) {
        if (index_ == 0U) {
            if (byte == 0x55U) frame_[index_++] = byte;
            return false;
        }
        if (index_ == 1U) {
            if (byte == 0x55U) frame_[0] = byte;
            else frame_[index_++] = byte;
            return false;
        }
        frame_[index_++] = byte;
        if (index_ < kFrameSize) {
            return false;
        }
        uint8_t sum = 0U;
        for (uint8_t i = 0; i < kFrameSize - 1U; ++i) sum = static_cast<uint8_t>(sum + frame_[i]);
        const bool valid = (sum == frame_[kFrameSize - 1U]);
        if (valid && frame_[1] == 0x53U) {
            yaw_deg_ = decodeAngleDeg(frame_[6], frame_[7]);
        }
        index_ = 0;
        return valid;
    }
    inline float get_yaw() const { return yaw_deg_; }

private:
    static constexpr uint8_t kFrameSize = 11;

    uint8_t index_{0};
    float yaw_deg_{0.0f};
};

