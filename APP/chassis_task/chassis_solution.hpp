/**
 * @file chassis_solution.hpp
 * @author YE
 * @brief 底盘解算方案，包含运动学解算和PID控制
 * @version 0.5
 * @date 2026-06-11
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :全向轮解算 + yaw-aware位置环 + pos_pid_ public供Ozone调参
 * @note :
 * @versioninfo :
 */
 #pragma once

#include "Motor.hpp"
#include "pid_controller.h"
#include "topic_pool.h"

#include <array>
#include <cstddef>

class Omni45Chassis {
public:
  enum WheelIndex : size_t {
    kFrontLeft = 0,
    kFrontRight = 1,
    kRearLeft = 2,
    kRearRight = 3,
    kWheelCount = 4,
  };

  struct Geometry {
    float wheel_diameter_m;
    float track_width_m;
    float wheel_base_m;

    Geometry(float wheel_diameter = 0.15378f, float track_width = 0.44f,
             float wheel_base = 0.38f)
        : wheel_diameter_m(wheel_diameter), track_width_m(track_width),
          wheel_base_m(wheel_base) {}
  };

  struct SpeedPidParam {
    float kp;
    float ki;
    float kd;
    float max_out;
    float deadband;
    uint16_t improve;

    SpeedPidParam(float kp_in = 105.0f, float ki_in = 75.0f,
                  float kd_in = 0.20f, float max_out_in = 20000.0f,
                  float deadband_in = 0.3f, uint16_t improve_in = NONE)
        : kp(kp_in), ki(ki_in), kd(kd_in), max_out(max_out_in),
          deadband(deadband_in), improve(improve_in) {}
  };

  Omni45Chassis(C620Motor &motor_fl, C620Motor &motor_fr, C620Motor &motor_rl,
                C620Motor &motor_rr, const Geometry &geometry = Geometry())
      : motors_{&motor_fl, &motor_fr, &motor_rl, &motor_rr},
        geometry_(geometry) {
    setWheelDirectionSign({1.0f, -1.0f, 1.0f, -1.0f});
    configureSpeedPid(SpeedPidParam());
    configurePosHoldPid();
  }

  void configureGeometry(const Geometry &geometry) { geometry_ = geometry; }

  void configureSpeedPid(const std::array<SpeedPidParam, kWheelCount> &params) {
    for (size_t i = 0; i < speed_pid_.size(); ++i) {
      applyPidParam(speed_pid_[i], params[i]);
    }
  }

  void configureSpeedPid(const SpeedPidParam &param) {
    for (PID_t &pid : speed_pid_) {
      applyPidParam(pid, param);
    }
  }

  void setWheelDirectionSign(const std::array<float, 4> &direction_sign) {
    direction_sign_ = direction_sign;
  }

  std::array<float, 4> solveWheelRpm(const pub_chassis_cmd &cmd) const {
    const float vx = cmd.linear_x_;
    const float vy = cmd.linear_y_;
    const float wz = cmd.omega_;

    const float rotation_term =
        0.5f * (geometry_.track_width_m + geometry_.wheel_base_m) * wz;

    const float wheel_fl = vx + vy - rotation_term;
    const float wheel_fr = vx - vy + rotation_term;
    const float wheel_rl = vx - vy - rotation_term;
    const float wheel_rr = vx + vy + rotation_term;

    const float mps_to_rpm = 60.0f / (kPi * geometry_.wheel_diameter_m);
    return {wheel_fl * mps_to_rpm, wheel_fr * mps_to_rpm, wheel_rl * mps_to_rpm,
            wheel_rr * mps_to_rpm};
  }

  void run(const pub_chassis_cmd &cmd) {
    target_rpm_ = solveWheelRpm(cmd);
    for (size_t i = 0; i < motors_.size(); ++i) {
      target_rpm_[i] *= direction_sign_[i];
      pid_output_[i] = PID_Calculate(&speed_pid_[i],
                                     motors_[i]->getRawCurrentSpeed(),
                                     target_rpm_[i]);
    //   motors_[i]->setMotorCmd(pid_output_[i]);
    }
  }

  void runHold(const std::array<float, kWheelCount> &target_pos) {
    hold_target_pos_ = target_pos;
    for (size_t i = 0; i < motors_.size(); ++i) {
      hold_pos_error_[i] =
          hold_target_pos_[i] - motors_[i]->getCurrentSumPos();
      float target_speed =
          PID_Calculate(&pos_pid_[i], 0.0f, hold_pos_error_[i]);
      pid_output_[i] = PID_Calculate(&speed_pid_[i],
                                     motors_[i]->getRawCurrentSpeed(),
                                     target_speed);
    //   motors_[i]->setMotorCmd(pid_output_[i]);
    }
  }

  void runHoldWithYaw(const std::array<float, kWheelCount> &base_pos,
                      float yaw_delta_deg) {
    const float ratio = yawToWheelRatio();
    const float wheel_adjust = -ratio * yaw_delta_deg;
    for (size_t i = 0; i < motors_.size(); ++i)
      hold_target_pos_[i] = base_pos[i] + wheel_adjust;
    runHold(hold_target_pos_);
  }

  float yawToWheelRatio() const {
    return (geometry_.track_width_m + geometry_.wheel_base_m) /
           geometry_.wheel_diameter_m;
  }

  void configurePosHoldPid(float kp = 3.0f, float ki = 0.01f,
                           float kd = 0.00f, float max_out = 600.0f,
                           float deadband = 0.5f,
                           float integral_limit = 200.0f) {
    for (PID_t &pid : pos_pid_) {
      pid = {};
      pid.Kp = kp;
      pid.Ki = ki;
      pid.Kd = kd;
      pid.MaxOut = max_out;
      pid.DeadBand = deadband;
      pid.IntegralLimit = integral_limit;
      pid.Improve = Integral_Limit;
      PID_Init(&pid);
    }
  }

  const std::array<float, 4> &targetRpm() const { return target_rpm_; }
  const std::array<float, 4> &pidOutput() const { return pid_output_; }

  std::array<PID_t, kWheelCount> pos_pid_{};
  std::array<float, kWheelCount> hold_pos_error_{};
  std::array<float, kWheelCount> hold_target_pos_{};

private:
  static constexpr float kPi = 3.14159265358979323846f;

  static void applyPidParam(PID_t &pid, const SpeedPidParam &param) {
    pid = {};
    pid.Kp = param.kp;
    pid.Ki = param.ki;
    pid.Kd = param.kd;
    pid.MaxOut = param.max_out;
    pid.DeadBand = param.deadband;
    pid.Improve = param.improve;
    PID_Init(&pid);
  }

  std::array<C620Motor *, 4> motors_{};
  Geometry geometry_{};
  std::array<float, kWheelCount> direction_sign_{};
  std::array<PID_t, kWheelCount> speed_pid_{};
  std::array<float, kWheelCount> target_rpm_{};
  std::array<float, kWheelCount> pid_output_{};
};


/**

前宽：604 mm
侧宽：439 mm
轮径：80 mm

M2      M1
|--------|
|        |
|--------|
M3      M4

 */
