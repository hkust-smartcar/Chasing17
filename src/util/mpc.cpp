/*
 * mpc.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening), Peter Tse (mcreng)
 *
 * Implementations for Mpc class.
 *
 */

#include "util/mpc.h"

#include <cmath>
#include <cstdint>

#include "libsc/alternate_motor.h"
#include "libsc/system.h"
#include "libsc/timer.h"

using libsc::AlternateMotor;
using libsc::System;
using libsc::Timer;
using std::abs;
using util::to_string;

namespace util {
/* Data Sets
 * kU = 0.003. Tu = 0.148
 * {0.002222222, 0, 0}
 * {0.002, 0.0001, 0}
 * {0.002, 0.0001, 0.00032/0.0005} //kinda lags the change a bit
 * {0.00198, 0.002, 0.00035} //seems to be working quite well, only tiny fluctuations
 */
constexpr float Mpc::kP;
constexpr float Mpc::kI;
constexpr float Mpc::kD;

constexpr uint8_t Mpc::kOverrideWaitCycles;
constexpr uint16_t Mpc::kProtectionMinCount;

constexpr uint16_t Mpc::MotorConstants::kUpperBound;
constexpr uint16_t Mpc::MotorConstants::kLowerBound;
constexpr uint16_t Mpc::MotorConstants::kUpperHardLimit;
constexpr uint16_t Mpc::MotorConstants::kLowerHardLimit;

Mpc::Mpc(libsc::DirEncoder* e, libsc::AlternateMotor* m, bool isClockwise)
    : motor_(m), encoder_(e) {
  static_assert(MotorConstants::kLowerHardLimit < MotorConstants::kUpperHardLimit,
                "Hard Lower Bound should be smaller than Hard Upper Bound");
  static_assert(MotorConstants::kLowerBound < MotorConstants::kUpperBound,
                "Lower Bound should be smaller than Upper Bound");

  motor_->SetPower(0);
  motor_->SetClockwise(isClockwise);
  UpdateEncoder();
}

Mpc::~Mpc() {
  encoder_.reset();
  motor_.reset();
}

void Mpc::SetTargetSpeed(const int16_t speed, bool commit_now) {
  target_speed_ = speed;
  if (commit_now) {
    CommitTargetSpeed();
  }
}

void Mpc::AddToTargetSpeed(const int16_t d_speed, bool commit_now) {
  target_speed_ += target_speed_ > 0 ? d_speed : -d_speed;
  if (commit_now) {
    CommitTargetSpeed();
  }
}

void Mpc::DoCorrection() {
  // cleanup from previous cycle if it is out of range
  // [kMotorLowerBound,kMotorUpperBound]
  motor_->SetPower(util::clamp<uint16_t>(motor_->GetPower(), MotorConstants::kLowerBound, MotorConstants::kUpperBound));

  // sets the correction target speed to the new speed, if
  // commit_target_flag_ is true.
  if (commit_target_flag_) {
    curr_speed_ = target_speed_;
    commit_target_flag_ = false;
  }

  UpdateEncoder();
  if (force_start_count_ > 0) {
    --force_start_count_;
  }

  // motor protection - turn off motor when encoder has null value
  // override this if force_start_count is bigger than 0
  if (abs(last_encoder_val_) < kProtectionMinCount && force_start_count_ == 0) {
    motor_->SetPower(0);
    return;
  } else if (last_encoder_val_ > 65530) {
    // workaround for bug in quad_encoder:
    // regression where encoder value will rise to 65535 at indeterminate intervals
    return;
  }

  // get the speed difference and add power linearly.
  // bigger difference = higher power difference
  int16_t speed_diff = static_cast<int16_t>(abs(curr_speed_) - std::fabs(average_encoder_val_));
  motor_->AddPower(speed_diff * kP);

  // add the speed difference in consideration to the cumulative error
  // greater the cumulative error, higher the power
  cum_error_ += speed_diff * (last_encoder_duration_);
  motor_->AddPower(static_cast<int16_t>(cum_error_ * kI));

  // add the speed difference in consideration to the rate of change of error
  // greater the change, higher the power
  motor_->AddPower(static_cast<int16_t>((speed_diff - prev_error_) / (last_encoder_duration_)) * kD);
  prev_error_ = speed_diff;

  // hard limit bounds checking
  motor_->SetPower(util::clamp<uint16_t>(motor_->GetPower(),
                                         MotorConstants::kLowerHardLimit,
                                         MotorConstants::kUpperHardLimit));
}

void Mpc::CommitTargetSpeed() {
  commit_target_flag_ = true;
  DoCorrection();
}

void Mpc::SetForceOverride(bool force_override) {
  if (force_override) {
    force_start_count_ = kOverrideWaitCycles;
  } else {
    force_start_count_ = 0;
  }
}

void Mpc::UpdateEncoder() {
  last_encoder_duration_ = GetTimeElapsed();

  encoder_->Update();
  last_encoder_val_ = encoder_->GetCount() * 1000 / static_cast<int32_t>(last_encoder_duration_);

  last_ten_encoder_val_.push_back(last_encoder_val_);
  while (last_ten_encoder_val_.size() > 10) last_ten_encoder_val_.erase(last_ten_encoder_val_.begin());
  average_encoder_val_ = 0;
  for (auto& m : last_ten_encoder_val_) {
    average_encoder_val_ += m;
  }
  average_encoder_val_ /= last_ten_encoder_val_.size();

  time_encoder_start_ = libsc::System::Time();
}

MpcDebug::MpcDebug(Mpc* mpc) : mpc_(mpc) {}

void MpcDebug::OutputEncoderMotorValues(libsc::LcdConsole* console) const {
  std::string s = to_string(mpc_->last_encoder_val_) + " " + to_string(mpc_->motor_->GetPower()) + "\n";
  console->WriteString(s.c_str());
}

void MpcDebug::OutputLastEncoderValues(libsc::LcdConsole* console) const {
  std::string s = to_string(mpc_->last_encoder_duration_) + " " + to_string(mpc_->last_encoder_val_) + "\n";
  console->WriteString(s.c_str());
}

void MpcDebug::SetMotorPower(uint16_t power, bool is_clockwise) {
  mpc_->motor_->SetClockwise(is_clockwise);
  mpc_->motor_->SetPower(power);
}
}  // namespace util
