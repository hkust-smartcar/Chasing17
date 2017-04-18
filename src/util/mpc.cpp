/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 *
 * Implementations for Mpc class.
 *
 */

#include "util/mpc.h"

#include <cstdint>

#include "libsc/alternate_motor.h"
#include "libsc/system.h"
#include "libsc/timer.h"

using libsc::AlternateMotor;
using libsc::System;
using libsc::Timer;
using std::abs;

namespace util {
void Mpc::SetTargetSpeed(const int16_t speed, bool commit_now) {
  target_speed_ = speed;
  if (commit_now) {
    CommitTargetSpeed();
  }
}

void Mpc::AddToTargetSpeed(const int16_t d_speed, bool commit_now) {
  target_speed_ += d_speed;
  if (commit_now) {
    CommitTargetSpeed();
  }
}

void Mpc::DoCorrection() {
  // cleanup from previous cycle if it is out of range
  // [kMotorLowerBound,kMotorUpperBound]
  if (motor_->GetPower() > static_cast<uint16_t>(MotorConstants::kUpperBound)) {
    motor_->SetPower(static_cast<uint16_t>(MotorConstants::kUpperBound));
  } else if (motor_->GetPower() < static_cast<uint16_t>(MotorConstants::kLowerBound)) {
    motor_->SetPower(static_cast<uint16_t>(MotorConstants::kLowerBound));
  }

  // sets the correction target speed to the new speed, if
  // commit_target_flag_ is true.
  if (commit_target_flag_) {
    curr_speed_ = target_speed_;
    commit_target_flag_ = false;
  }

  UpdateEncoder();

  // motor protection - turn off motor when motor is on but encoder has null value
  if (motor_->GetPower() != 0 && last_encoder_val_ == 0) {
    motor_->SetPower(0);
    return;
  }

  // checks if the motor direction differs from our target
  if (!HasSameSign(last_encoder_val_, static_cast<int32_t>(curr_speed_))) {
    motor_->SetClockwise(!motor_->IsClockwise());
  }

  // get the speed difference and add power linearly.
  // bigger difference = higher power difference
  int16_t speed_diff = static_cast<int16_t>(abs(last_encoder_val_) - abs(curr_speed_));
  motor_->AddPower(-speed_diff / static_cast<uint16_t>(MotorConstants::kDiffFactor));

  // hard limit bounds checking
  if (motor_->GetPower() > static_cast<uint16_t>(MotorConstants::kUpperHardLimit)) {
    motor_->SetPower(static_cast<uint16_t>(MotorConstants::kUpperHardLimit));
  } else if (motor_->GetPower() < static_cast<uint16_t>(MotorConstants::kLowerHardLimit)) {
    motor_->SetPower(static_cast<uint16_t>(MotorConstants::kLowerHardLimit));
  }
}

void Mpc::CommitTargetSpeed() {
  commit_target_flag_ = true;
  DoCorrection();
}

void Mpc::UpdateEncoder() {
  last_encoder_duration_ = GetTimeElapsed();
  encoder_->Update();
  last_encoder_val_ = encoder_->GetCount() * 1000 / static_cast<int32_t>(last_encoder_duration_);
  time_encoder_start_ = libsc::System::Time();
}

void MpcDebug::OutputEncoderMotorValues(libsc::LcdConsole* console) const {
  std::string s = std::to_string(mpc_->last_encoder_val_) + " " + std::to_string(mpc_->motor_->GetPower()) + "\n";
  console->WriteString(s.c_str());
}

void MpcDebug::OutputLastEncoderValues(libsc::LcdConsole* console) const {
  std::string s = std::to_string(mpc_->last_encoder_duration_) + " " + std::to_string(mpc_->last_encoder_val_) + "\n";
  console->WriteString(s.c_str());
}

void MpcDebug::SetMotorPower(uint16_t power, bool is_clockwise) {
  mpc_->motor_->SetClockwise(is_clockwise);
  mpc_->motor_->SetPower(power);
}
}  // namespace util
