/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening), Peter Tse (mcreng)
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
/* Data Sets
 * kU = 0.003. Tu = 0.148
 * {0.002222222, 0, 0}
 * {0.002, 0.0001, 0}
 * {0.002, 0.0001, 0.00032/0.0005} //kinda lags the change a bit
 * {0.00198, 0.002, 0.00035} //seems to be working quite well, only tiny fluctuations
 */
float Mpc::kP = 0.00198;
float Mpc::kI = 0.002;
float Mpc::kD = 0;

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
  if (last_encoder_val_ == 0) {
    motor_->SetPower(0);
    return;
  }

  // get the speed difference and add power linearly.
  // bigger difference = higher power difference
  int16_t speed_diff = static_cast<int16_t>(abs(curr_speed_) - abs(average_encoder_val_));
  motor_->AddPower(speed_diff * kP);

  // add the speed difference in consideration to the cumulative error
  // greater the cumulative error, higher the power
  cum_error_ += speed_diff * (last_encoder_duration_ / 1000.0);
  motor_->AddPower(static_cast<int16_t>(cum_error_ * kI));

  // add the speed difference in consideration to the rate of change of error
  // greater the change, higher the power
  motor_->AddPower(static_cast<int16_t>((speed_diff - prev_error_) / (last_encoder_duration_ / 1000.0)) * kD);
  prev_error_ = speed_diff;

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
  last_ten_encoder_val_.push_back(last_encoder_val_);
  while (last_ten_encoder_val_.size() > 10) last_ten_encoder_val_.pop_front();
  average_encoder_val_ = 0;
  for (auto&& m : last_ten_encoder_val_){
	  average_encoder_val_ += m;
  }
  average_encoder_val_ /= static_cast<int32_t>(last_ten_encoder_val_.size());

  time_encoder_start_ = libsc::System::Time();
}

void MpcDebug::OutputEncoderMotorValues(libsc::LcdConsole* console) const {
//  std::string s = std::to_string(mpc_->last_encoder_val_) + " " + std::to_string(mpc_->motor_->GetPower()) + "\n";
//  console->WriteString(s.c_str());
}

void MpcDebug::OutputLastEncoderValues(libsc::LcdConsole* console) const {
//  std::string s = std::to_string(mpc_->last_encoder_duration_) + " " + std::to_string(mpc_->last_encoder_val_) + "\n";
//  console->WriteString(s.c_str());
}

void MpcDebug::SetMotorPower(uint16_t power, bool is_clockwise) {
  mpc_->motor_->SetClockwise(is_clockwise);
  mpc_->motor_->SetPower(power);
}
}  // namespace util
