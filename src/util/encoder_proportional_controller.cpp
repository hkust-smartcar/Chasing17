/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 */

#include "util/encoder_proportional_controller.h"

#include <cstdint>

#include "libsc/alternate_motor.h"
#include "libsc/system.h"
#include "libsc/timer.h"

using libsc::AlternateMotor;
using libsc::System;
using libsc::Timer;
using std::abs;

namespace util {
void EncoderPController::DoCorrection() {
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
}  // namespace util
