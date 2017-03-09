/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 */

#include "util/encoder_proportional_controller.h"

using libsc::AlternateMotor;
using libsc::System;
using libsc::Timer;

namespace util {
void EncoderPController::DoCorrection() {
  // cleanup from previous cycle if it is out of range
  // [kMotorLowerBound,kMotorUpperBound]
  if (motor_->GetPower() > kMotorUpperBound) {
    motor_->SetPower(kMotorUpperBound);
  } else if (motor_->GetPower() < kMotorLowerBound) {
    motor_->SetPower(kMotorLowerBound);
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
  int16_t speed_diff = static_cast<int16_t>(std::abs(last_encoder_val_) - std::abs(curr_speed_));
  motor_->AddPower(-speed_diff / kMotorDFactor);

  // hard limit bounds checking
  if (motor_->GetPower() > kMotorUpperHardLimit) {
    motor_->SetPower(kMotorUpperHardLimit);
  } else if (motor_->GetPower() < kMotorLowerHardLimit) {
    motor_->SetPower(kMotorLowerHardLimit);
  }
}
}  // namespace util
