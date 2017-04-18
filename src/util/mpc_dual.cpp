/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 *
 * Implementations for EpcDual class.
 *
 */

#include "util/mpc_dual.h"

#include "car_manager.h"

using std::to_string;

namespace util {
void MpcDual::DoCorrection() {
  uint16_t servo_angle = CarManager::GetServoDeg();
  int16_t servo_diff = servo_angle - CarManager::ServoBounds::kCenter;

  // calculates the path deviation, then times a factor to account for speed
  // difference between left/right wheels
  if (servo_diff > 0) {  // turning right
    uint16_t motor_speed_diff = servo_diff - (CarManager::ServoBounds::kCenter - CarManager::ServoBounds::kLeftBound);
    motor_speed_diff *= (82 / 100);
    mpc_left_->AddToTargetSpeed(motor_speed_diff, false);
    mpc_right_->AddToTargetSpeed(-motor_speed_diff, false);
  } else if (servo_diff < 0) {  // turning left
    uint16_t motor_speed_diff = servo_diff - (CarManager::ServoBounds::kRightBound - CarManager::ServoBounds::kCenter);
    motor_speed_diff *= (82 / 100);
    mpc_right_->AddToTargetSpeed(motor_speed_diff, false);
    mpc_left_->AddToTargetSpeed(-motor_speed_diff, false);
  }

  mpc_left_->DoCorrection();
  mpc_right_->DoCorrection();
}

void MpcDual::SetTargetSpeed(const int16_t speed, bool commit_now) {
  mpc_left_->SetTargetSpeed(speed);
  mpc_right_->SetTargetSpeed(speed);
  if (commit_now) {
    DoCorrection();
  }
}

void MpcDual::AddToTargetSpeed(const int16_t speed, bool commit_now) {
  mpc_left_->AddToTargetSpeed(speed);
  mpc_right_->AddToTargetSpeed(speed);
  if (commit_now) {
    DoCorrection();
  }
}

libsc::Timer::TimerInt MpcDual::GetTimeElapsed(MotorSide side) const {
  if (side == MotorSide::kLeft) {
    return mpc_left_->GetTimeElapsed();
  } else {
    return mpc_right_->GetTimeElapsed();
  }
}

int32_t MpcDual::GetCurrentSpeed(MotorSide side) const {
  if (side == MotorSide::kLeft) {
    return mpc_left_->GetCurrentSpeed();
  } else {
    return mpc_right_->GetCurrentSpeed();
  }
}

void MpcDualDebug::OutputEncoderMotorValues(libsc::LcdConsole* console, MpcDual::MotorSide side) const {
  std::string s = "";
  if (side == MpcDual::MotorSide::kLeft || side == MpcDual::MotorSide::kBoth) {
    s += "L: " + to_string(mpc_dual_->mpc_left_->last_encoder_val_) +
        " " + to_string(mpc_dual_->mpc_left_->motor_->GetPower());
  }
  if (side == MpcDual::MotorSide::kRight || side == MpcDual::MotorSide::kBoth) {
    if (s != "") { s += "\n"; }
    s += "R: " + to_string(mpc_dual_->mpc_right_->last_encoder_val_) +
        " " + to_string(mpc_dual_->mpc_right_->motor_->GetPower());
  }
  console->WriteString(s.c_str());
}

void MpcDualDebug::OutputLastEncoderValues(libsc::LcdConsole* console, MpcDual::MotorSide side) const {
  std::string s = "";
  if (side == MpcDual::MotorSide::kLeft || side == MpcDual::MotorSide::kBoth) {
    s += "L: " + to_string(mpc_dual_->mpc_left_->last_encoder_duration_) +
        " " + to_string(mpc_dual_->mpc_left_->last_encoder_val_);
  }
  if (side == MpcDual::MotorSide::kRight || side == MpcDual::MotorSide::kBoth) {
    if (s != "") { s += "\n"; }
    s += "R: " + to_string(mpc_dual_->mpc_right_->last_encoder_duration_) +
        " " + to_string(mpc_dual_->mpc_right_->last_encoder_val_);
  }
  console->WriteString(s.c_str());
}

void MpcDualDebug::SetMotorPower(uint16_t power, MpcDual::MotorSide side, bool is_clockwise) {
  switch (side) {
    case MpcDual::MotorSide::kLeft:
      mpc_dual_->mpc_left_->motor_->SetClockwise(is_clockwise);
      mpc_dual_->mpc_left_->motor_->SetPower(power);
      break;
    case MpcDual::MotorSide::kRight:
      mpc_dual_->mpc_right_->motor_->SetClockwise(is_clockwise);
      mpc_dual_->mpc_right_->motor_->SetPower(power);
      break;
    case MpcDual::MotorSide::kBoth:
      mpc_dual_->mpc_left_->motor_->SetClockwise(is_clockwise);
      mpc_dual_->mpc_left_->motor_->SetPower(power);
      mpc_dual_->mpc_right_->motor_->SetClockwise(is_clockwise);
      mpc_dual_->mpc_right_->motor_->SetPower(power);
      break;
  }
}

libsc::Timer::TimerInt MpcDualDebug::GetLastRunDuration(MpcDual::MotorSide side) const {
  switch (side) {
    case MpcDual::MotorSide::kLeft:
      return mpc_dual_->mpc_left_->last_encoder_duration_;
    case MpcDual::MotorSide::kRight:
      return mpc_dual_->mpc_right_->last_encoder_duration_;
    case MpcDual::MotorSide::kBoth:
      return 0;
  }
}

int32_t MpcDualDebug::GetEncoderVal(MpcDual::MotorSide side) const {
  switch (side) {
    case MpcDual::MotorSide::kLeft:
      return mpc_dual_->mpc_left_->last_encoder_val_;
    case MpcDual::MotorSide::kRight:
      return mpc_dual_->mpc_right_->last_encoder_val_;
    case MpcDual::MotorSide::kBoth:
      return 0;
  }
}
}  // namespace util
