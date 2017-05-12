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

//using std::to_string;

namespace util {
void MpcDual::DoCorrection() {
  CarManager::ServoBounds s = CarManager::GetServoBounds();

  float servo_diff = s.kCenter - CarManager::GetServoDeg();

  // calculates the path deviation, then times a factor to account for speed
  // difference between left/right wheels
  if (servo_diff < 0) {  // turning left
    float motor_speed_diff = servo_diff / (s.kCenter - s.kLeftBound);
    motor_speed_diff *= 0.142;
    motor_speed_diff_ = motor_speed_diff;
    mpc_right_->AddToTargetSpeed(mpc_right_->GetTargetSpeed() * motor_speed_diff, false);
    mpc_left_->AddToTargetSpeed(mpc_left_->GetTargetSpeed() * -motor_speed_diff, false);
  } else if (servo_diff > 0) {  // turning right
    float motor_speed_diff = servo_diff / (s.kRightBound - s.kCenter);
    motor_speed_diff *= 0.142;
    motor_speed_diff_ = motor_speed_diff;
    mpc_left_->AddToTargetSpeed(mpc_left_->GetTargetSpeed() * motor_speed_diff, false);
    mpc_right_->AddToTargetSpeed(mpc_right_->GetTargetSpeed() * -motor_speed_diff, false);
  }

  mpc_left_->SetCommitFlag(true);
  mpc_right_->SetCommitFlag(true);
  mpc_left_->DoCorrection();
  mpc_right_->DoCorrection();
}

void MpcDual::SetTargetSpeed(const int16_t speed, bool commit_now) {
  mpc_left_->SetTargetSpeed(speed, false);
  mpc_right_->SetTargetSpeed(speed, false);
  if (commit_now) {
    DoCorrection();
  }
}

void MpcDual::AddToTargetSpeed(const int16_t speed, bool commit_now) {
  mpc_left_->AddToTargetSpeed(speed, false);
  mpc_right_->AddToTargetSpeed(speed, false);
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
  console->SetCursorRow(0);
  std::string s = "";
  if (side == MpcDual::MotorSide::kLeft || side == MpcDual::MotorSide::kBoth) {
//    s += "L: " + std::to_string(mpc_dual_->mpc_left_->last_encoder_val_) +
//        " " + std::to_string(mpc_dual_->mpc_left_->motor_->GetPower());
  }
  if (side == MpcDual::MotorSide::kRight || side == MpcDual::MotorSide::kBoth) {
    if (s != "") { s += "\n"; }
//    s += "R: " + std::to_string(mpc_dual_->mpc_right_->last_encoder_val_) +
//        " " + std::to_string(mpc_dual_->mpc_right_->motor_->GetPower());
  }
  if (s != "") { s += "\n"; }
  console->WriteString(s.c_str());
  if (mpc_dual_->mpc_left_->last_encoder_val_ > 65535) {
    console->SetCursorRow(6);
    s = "L > 2^15";
    console->WriteString(s.c_str());
  }
  if (mpc_dual_->mpc_right_->last_encoder_val_ > 65535) {
    console->SetCursorRow(7);
    s = "R > 2^15";
    console->WriteString(s.c_str());
  }
  console->SetCursorRow(8);
//  s = "L target: " + std::to_string(mpc_dual_->mpc_left_->GetTargetSpeed()) + "\n";
  console->WriteString(s.c_str());
  console->SetCursorRow(9);
//  s = "R target: " + std::to_string(mpc_dual_->mpc_right_->GetTargetSpeed()) + "\n";
  console->WriteString(s.c_str());
}

void MpcDualDebug::OutputLastEncoderValues(libsc::LcdConsole* console, MpcDual::MotorSide side) const {
  console->SetCursorRow(3);
  std::string s = "";
  if (side == MpcDual::MotorSide::kLeft || side == MpcDual::MotorSide::kBoth) {
//    s += "L: " + std::to_string(mpc_dual_->mpc_left_->last_encoder_duration_) +
//        " " + std::to_string(mpc_dual_->mpc_left_->last_encoder_val_);
  }
  if (side == MpcDual::MotorSide::kRight || side == MpcDual::MotorSide::kBoth) {
    if (s != "") { s += "\n"; }
//    s += "R: " + std::to_string(mpc_dual_->mpc_right_->last_encoder_duration_) +
//        " " + std::to_string(mpc_dual_->mpc_right_->last_encoder_val_);
  }
  if (s != "") { s += "\n"; }
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
    default:
      // all cases covered
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
    default:
      // all cases covered
      return 0;
  }
}

int32_t MpcDualDebug::GetEncoderVal(MpcDual::MotorSide side) const {
  switch (side) {
    case MpcDual::MotorSide::kLeft:
      return mpc_dual_->mpc_left_->average_encoder_val_;
    case MpcDual::MotorSide::kRight:
      return mpc_dual_->mpc_right_->average_encoder_val_;
    case MpcDual::MotorSide::kBoth:
      return 0;
    default:
      // all cases covered
      return 0;
  }
}
}  // namespace util
