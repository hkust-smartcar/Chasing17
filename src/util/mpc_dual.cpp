/*
 * mpc_dual.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 *
 * Implementations for MpcDual class.
 *
 */

#include "util/mpc_dual.h"

#include "car_manager.h"

using libsc::AlternateMotor;
using libsc::DirEncoder;
using util::to_string;

namespace util {
MpcDual::MpcDual(AlternateMotor* motor_left,
                 AlternateMotor* motor_right,
                 DirEncoder* encoder_left,
                 DirEncoder* encoder_right) {
  mpc_left_ = util::make_unique<Mpc>(encoder_left, motor_left, true);
  mpc_right_ = util::make_unique<Mpc>(encoder_right, motor_right, false);
}

MpcDual::~MpcDual() {
  mpc_left_.reset(nullptr);
  mpc_right_.reset(nullptr);
}

void MpcDual::DoCorrection() {
  CarManager::ServoBounds s = CarManager::GetServoBounds();
  CarManager::SideRatio ratio = CarManager::GetSideRatio();

  int servo_diff = s.kCenter - CarManager::GetServoDeg();

  // calculates the path deviation, then times a factor to account for speed
  // difference between left/right wheels
  if (mpc_left_->GetTargetSpeed() == 0 || mpc_right_->GetTargetSpeed() == 0) {  // stopping
    // set the target speeds to 0
    mpc_left_->SetTargetSpeed(0, false);
    mpc_right_->SetTargetSpeed(0, false);

  } else if (servo_diff < 0) {  // turning left
    // calculate the ratio of current steering angle to max steering angle
    float motor_speed_diff = static_cast<float>(servo_diff) / (s.kCenter - s.kLeftBound);
    motor_speed_diff_ = motor_speed_diff;

    // get the left turning radius
    float turning_radius = CarManager::kWheelbase / ratio.kLeft;
    float motor_speed_diff_left = motor_speed_diff;
    float motor_speed_diff_right = motor_speed_diff;

    // calculate the speed difference relative to the center of the car
    motor_speed_diff_left *= 1 - ((turning_radius - CarManager::kAxleLength / 2) / turning_radius);
    motor_speed_diff_right *= 1 - ((turning_radius + CarManager::kAxleLength / 2) / turning_radius);

    // add to the current target speed
    mpc_right_->AddToTargetSpeed(mpc_right_->GetTargetSpeed() * motor_speed_diff_right, false);
    mpc_left_->AddToTargetSpeed(mpc_left_->GetTargetSpeed() * -motor_speed_diff_left, false);
  } else if (servo_diff > 0) {  // turning right
    // calculate the ratio of current steering angle to max steering angle
    float motor_speed_diff = static_cast<float>(servo_diff)  / (s.kCenter - s.kRightBound);
    motor_speed_diff_ = motor_speed_diff;

    // get the right turning radius
    float turning_radius = CarManager::kWheelbase / ratio.kRight;
    float motor_speed_diff_left = motor_speed_diff;
    float motor_speed_diff_right = motor_speed_diff;

    // calculate the speed difference relative to the center of the car
    motor_speed_diff_left *= 1 - ((turning_radius - CarManager::kAxleLength / 2) / turning_radius);
    motor_speed_diff_right *= 1 - ((turning_radius + CarManager::kAxleLength / 2) / turning_radius);

    // add to the current target speed
    mpc_left_->AddToTargetSpeed(mpc_left_->GetTargetSpeed() * motor_speed_diff_left, false);
    mpc_right_->AddToTargetSpeed(mpc_right_->GetTargetSpeed() * -motor_speed_diff_right, false);
  } else {  // straight
    // just set them as they are
    mpc_left_->SetTargetSpeed(mpc_left_->GetTargetSpeed(), false);
    mpc_right_->SetTargetSpeed(mpc_right_->GetTargetSpeed(), false);
  }

  mpc_left_->SetCommitFlag(true);
  mpc_right_->SetCommitFlag(true);
  mpc_left_->DoCorrection();
  mpc_right_->DoCorrection();
}

void MpcDual::SetTargetSpeed(const int16_t speed, bool commit_now) {
  mpc_left_->SetTargetSpeed(speed, false);
  mpc_right_->SetTargetSpeed(-speed, false);
  if (commit_now) {
    DoCorrection();
  }
}

void MpcDual::AddToTargetSpeed(const int16_t speed, bool commit_now) {
  mpc_left_->AddToTargetSpeed(speed, false);
  mpc_right_->AddToTargetSpeed(-speed, false);
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

void MpcDual::SetForceOverride(bool force_override, MotorSide side) {
  if (side == MotorSide::kBoth || side == MotorSide::kLeft) {
    mpc_left_->SetForceOverride(force_override);
  }
  if (side == MotorSide::kBoth || side == MotorSide::kRight) {
    mpc_right_->SetForceOverride(force_override);
  }
}

MpcDualDebug::MpcDualDebug(MpcDual* mpc_dual) : mpc_dual_(mpc_dual)
{}

void MpcDualDebug::OutputEncoderMotorValues(libsc::LcdConsole* console, MpcDual::MotorSide side) const {
  std::string s = "";

  // display encoder values + motor power
  console->SetCursorRow(0);
  if (side == MpcDual::MotorSide::kLeft || side == MpcDual::MotorSide::kBoth) {
    s += "L: " + to_string(mpc_dual_->mpc_left_->last_encoder_val_) +
        " " + to_string(mpc_dual_->mpc_left_->motor_->GetPower());
  }
  if (side == MpcDual::MotorSide::kRight || side == MpcDual::MotorSide::kBoth) {
    if (s != "") { s += "\n"; }
    s += "R: " + to_string(mpc_dual_->mpc_right_->last_encoder_val_) +
        " " + to_string(mpc_dual_->mpc_right_->motor_->GetPower());
  }
  if (s != "") { s += "\n"; }
  util::ConsoleWriteString(console, s); 

  // display target speed
  console->SetCursorRow(8);
  s = "L target: " + to_string(mpc_dual_->mpc_left_->GetTargetSpeed()) + "\n";
  console->WriteString(s.c_str());
  console->SetCursorRow(9);
  s = "R target: " + to_string(mpc_dual_->mpc_right_->GetTargetSpeed()) + "\n";
  console->WriteString(s.c_str());
}

void MpcDualDebug::OutputLastEncoderValues(libsc::LcdConsole* console, MpcDual::MotorSide side) const {
  std::string s = "";

  // output encoder duration + encoder value
  console->SetCursorRow(3);
  if (side == MpcDual::MotorSide::kLeft || side == MpcDual::MotorSide::kBoth) {
    s += "L: " + to_string(mpc_dual_->mpc_left_->last_encoder_duration_) +
        " " + to_string(mpc_dual_->mpc_left_->last_encoder_val_);
  }
  if (side == MpcDual::MotorSide::kRight || side == MpcDual::MotorSide::kBoth) {
    if (s != "") { s += "\n"; }
    s += "R: " + to_string(mpc_dual_->mpc_right_->last_encoder_duration_) +
        " " + to_string(mpc_dual_->mpc_right_->last_encoder_val_);
  }
  if (s != "") { s += "\n"; }

  ConsoleWriteString(console, s);
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
