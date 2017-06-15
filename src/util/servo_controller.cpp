/*
 * servo_controller.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 *
 * Implementations for ServoController class.
 *
 */

#include "util/servo_controller.h"

#include "util/util.h"

using libsc::LcdConsole;

namespace util {

constexpr float ServoController::kP;
constexpr float ServoController::kD;

ServoController::ServoController(libsc::FutabaS3010* s)
    : servo_(s) {}

ServoController::~ServoController() {
  servo_.reset();
}

void ServoController::DoCorrection() {
  last_servo_duration_ = GetTimeElapsed();

  if (commit_target_flag_) {
    curr_servo_target_ = servo_target_;
    commit_target_flag_ = false;
  }

  last_servo_vals_.push_back(curr_servo_target_);
  while (last_servo_vals_.size() > 5) last_servo_vals_.erase(last_servo_vals_.begin());
  average_servo_val_ = 0;
  for (auto& m : last_servo_vals_) {
    average_servo_val_ += m;
  }
  average_servo_val_ /= last_servo_vals_.size();

  CarManager::ServoBounds s = CarManager::GetServoBounds();
  CarManager::ServoAngles a = CarManager::GetServoAngles();

  int16_t target_angle_change = curr_servo_target_;
  uint16_t new_angle = s.kCenter;

  target_angle_change *= kP;
  last_servo_error_ = ((target_angle_change - average_servo_val_) / (last_servo_duration_ / 1000.0)) * kD;
  target_angle_change += ((target_angle_change - average_servo_val_) / (last_servo_duration_ / 1000.0)) * kD;

  if (target_angle_change > 0) {
    new_angle -= (s.kCenter - s.kRightBound) * (static_cast<float>(curr_servo_target_) / a.kRightAngle);
  } else if (curr_servo_target_ < 0) {
    new_angle -= (s.kLeftBound - s.kCenter) * (static_cast<float>(curr_servo_target_) / a.kLeftAngle);
  }

  if (new_angle > s.kLeftBound) {
    new_angle = s.kLeftBound;
  } else if (new_angle < s.kRightBound) {
    new_angle = s.kRightBound;
  }

  servo_->SetDegree(new_angle);

  last_servo_target_ = curr_servo_target_;
  time_servo_start_ = libsc::System::Time();
}

void ServoController::SetTargetAngle(int16_t change, bool commit_now) {
  servo_target_ = change;
  if (commit_now) {
    CommitTargetAngle();
  }
}

void ServoController::CommitTargetAngle() {
  commit_target_flag_ = true;
  DoCorrection();
}

ServoControllerDebug::ServoControllerDebug(ServoController* servo_ctrl) : servo_controller_(servo_ctrl)
{}

void ServoControllerDebug::OutputErrorValues(LcdConsole* console) const {
  std::string s = "";
  for (auto& a : servo_controller_->last_servo_vals_) {
    s += util::to_string(a) + " ";
  }
  util::ConsoleClearRow(console, 5);
  util::ConsoleWriteString(console, "5avg: " + util::to_string(servo_controller_->average_servo_val_));
  util::ConsoleClearRow(console, 6);
  util::ConsoleWriteString(console, "v: " + s);
}

void ServoControllerDebug::OutputServoValues(LcdConsole* console) const {
  util::ConsoleClearRow(console, 0);
  util::ConsoleWriteString(console, "raw: " + util::to_string(servo_controller_->GetRawAngle()));
  util::ConsoleClearRow(console, 1);
  util::ConsoleWriteString(console, "trgt: " + util::to_string(servo_controller_->curr_servo_target_));
  util::ConsoleClearRow(console, 2);
  util::ConsoleWriteString(console, "last: " + util::to_string(servo_controller_->last_servo_target_));
  util::ConsoleClearRow(console, 3);
  util::ConsoleWriteString(console, "s_err: " + util::to_string(servo_controller_->last_servo_error_));
}
}  // namespace util
