//
// Created by david on 8/6/2017.
//

#include "util/servo_controller.h"

namespace util {

constexpr float ServoController::kP;
constexpr float ServoController::kI;
constexpr float ServoController::kD;

ServoController::ServoController(Mpu9250* c, libsc::FutabaS3010* s)
    : compass_(c), servo_(s) {}

ServoController::~ServoController() {
  servo_.reset();
  compass_.reset();
}

void ServoController::DoCorrection() {
  // sets the target compass angle to the new value, if committing
  if (commit_target_flag_) {
    target_compass_angle_ += servo_change_target_;
    target_compass_angle_ %= 360;
    commit_target_flag_ = false;
  }

  UpdateCompass();

  int16_t angle_diff = CalcAngleDiff(target_compass_angle_, last_compass_angle_);
  prev_error_ = angle_diff;
  int8_t left_speed = CarManager::GetLeftSpeed();
  int8_t right_speed = CarManager::GetRightSpeed();

  uint16_t new_angle = servo_->GetDegree();
  new_angle -= angle_diff * kP;
  new_angle -= (angle_diff - prev_error_) / (last_compass_duration_ / 1000.0) * kD;

  CarManager::ServoBounds s;
  switch (CarManager::GetCar()) {
    case CarManager::Car::kCar1:
      s = CarManager::kBoundsCar1;
      break;
    case CarManager::Car::kCar2:
      s = CarManager::kBoundsCar2;
      break;
    default:
      // all cases covered
      break;
  }

  if (new_angle > s.kLeftBound) {
    new_angle = s.kLeftBound;
  } else if (new_angle < s.kRightBound) {
    new_angle = s.kRightBound;
  }

  servo_->SetDegree(new_angle);
}

void ServoController::SetTargetAngle(int16_t change, bool commit_now) {
  servo_change_target_ = change;
  if (commit_now) {
    CommitTargetAngle();
  }
}

void ServoController::CommitTargetAngle() {
  commit_target_flag_ = true;
  DoCorrection();
}

void ServoController::UpdateCompass() {
  last_compass_duration_ = GetTimeElapsed();

  // TODO(Derppening): Replace with function call to Mpu9250
  last_compass_angle_ /* = compass_->GetValues() */;

  last_ten_compass_val_.push_back(last_compass_angle_);
  while (last_ten_compass_val_.size() > 10) last_ten_compass_val_.erase(last_ten_compass_val_.cbegin());
  for (auto& m : last_ten_compass_val_) {
    average_compass_val_ += (m + 360);
  }
  average_compass_val_ %= 360;
  average_compass_val_ /= last_ten_compass_val_.size();

  time_encoder_start_ = libsc::System::Time();
}

}  // namespace util
