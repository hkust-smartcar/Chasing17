/*
 * car_manager.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening), Peter Tse (mcreng)
 *
 * Implementations for CarManager class.
 *
 */

#include "car_manager.h"

#include <memory>

#include "libsc/futaba_s3010.h"

#include "util/mpc.h"
#include "util/mpc_dual.h"

using libsc::FutabaS3010;
using std::move;
using std::unique_ptr;
using util::Mpc;
using util::MpcDual;
using util::ServoController;

uint16_t CarManager::us_distance_ = 0;
int32_t CarManager::left_speed_ = 0;
int32_t CarManager::right_speed_ = 0;
uint16_t CarManager::servo_deg_ = 0;
int8_t CarManager::slope_deg_ = 0;
CarManager::Side CarManager::side_ = CarManager::Side::kMiddle;
CarManager::Feature CarManager::feature_ = CarManager::Feature::kStraight;
CarManager::Identity CarManager::identity_ = CarManager::Identity::kFront;
CarManager::Car CarManager::car_ = CarManager::Car::kCar1;

constexpr CarManager::ServoBounds CarManager::kBoundsCar1;
constexpr CarManager::ServoBounds CarManager::kBoundsCar2;

constexpr CarManager::ServoAngles CarManager::kAnglesCar1;
constexpr CarManager::ServoAngles CarManager::kAnglesCar2;

constexpr CarManager::SideRatio CarManager::kRatioCar1;
constexpr CarManager::SideRatio CarManager::kRatioCar2;

CarManager::PidValues CarManager::kMotorPidCar1 = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
constexpr CarManager::PidValues CarManager::kMotorPidCar2;

unique_ptr<Mpc> CarManager::epc_left_ = nullptr;
unique_ptr<Mpc> CarManager::epc_right_ = nullptr;
unique_ptr<MpcDual> CarManager::epc_ = nullptr;
unique_ptr<ServoController> CarManager::servo_controller_ = nullptr;
unique_ptr<FutabaS3010> CarManager::servo_ = nullptr;
unique_ptr<FcYyUsV4> CarManager::usir_ = nullptr;
//unique_ptr<Mpu9250> CarManager::mpu_ = nullptr;

void CarManager::Init(Config config) {
  static_assert(kBoundsCar1.kLeftBound > kBoundsCar1.kCenter && kBoundsCar1.kCenter > kBoundsCar1.kRightBound,
                "Incorrect car 1 servo boundaries");
  static_assert(kBoundsCar2.kLeftBound > kBoundsCar2.kCenter && kBoundsCar2.kCenter > kBoundsCar2.kRightBound,
                "Incorrect car 2 servo boundaries");

  epc_left_ = move(config.epc_left);
  epc_right_ = move(config.epc_right);
  epc_ = move(config.epc);
  servo_controller_ = move(config.servo_controller);
//  mpu_ = move(config.mpu);
  servo_ = move(config.servo);
  identity_ = config.identity;
  car_ = config.car;
}

void CarManager::UpdateParameters() {
  UpdateSpeed();
  UpdateServoAngle();
  UpdateDistance();
//  UpdateSlope();
}

void CarManager::SwitchIdentity() {
  if (identity_ == Identity::kFront) {
    identity_ = Identity::kBack;
  } else {
    identity_ = Identity::kFront;
  }
}

CarManager::ServoBounds CarManager::GetServoBounds() {
  switch (car_) {
    default:
      // all cases covered
    case Car::kCar1:
      return kBoundsCar1;
    case Car::kCar2:
      return kBoundsCar2;
  }
}

CarManager::ServoAngles CarManager::GetServoAngles() {
  switch (car_) {
    default:
      // all cases covered
    case Car::kCar1:
      return kAnglesCar1;
    case Car::kCar2:
      return kAnglesCar2;
  }
}

CarManager::SideRatio CarManager::GetSideRatio() {
  switch (car_) {
    default:
      // all cases covered
    case Car::kCar1:
      return kRatioCar1;
    case Car::kCar2:
      return kRatioCar2;
  }
}

CarManager::PidValues CarManager::GetMotorPidValues() {
  switch (CarManager::GetCar()) {
    default:
    case CarManager::Car::kCar1:
      return kMotorPidCar1;
    case CarManager::Car::kCar2:
      return kMotorPidCar2;
  }
}

void CarManager::SetOverrideProtection(const bool override_protection, const MotorSide side) {
  if (side == MotorSide::kBoth) {
    if (epc_ != nullptr) {
      epc_->SetForceOverride(override_protection);
      return;
    }
    if (epc_left_ != nullptr) {
      epc_left_->SetForceOverride(override_protection);
    }
    if (epc_right_ != nullptr) {
      epc_right_->SetForceOverride(override_protection);
    }
    return;
  } else if (side == MotorSide::kLeft && epc_left_ != nullptr) {
    epc_left_->SetForceOverride(override_protection);
  } else if (side == MotorSide::kRight && epc_right_ != nullptr) {
    epc_right_->SetForceOverride(override_protection);
  }
}

void CarManager::SetTargetSpeed(const int16_t speed, MotorSide src) {
  if (src == MotorSide::kBoth && epc_ != nullptr) {
    epc_->SetTargetSpeed(speed, false);
    return;
  }

  if (src == MotorSide::kLeft && epc_left_ != nullptr) {
    epc_left_->SetTargetSpeed(speed, false);
  } else if (src == MotorSide::kRight && epc_right_ != nullptr) {
    epc_right_->SetTargetSpeed(speed, false);
  } else if (src == MotorSide::kBoth) {
    if (epc_left_ != nullptr) {
      epc_left_->SetTargetSpeed(speed, false);
    }
    if (epc_right_ != nullptr) {
      epc_right_->SetTargetSpeed(speed, false);
    }
  }
}

void CarManager::SetTargetAngle(const int16_t angle) {
  ServoBounds s = GetServoBounds();
  ServoAngles a = GetServoAngles();

  if (servo_controller_ != nullptr) {
    int16_t new_angle = angle;

    if (new_angle > a.kRightAngle) {
      new_angle = a.kRightAngle;
    } else if (new_angle < -a.kLeftAngle) {
      new_angle = -a.kLeftAngle;
    }

    servo_controller_->SetTargetAngle(new_angle, false);
    return;
  }

  if (servo_ != nullptr) {
    uint16_t new_angle = s.kCenter;
    if (angle > 0) {
      new_angle -= (s.kCenter - s.kRightBound) * (static_cast<float>(angle) / a.kRightAngle);
    } else if (angle < 0) {
      new_angle -= (s.kLeftBound - s.kCenter) * (static_cast<float>(angle) / a.kLeftAngle);
    }

    if (new_angle > s.kLeftBound) {
      new_angle = s.kLeftBound;
    } else if (new_angle < s.kRightBound) {
      new_angle = s.kRightBound;
    }

    servo_->SetDegree(new_angle);
    return;
  }
}

void CarManager::UpdateDistance() {
  if (usir_ == nullptr) {
    return;
  }

  if (identity_ == Identity::kFront) {
    us_distance_ = FcYyUsV4::kMinDistance;
  } else {
    us_distance_ = usir_->GetAvgDistance();
  }
}

void CarManager::UpdateSpeed() {
  if (epc_ != nullptr) {
    epc_->SetCommitFlag(true);
    epc_->DoCorrection();
    left_speed_ = epc_->GetCurrentSpeed(MpcDual::MotorSide::kLeft);
    right_speed_ = epc_->GetCurrentSpeed(MpcDual::MotorSide::kRight);
    return;
  }

  if (epc_left_ != nullptr) {
    epc_left_->SetCommitFlag(true);
    epc_left_->DoCorrection();
    left_speed_ = epc_left_->GetCurrentSpeed();
  }
  if (epc_right_ != nullptr) {
    epc_right_->SetCommitFlag(true);
    epc_right_->DoCorrection();
    right_speed_ = epc_right_->GetCurrentSpeed();
  }
}

void CarManager::UpdateServoAngle() {
  if (servo_controller_ != nullptr) {
    servo_controller_->SetCommitFlag(true);
    servo_controller_->DoCorrection();
    servo_deg_ = servo_controller_->GetRawAngle();
    return;
  }

  if (servo_ != nullptr) {
    servo_deg_ = servo_->GetDegree();
    return;
  }
}

void CarManager::UpdateSlope() {
  if (mpu_ == nullptr) {
    return;
  }
  // TODO(Derppening): Replace with call to MPU9250
  slope_deg_ = 0;
}
