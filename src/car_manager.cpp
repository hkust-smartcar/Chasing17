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

#include "mpu9250.h"
#include "util/mpc.h"
#include "util/mpc_dual.h"

using libsc::FutabaS3010;
using std::move;
using std::unique_ptr;
using util::Mpc;
using util::MpcDual;

uint8_t CarManager::left_speed_ = 0;
uint8_t CarManager::right_speed_ = 0;
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

unique_ptr<Mpc> CarManager::epc_left_ = nullptr;
unique_ptr<Mpc> CarManager::epc_right_ = nullptr;
unique_ptr<MpcDual> CarManager::epc_ = nullptr;
unique_ptr<Mpu9250> CarManager::mpu_ = nullptr;
unique_ptr<FutabaS3010> CarManager::servo_ = nullptr;

void CarManager::Init(Config config) {
  epc_left_ = move(config.epc_left);
  epc_right_ = move(config.epc_right);
  epc_ = move(config.epc);
  mpu_ = move(config.mpu);
  servo_ = move(config.servo);
  identity_ = config.identity;
  car_ = config.car;
}

void CarManager::UpdateParameters() {
  UpdateSpeed();
  UpdateSlope();
  UpdateServoAngle();
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

void CarManager::SetTargetSpeed(int16_t speed, MotorSide src) {
  if (epc_ != nullptr) {
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

void CarManager::SetTargetAngle(uint16_t angle) {
  if (servo_ == nullptr) {
    return;
  }
  ServoBounds s;
  switch (car_) {
    case Car::kCar1:
      s = kBoundsCar1;
      break;
    case Car::kCar2:
      s = kBoundsCar2;
      break;
  }
  if (angle > s.kLeftBound) {
    angle = s.kLeftBound;
  } else if (angle < s.kRightBound) {
    angle = s.kRightBound;
  }
  servo_->SetDegree(angle);
}

void CarManager::UpdateSpeed() {
  if (epc_ != nullptr) {
    epc_->DoCorrection();
    left_speed_ = epc_->GetCurrentSpeed(MpcDual::MotorSide::kLeft) / 100;
    right_speed_ = epc_->GetCurrentSpeed(MpcDual::MotorSide::kRight) / 100;
    return;
  }
  if (epc_left_ != nullptr) {
    epc_left_->DoCorrection();
    left_speed_ = epc_left_->GetCurrentSpeed() / 100;
  }
  if (epc_right_ != nullptr) {
    epc_right_->DoCorrection();
    right_speed_ = epc_right_->GetCurrentSpeed() / 100;
  }
}

void CarManager::UpdateServoAngle() {
  if (servo_ == nullptr) {
    return;
  }
  servo_deg_ = servo_->GetDegree();
}

void CarManager::UpdateSlope() {
  if (mpu_ == nullptr) {
    return;
  }
  // TODO(Derppening): Replace with call to MPU9250
  slope_deg_ = 0;
}
