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

#include "util/util.h"

using libsc::FutabaS3010;
using std::move;
using std::unique_ptr;

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

constexpr CarManager::PidValues CarManager::kMotorPidCar1;
constexpr CarManager::PidValues CarManager::kMotorPidCar2;

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
  switch (car_) {
    default:
    case CarManager::Car::kCar1:
      return kMotorPidCar1;
    case CarManager::Car::kCar2:
      return kMotorPidCar2;
  }
}
