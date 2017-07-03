/*
 * car_manager.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening), Peter Tse (mcreng)
 *
 * Implementations for CarManager struct.
 *
 */

#include "car_manager.h"

uint16_t CarManager::us_distance_ = 0;
int32_t CarManager::left_speed_ = 0;
int32_t CarManager::right_speed_ = 0;
uint16_t CarManager::servo_deg_ = 0;
int8_t CarManager::slope_deg_ = 0;
CarManager::Side CarManager::side_ = CarManager::Side::kMiddle;
CarManager::Feature CarManager::feature_ = CarManager::Feature::kStraight;
CarManager::Identity CarManager::identity_ = CarManager::Identity::kFront;
CarManager::Car CarManager::car_ = CarManager::Car::kCar1;

constexpr CarManager::ServoAngles CarManager::kAnglesCar1;
constexpr CarManager::ServoAngles CarManager::kAnglesCar2;

constexpr CarManager::SideRatio CarManager::kRatioCar1;
constexpr CarManager::SideRatio CarManager::kRatioCar2;

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

CarManager::ServoAngles CarManager::GetServoAngles(Car c) {
  switch (c) {
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

CarManager::SideRatio CarManager::GetSideRatio(Car c) {
  switch (c) {
    default:
      // all cases covered
    case Car::kCar1:
      return kRatioCar1;
    case Car::kCar2:
      return kRatioCar2;
  }
}
