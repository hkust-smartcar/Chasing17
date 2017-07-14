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

uint16_t CarManager::pid_preset_ = 0;

CarManager::Edges CarManager::Edges::grad() {
  Edges temp;
  for (int i = 1; i < this->size(); i++) {
    auto last = this->points[i];
    auto second_last = this->points[i - 1];
    temp.push(last.first - second_last.first, last.second - second_last.second);
  }
  return temp;
}
