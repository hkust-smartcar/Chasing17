/* overtake.cpp
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening), Peter Tse (mcreng)
 *
 * Implementations for Overtake class.
 *
 */

#include "algorithm/overtake.h"

#include <memory>

#include "libbase/misc_types.h"
#include "libsc/infra_red_sensor.h"
#include "libsc/system.h"
#include "libsc/us_100.h"

#include "bluetooth.h"
#include "car_manager.h"
#include "util/mpc.h"

using libsc::InfraRedSensor;
using libsc::System;
using libsc::Timer;
using libsc::Us100;
using std::move;
using std::unique_ptr;
using util::Mpc;

bool Overtake::is_overtaking_ = false;
Timer::TimerInt Overtake::time_begin_ = 0;
uint16_t Overtake::dist_ = 0;

unique_ptr<BTComm> Overtake::bluetooth_ = nullptr;
unique_ptr<InfraRedSensor> Overtake::ir_front_ = nullptr;
unique_ptr<InfraRedSensor> Overtake::ir_rear_ = nullptr;
unique_ptr<Us100> Overtake::us_ = nullptr;

// TODO(Derppening): Separate everything into tinier functions

bool Overtake::DecideOvertake() {
  if (static_cast<bool>(CarManager::GetIdentity())) { // front car
    if (bluetooth_->getOvertakeReq() == BTComm::OvertakeStatus::kRequested) {  // rear car requested overtake
      // TODO(Derppening): Add cases for rejection
      bluetooth_->replyOvertakeReq(true);
      is_overtaking_ = true;
      return true;
    } else {  // send current data to rear car
      bluetooth_->sendInfo(CarManager::GetLeftSpeed(), CarManager::GetSlope(), dist_ / 10, CarManager::GetSide());
      bluetooth_->sendFeature(CarManager::GetFeature());
      is_overtaking_ = false;
      return false;
    }
  }

  if (!(ir_front_->IsDetected())) {
    is_overtaking_ = false;
    return false;
  }

  // currently, only request overtake if feature is straight line
  // seems like features aren't implemented yet, so...
  if (bluetooth_->getBufferFeature() != CarManager::Feature::kStraight) {
    is_overtaking_ = false;
    return false;
  }

  // why are we on the same side?
  // why am i in the middle? follow left/right!
  if (CarManager::GetSide() == bluetooth_->getBufferSide() ||
      CarManager::GetSide() == CarManager::Side::kMiddle) {
    is_overtaking_ = false;
    return false;
  }

  // seems like we're ready to overtake.
  // check our status
  switch (bluetooth_->getOvertakeReq()) {
    case BTComm::OvertakeStatus::kRejected:
      // recently rejected. reset it and try again
      bluetooth_->resetOvertakeReq();
    case BTComm::OvertakeStatus::kNeutral:
      bluetooth_->reqOvertake();
      break;
    case BTComm::OvertakeStatus::kRequested:
      // requested. don't bother sending another packet
      break;
    case BTComm::OvertakeStatus::kAgreed:
      // no such case
      break;
  }

  return true;
}

bool Overtake::ExecuteOvertake() {
  // update parameters first. the data will be handy anyways
  UpdateParameters();

  // no agreement on overtaking. decide instead
  if (bluetooth_->getOvertakeReq() != BTComm::OvertakeStatus::kAgreed ||
      !is_overtaking_) {
    DecideOvertake();
    return false;
  }

  if (time_begin_ == 0) {
    time_begin_ = System::Time();
  } else if (Timer::TimeDiff(System::Time(), time_begin_) > kOvertakeTimeout) {
    Cleanup(static_cast<bool>(CarManager::GetIdentity()));
  }

  is_overtaking_ = true;

  if (static_cast<bool>(CarManager::GetIdentity())) {
    us_->Start();

    bluetooth_->sendInfo(CarManager::GetLeftSpeed(), CarManager::GetSlope(), dist_ / 10, CarManager::GetSide());

    if (ir_front_->IsDetected() &&
        dist_ > 50 && dist_ < 100) {
      CarManager::SwitchIdentity();
      Cleanup(false);
      bluetooth_->ReqSwitchID();
    }
  } else {
    us_->Stop();

    // TODO(Derppening): calculate speed difference, then +50% to catch up
    CarManager::SetTargetSpeed(4000);

    if (bluetooth_->hasSwitchIDReq()) {
      CarManager::SwitchIdentity();
      Cleanup(true);
    }
  }
  return true;
}

void Overtake::Init(Config config) {
  bluetooth_ = move(config.bluetooth);
  ir_front_ = move(config.ir_front);
  ir_rear_ = move(config.ir_rear);
  us_ = move(config.us);
}

void Overtake::Cleanup(bool identity) {
  if (identity) {
    if (us_->IsAvailable()) {
      us_->Stop();
    }
  } else {
    if (!(us_->IsAvailable())) {
      us_->Start();
    }
  }
  time_begin_ = 0;
}

void Overtake::UpdateParameters() {
  CarManager::UpdateParameters();
  UpdateDist();
}

void Overtake::UpdateDist() {
  if (static_cast<bool>(CarManager::GetIdentity())) {
    dist_ = kDistanceInf;
  } else {
    dist_ = us_->GetDistance();
  }
}
