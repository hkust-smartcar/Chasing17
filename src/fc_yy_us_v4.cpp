/*
 * fc_yy_us_v4.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Leslie Lee (LeeChunHei), Peter Tse (mcreng), David Mak (Derppening)
 *
 * Implementation for FcYyUsV4 class.
 *
 */

#include "fc_yy_us_v4.h"

#include "libsc/system.h"

#include <vector>
#include <cmath>

using libsc::System;

uint32_t FcYyUsV4::impulse_start_time_ = 0;
uint16_t FcYyUsV4::distance_ = 0;
uint16_t FcYyUsV4::average_distance_ = 0;
std::vector<uint16_t> FcYyUsV4::last_ten_distance_{};
uint16_t FcYyUsV4::std_deviation_ = 0;

void FcYyUsV4::listener(Gpi* gpi) {
  if (gpi->Get()) {
    // TODO(Derppening): Determine why we switched to System::Time() again
    impulse_start_time_ = System::Time();  // Time10Us();
    average_distance_ = 0;
    std_deviation_ = 0;

  } else {
    uint16_t dist = (System::Time() - impulse_start_time_) * 6.8;
    if (dist > 2000) { //max: 5500, filter > 2000mm
      distance_ = kMaxDistance; average_distance_ = 0; return;
    } else if (dist < 20) {
      distance_ = kMinDistance; average_distance_ = 0; return;
    } else {
      distance_ = dist;
    }

    uint16_t old_average_distance_ = average_distance_;
    uint16_t old_std_deviation_ = std_deviation_;

    //average the distance
    last_ten_distance_.push_back(distance_);
    while (last_ten_distance_.size() > 10) last_ten_distance_.erase(last_ten_distance_.begin());
    average_distance_ = 0;
    for (auto&& m : last_ten_distance_){
      average_distance_ += m;
    }
    average_distance_ /= static_cast<int32_t>(last_ten_distance_.size());

    //find S.D.
    std_deviation_ = 0;
    for (auto&& m : last_ten_distance_){
      std_deviation_ += (m-average_distance_) * (m-average_distance_);
    }
    std_deviation_ = std::sqrt(std_deviation_ / last_ten_distance_.size());

    if (std_deviation_ > 150){  // filter outliers
      average_distance_ = old_average_distance_;
      std_deviation_ = old_std_deviation_;
    }
  }
}

FcYyUsV4::FcYyUsV4(libbase::k60::Pin::Name pin) {
  gpi_config_.pin = pin;
  gpi_config_.interrupt = Pin::Config::Interrupt::kBoth;
  gpi_config_.isr = listener;
  gpi_config_.config.set(Pin::Config::kPassiveFilter);
  m_pin_ = Gpi(gpi_config_);
}
