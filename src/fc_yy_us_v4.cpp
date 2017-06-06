/*
 * fc_yy_us_v4.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Leslie Lee (LeeChunHei), Peter Tse (mcreng)
 *
 * Implementation for FcYyUsV4 class.
 *
 */

#include "fc_yy_us_v4.h"

#include "libsc/system.h"

#include <vector>

using libsc::System;

uint32_t FcYyUsV4::impulse_start_time_ = 0;
unsigned int FcYyUsV4::distance_ = 0;
unsigned int FcYyUsV4::average_distance_ = 0;
std::vector<unsigned int> FcYyUsV4::last_ten_distance_{};

void FcYyUsV4::listener(Gpi* gpi) {
  if (gpi->Get()) {
    impulse_start_time_ = System::Time10Us();
  } else {
    unsigned int dist = (System::Time10Us() - impulse_start_time_) * 6.8;
    if (dist > 2000) { //max: 5500, filter > 2000mm
      distance_ = kMaxDistance; average_distance_ = 0; return;
    } else if (dist < 20) {
      distance_ = kMinDistance; average_distance_ = 0; return;
    } else {
      distance_ = dist;
    }

    //average the distance
    last_ten_distance_.push_back(distance_);
    while (last_ten_distance_.size() > 10) last_ten_distance_.erase(last_ten_distance_.begin());
    average_distance_ = 0;
    for (auto&& m : last_ten_distance_){
    	average_distance_ += m;
    }
    average_distance_ /= static_cast<int32_t>(last_ten_distance_.size());
  }
}

FcYyUsV4::FcYyUsV4(libbase::k60::Pin::Name pin) {
  gpi_config_.pin = pin;
  gpi_config_.interrupt = Pin::Config::Interrupt::kBoth;
  gpi_config_.isr = listener;
  gpi_config_.config.set(Pin::Config::kPassiveFilter);
  m_pin_ = Gpi(gpi_config_);
}
