/*
 * fc_yy_us_v4.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Leslie Lee (LeeChunHei)
 *
 * Implementation for FcYyUsV4 class.
 *
 */

#include "fc_yy_us_v4.h"

#include "libsc/system.h"

using libsc::System;

uint32_t FcYyUsV4::impulse_start_time_ = 0;
unsigned int FcYyUsV4::distance_ = 0;

void FcYyUsV4::listener(Gpi* gpi) {
  if (gpi->Get()) {
    impulse_start_time_ = System::Time100Us();
  } else {
    unsigned int dist = (System::Time100Us() - impulse_start_time_) * 34;
    if (dist > 5500) {
      distance_ = kMaxDistance;
    } else if (dist < 20) {
      distance_ = kMinDistance;
    } else {
      distance_ = dist;
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
