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
int FcYyUsV4::distance_ = 0;

void FcYyUsV4::listener(Gpi* gpi) {
  if (gpi->Get()) {
    impulse_start_time_ = System::Time();
  } else {
    int temp = (System::Time() - impulse_start_time_) * 340;
    if (temp < 550000 && temp > 20) {
      distance_ = temp;
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
