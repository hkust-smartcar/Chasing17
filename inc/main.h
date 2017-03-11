/**
 * main.h
 *
 * Author: Peter Tse (mcreng)
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstring>
#include <string>
#include "libbase/k60/mcg.h"
#include "libsc/system.h"
#include "libsc/led.h"
#include "libsc/button.h"
#include "libsc/k60/ov7725.h"
#include "libsc/st7735r.h"
#include "libsc/lcd_console.h"
#include "libsc/alternate_motor.h"
#include "libsc/futaba_s3010.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libsc/dir_encoder.h"
#include "libsc/joystick.h"
#include "libutil/misc.h"

#pragma once

namespace libbase {
namespace k60 {
Mcg::Config Mcg::GetMcgConfig() {
  Mcg::Config config;
  config.external_oscillator_khz = 50000;
  config.core_clock_khz = 150000;
  return config;
}

}  // namespace k60
}  // namespace libbase
