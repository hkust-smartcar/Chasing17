/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 */

#include <string>

#include "libbase/k60/mcg.h"
#include "libsc/system.h"

#include "algorithm/king/main.h"
#include "algorithm/leslie/main.h"

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

using libsc::System;

enum struct Algorithm {
  kKing,
  kLeslie
};

int main() {
  System::Init();

  // modify next line to switch between algorithms
  constexpr Algorithm a = Algorithm::kKing;

  // modify next line to enable/disable encoder
  constexpr bool has_encoder = false;

  if (a == Algorithm::kKing) {
    algorithm::king::main(has_encoder);
  } else {
    algorithm::leslie::main(has_encoder);
  }

  while (true) {
  }

  return 0;
}
