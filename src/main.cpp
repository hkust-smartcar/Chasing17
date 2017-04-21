/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 *
 * Program entry point.
 *
 */

#include "libbase/k60/mcg.h"
#include "libsc/system.h"

#include "algorithm/bt-demo.h"
#include "algorithm/receiver.h"
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
  kLeslie,
  kReceiver,
  kBluetoothTest,
  kKingReceive
};

int main() {
  System::Init();

  // modify next line to switch between algorithms
  constexpr Algorithm a = Algorithm::kKingReceive;

  // modify next line to enable/disable encoder
  constexpr bool has_encoder = false;

  // modify next line to change which car we're working with
  CarManager::Car c = CarManager::Car::kOld;

  CarManager::ServoBounds s = c == CarManager::Car::kOld ? CarManager::old_car : CarManager::new_car;
  switch (a) {
    case Algorithm::kKing:
      algorithm::king::main(has_encoder, s);
      break;
    case Algorithm::kLeslie:
      algorithm::leslie::main(has_encoder);
      break;
    case Algorithm::kReceiver:
      algorithm::receiver();
      break;
    case Algorithm::kBluetoothTest:
      algorithm::BluetoothDemo(has_encoder);
      break;
    case Algorithm::kKingReceive:
      algorithm::king::main_receive(has_encoder, s);
      break;
    default:
      // not handled
      break;
  }

  while (true) {
  }

  return 0;
}
