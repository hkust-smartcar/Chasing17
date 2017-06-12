/* main.cpp
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
#include "libsc/battery_meter.h"

#include "algorithm/david/main.h"
#include "algorithm/king/main.h"
#include "algorithm/leslie/main.h"
#include "algorithm/optimal/main.h"
#include "algorithm/distance.h"
#include "util/testground.h"
#include "util/unit_tests.h"

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
  kKingReceive,
  kOptimal,
  kTestGround,
  kDavid,
  kDistance
};

int main() {
  System::Init();

//  util::CameraTest();

  BatteryMeter::Config ConfigBM;
  ConfigBM.voltage_ratio = 0.4;
  BatteryMeter bm(ConfigBM);
  // Battery Check
  // TODO(Derppening): Find a better way to halt program when battery is lower than expected
  while (bm.GetVoltage() <= 7.4);

  // modify next line to switch between algorithms
  constexpr Algorithm a = Algorithm::kOptimal;

  // modify next line to enable/disable encoder
  constexpr bool has_encoder = true;

  // modify next line to change which car we're working with
  CarManager::Car c = CarManager::Car::kCar1;

  CarManager::ServoBounds s = c == CarManager::Car::kCar1 ? CarManager::kBoundsCar1 : CarManager::kBoundsCar2;
  switch (a) {
    case Algorithm::kKing:
      algorithm::king::main(has_encoder, s);
      break;
    case Algorithm::kLeslie:
      algorithm::leslie::main(has_encoder);
      break;
    case Algorithm::kKingReceive:
      algorithm::king::main_receive(has_encoder, s);
      break;
    case Algorithm::kOptimal:
      algorithm::optimal::main(c);
      break;
    case Algorithm::kDavid:
      algorithm::david::main();
      break;
    case Algorithm::kTestGround:
      util::testground::main();
      break;
    case Algorithm::kDistance:
      algorithm::USIRDemo();
      break;
    default:
      // all cases covered
      break;
  }

  while (true) {
  }

  return 0;
}
