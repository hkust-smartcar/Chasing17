/* main.cpp
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 *
 * Program entry point.
 *
 */

#include "../inc/algorithm/optimal_car1.h"
#include "../inc/algorithm/optimal_car2.h"
#include "libbase/k60/mcg.h"
#include "libsc/battery_meter.h"
#include "libsc/lcd_console.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"
#include "libsc/joystick.h"

#include "debug.h"

#include "algorithm/distance.h"
#include "util/testground.h"
#include "util/util.h"
#include "car_manager.h"
#include "util/pid_tuning.h"

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

using libsc::BatteryMeter;
using libsc::Lcd;
using libsc::LcdConsole;
using libsc::St7735r;
using libsc::System;
using libsc::Joystick;

enum struct Algorithm {
  kOptimal,
  kTestGround,
  kDistance,
  kPID
};

int main() {
  System::Init();

  BatteryMeter::Config ConfigBM;
  ConfigBM.voltage_ratio = 0.4;
  BatteryMeter bm(ConfigBM);

  // Battery Check
  {
    St7735r::Config lcd_config;
    lcd_config.is_revert = true;
    St7735r lcd(lcd_config);
    lcd.Clear();

    LcdConsole::Config console_config;
    console_config.lcd = &lcd;
    LcdConsole console(console_config);

    float voltage;
    do {
      voltage = bm.GetVoltage();

      console.SetTextColor(voltage <= 7.4 ? Lcd::kRed : Lcd::kGreen);

      char temp[32];
      sprintf(temp, " Voltage: %.2fV", voltage);
      util::ConsoleClearRow(&console, 0);
      console.WriteString(temp);

      System::DelayMs(1000);
    } while (voltage <= 7.4);

  }

  // modify next line to switch between algorithms
  constexpr Algorithm a = Algorithm::kPID;

  // modify next line to change which car we're working with
  CarManager::Car c = CarManager::Car::kCar1;

  bool reset = false, skip_debug = false;
  {
    Joystick::Config joystick_config;
    joystick_config.id = 0;
    joystick_config.is_active_low = true;
    Joystick joystick(joystick_config);

    reset = (joystick.GetState() == Joystick::State::kSelect ? true : false);
//    skip_debug = (joystick.GetState() == Joystick::State::kIdle ? true : false);
  }

  if (!skip_debug)
    switch (debug(reset)) {
      case 1:
        c = CarManager::Car::kCar1;
        break;
      case 2:
        c = CarManager::Car::kCar2;
        break;
    }

  switch (a) {
    case Algorithm::kOptimal:
      switch (c) {
        case CarManager::Car::kCar1:
          algorithm::optimal::car1::main_car1(debug_flag::lcd_debug);
          break;
        case CarManager::Car::kCar2:
          algorithm::optimal::car2::main_car2(debug_flag::lcd_debug);
      }

      break;
    case Algorithm::kTestGround:
      util::testground::main();
      break;
    case Algorithm::kDistance:
      algorithm::USIRDemo();
      break;
    case Algorithm::kPID:
      util::pid_tuning::main();
      break;
    default:
      // all cases covered
      break;
  }

  while (true);

  return 0;
}
