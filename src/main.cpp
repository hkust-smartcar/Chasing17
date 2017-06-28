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
#include "libsc/lcd_typewriter.h"
#include "libsc/joystick.h"
#include "libbase/k60/flash.h"
#include "debug_console.h"

#include "algorithm/distance.h"
#include "util/testground.h"
#include "util/util.h"

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
using libsc::LcdTypewriter;
using libsc::Joystick;
using libbase::k60::Flash;

enum struct Algorithm {
  kOptimal,
  kTestGround,
  kDistance
};

int main() {
  System::Init();

  BatteryMeter::Config ConfigBM;
  ConfigBM.voltage_ratio = 0.4;
  BatteryMeter bm(ConfigBM);

  {
	// Battery Check
    St7735r::Config lcd_config;
    lcd_config.is_revert = true;
    St7735r lcd(lcd_config);
    lcd.Clear();

    LcdConsole::Config console_config;
    console_config.lcd = &lcd;
    LcdConsole lcdconsole(console_config);

    float voltage;
    do {
      voltage = bm.GetVoltage();

      lcdconsole.SetTextColor(voltage <= 7.4 ? Lcd::kRed : Lcd::kGreen);

      char temp[32];
      sprintf(temp, " Voltage: %.2fV", voltage);
      util::ConsoleClearRow(&lcdconsole, 0);
      lcdconsole.WriteString(temp);

      lcdconsole.WriteString("\nlong press hard reset");

      System::DelayMs(1000);
    } while (voltage <= 7.4);

    LcdTypewriter::Config writerconfig;
	writerconfig.lcd = &lcd;
	LcdTypewriter writer(writerconfig);

	Joystick::Config joystick_config;
	joystick_config.id = 0;
	joystick_config.is_active_low = true;
	Joystick joystick(joystick_config);

	Flash::Config flash_config;
	Flash flash(flash_config);

	DebugConsole console(&joystick,&lcd,&writer);
	bool isCar1=true,car_img;
	if(joystick.GetState()==Joystick::State::kIdle)
		console.SetFlash(&flash);
	console.PushItem("car",&isCar1,"1","2");
	console.Load();
	car_img = isCar1;

	typedef algorithm::optimal::car1::TuningVar V;
	V* v = &algorithm::optimal::car2::TuningVar;






	console.EnterDebug(">> start <<");

  }

  // modify next line to switch between algorithms
  constexpr Algorithm a = Algorithm::kOptimal;

  // modify next line to change which car we're working with
  constexpr CarManager::Car c = CarManager::Car::kCar1;

  switch (a) {
    case Algorithm::kOptimal:
    	switch (c){
    	case CarManager::Car::kCar1:
    		algorithm::optimal::car1::main_car1(false);
    		break;
    	case CarManager::Car::kCar2:
    		algorithm::optimal::car2::main_car2(false);
    	}

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

  while (true);

  return 0;
}
