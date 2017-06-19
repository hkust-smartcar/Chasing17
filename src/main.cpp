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

#include "libbase/k60/flash.h"
#include "debug_console.h"

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
    if (bm.GetVoltage() <= 7.4){
      console.SetTextColor(Lcd::kRed);
    } else {
      console.SetTextColor(Lcd::kGreen);
    }
    char temp[32];
    sprintf(temp, " Voltage: %.2fV", bm.GetVoltage());
    console.WriteString(temp);
    System::DelayMs(1000);
    while (bm.GetVoltage() <= 7.4);
  }



  {
	St7735r::Config lcd_config;
	lcd_config.is_revert = true;
	St7735r lcd(lcd_config);
	lcd.Clear();
    LcdTypewriter::Config writerconfig;
	writerconfig.lcd = &lcd;
	LcdTypewriter writer(writerconfig);

	Joystick::Config joystick_config;
	joystick_config.id = 0;
	joystick_config.is_active_low = true;
	Joystick joystick(joystick_config);

	Flash::Config flash_config;
	Flash flash(flash_config);

    DebugConsole console(&joystick,&lcd,&writer,10);

	Item item("distract");
//	console.PushItem(item);
	console.PushItem(Item("startY" ,&algorithm::optimal::TuningVar.starting_y,true));
	console.PushItem(Item("edgeLen",&algorithm::optimal::TuningVar.edge_length,true));
	console.PushItem(Item("crnrRng",&algorithm::optimal::TuningVar.corner_range,true));
	console.PushItem(Item("crnrHgtRatio",&algorithm::optimal::TuningVar.corner_height_ratio,true));
	console.PushItem(Item("crnrMin",&algorithm::optimal::TuningVar.corner_min,true));
	console.PushItem(Item("crnrMax",&algorithm::optimal::TuningVar.corner_max,true));
	console.PushItem(Item("minCrnrD",&algorithm::optimal::TuningVar.min_corners_dist,true));
	console.PushItem(Item("minEdgeD",&algorithm::optimal::TuningVar.min_edges_dist,true));
	console.PushItem(Item("trckWdThrsh",&algorithm::optimal::TuningVar.track_width_threshold,true));
	console.PushItem(Item("trckWdCThrs",&algorithm::optimal::TuningVar.track_width_change_threshold,true));
	console.PushItem(Item("sightDist",&algorithm::optimal::TuningVar.sightDist,true));
	console.PushItem(Item("strgtLThrs",&algorithm::optimal::TuningVar.straight_line_threshold,true));
	console.PushItem(Item("stopDist",&algorithm::optimal::TuningVar.stop_distance,true));
	console.PushItem(Item("blkDvLRtT",&algorithm::optimal::TuningVar.black_div_length_ratio_thresold,true));

	console.SetOffset(0);
	console.SetFlash(&flash);

	//Load();
	console.EnterDebug(">enter program<");
  }

  // modify next line to switch between algorithms
  constexpr Algorithm a = Algorithm::kOptimal;

  // modify next line to enable/disable encoder
  constexpr bool has_encoder = true;

  // modify next line to change which car we're working with
  CarManager::Car c = CarManager::Car::kCar1;

//  CarManager::ServoBounds s = c == CarManager::Car::kCar1 ? CarManager::kBoundsCar1 : CarManager::kBoundsCar2;
  CarManager::ServoBounds s;
  if(c==CarManager::Car::kCar1){
	  s = CarManager::kBoundsCar1;
  }
  else{
	  s = CarManager::kBoundsCar2;
  }
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
