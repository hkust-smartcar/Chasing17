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
#include "libsc/battery_meter.h"
#include "libsc/lcd_console.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"

#include "debug_console.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/joystick.h"
#include "libbase/k60/flash.h"

#include "algorithm/optimal.h"
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

  {
  	St7735r::Config lcd_config;
  	lcd_config.is_revert = true;
  	St7735r lcd(lcd_config);
  	lcd.Clear();
    libsc::LcdTypewriter::Config writerconfig;
  	writerconfig.lcd = &lcd;
  	libsc::LcdTypewriter writer(writerconfig);

  	libsc::Joystick::Config joystick_config;
  	joystick_config.id = 0;
  	joystick_config.is_active_low = true;
  	libsc::Joystick joystick(joystick_config);

  	libbase::k60::Flash::Config flash_config;
  	libbase::k60::Flash flash(flash_config);

      DebugConsole console(&joystick,&lcd,&writer,10);

  	Item item("distract");
  //	console.PushItem(item);
  	console.PushItem(Item("RoundGoLeft" ,&algorithm::optimal::TuningVar.roundabout_turn_left,true));
  	console.PushItem(Item("startY" ,&algorithm::optimal::TuningVar.starting_y,true));
  	console.PushItem(Item("edgeLen",&algorithm::optimal::TuningVar.edge_length,true));
  	console.PushItem(Item("edgeHorMax",&algorithm::optimal::TuningVar.edge_hor_search_max,true));
  	console.PushItem(Item("edgeMinWld",&algorithm::optimal::TuningVar.edge_min_worldview_bound_check,true));
  	console.PushItem(Item("cornrRange",&algorithm::optimal::TuningVar.corner_range,true));
  	console.PushItem(Item("cornrRatio",&algorithm::optimal::TuningVar.corner_height_ratio,true));
  	console.PushItem(Item("cornerMin",&algorithm::optimal::TuningVar.corner_min,true));
  	console.PushItem(Item("cornerMax",&algorithm::optimal::TuningVar.corner_max,true));
  	console.PushItem(Item("minCrnrDist",&algorithm::optimal::TuningVar.min_corners_dist,true));
  	console.PushItem(Item("minEdgeDist",&algorithm::optimal::TuningVar.min_edges_dist,true));
  	console.PushItem(Item("WidthThers",&algorithm::optimal::TuningVar.track_width_threshold,true));
  	console.PushItem(Item("WidthChange",&algorithm::optimal::TuningVar.track_width_change_threshold,true));
  	console.PushItem(Item("sightDist",&algorithm::optimal::TuningVar.sightDist,true));
  	console.PushItem(Item("sightExit",&algorithm::optimal::TuningVar.sightDist_exitRound,true));
  	console.PushItem(Item("straightThres",&algorithm::optimal::TuningVar.straight_line_threshold,true));
  	console.PushItem(Item("ActionDist",&algorithm::optimal::TuningVar.action_distance,true));
  	console.PushItem(Item("stopDist",&algorithm::optimal::TuningVar.stop_distance,true));
  	console.PushItem(Item("Black/Length",&algorithm::optimal::TuningVar.black_div_length_ratio_threshold,true));
  	console.PushItem(Item("featureTime",&algorithm::optimal::TuningVar.feature_inside_time,true));
  	console.PushItem(Item("crossStart",&algorithm::optimal::TuningVar.cross_cal_start_num,true));
  	console.PushItem(Item("crossRatio",&algorithm::optimal::TuningVar.cross_cal_ratio,true));
  	console.PushItem(Item("generalCal#",&algorithm::optimal::TuningVar.general_cal_num,true));
  	console.PushItem(Item("crossEncoder",&algorithm::optimal::TuningVar.cross_encoder_count,true));
  	console.PushItem(Item("roundEncoder",&algorithm::optimal::TuningVar.round_encoder_count,true));
  	console.PushItem(Item("ExitEncoder",&algorithm::optimal::TuningVar.roundExit_encoder_count,true));
  	console.PushItem(Item("EnterOffset",&algorithm::optimal::TuningVar.round_enter_offset,true));
  	console.PushItem(Item("C1ServoOffset",&algorithm::optimal::TuningVar.car1_servo_offset,true));
  	console.PushItem(Item("C2ServoOffset",&algorithm::optimal::TuningVar.car2_servo_offset,true));
  	console.PushItem(Item("roundMin",&algorithm::optimal::TuningVar.cross_cal_start_num,true));

  	console.SetOffset(0);
  	console.SetFlash(&flash);

  	//Load();
  	console.EnterDebug(">enter program<");
    }


  // modify next line to switch between algorithms
  constexpr Algorithm a = Algorithm::kOptimal;

  // modify next line to change which car we're working with
  CarManager::Car c = CarManager::Car::kCar1;

  switch (a) {
    case Algorithm::kOptimal:
      algorithm::optimal::main(c);
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
