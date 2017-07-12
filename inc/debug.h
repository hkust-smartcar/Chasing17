/*
 * debug.h
 *
 *  Created on: Jun 28, 2017
 *      Author: dipsy
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#include "libsc/st7735r.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/joystick.h"
#include "libbase/k60/flash.h"

#include "debug_console.h"
#include "car_manager.h"
#include "algorithm/optimal_car1.h"
#include "algorithm/optimal_car2.h"
#include "algorithm/optimal_car3.h"
#include "algorithm/optimal_car4.h"

using libsc::St7735r;
using libsc::System;
using libsc::LcdTypewriter;
using libsc::Joystick;
using libbase::k60::Flash;

uint16_t car = 0;
bool confirm = false;

namespace debug_flag{
	  bool lcd_debug = false;
}

void confirmCar() {
  confirm = true;
}

void loadItems(DebugConsole* console) {
  console->PushItem("config", &CarManager::config, 1.0);

  switch (car) {
    case 1: {
      using namespace algorithm::optimal::car1::TuningVar;

      // misc
      console->PushItem("lcd debug", &debug_flag::lcd_debug);
      console->PushItem("algo time", &show_algo_time, "true", "false");
      console->PushItem("single test", &single_car_testing, "true", "false");
      console->PushItem("overt_sel", &roundabout_overtake_flag, "y", "n");
      console->PushItem("rndabt sel", &roundabout_shortest_flag, "l", "r");
      console->PushItem("over_time", &overtake_interval_time, 5);
      console->PushItem("exit_dist", &exit_action_dist, 1);
      console->PushItem("rnd_off", &roundabout_offset, 1);
      console->PushItem("rndE_off", &round_exit_offset, 1);
//    console->PushItem("corner_size", &corner_range, 1);
//    console->PushItem("corner_min", &corner_min, 1);
//    console->PushItem("corner_max", &corner_max, 1);

      // speed
      console->PushItem("target spd:");
      console->PushItem("slow", &targetSpeed_slow, 5);
      console->PushItem("strght", &targetSpeed_straight, 5);
      console->PushItem("normal", &targetSpeed_normal, 5);
      console->PushItem("rndabt", &targetSpeed_round, 5);
      console->PushItem("s_turn", &targetSpeed_sharp_turn, 5);
      console->PushItem("insideRnd", &targetSpeed_inside, 5);

      // servo
      console->PushItem("servo pid:");
      console->PushItem("strght-p-r", &servo_straight_kp_right, 0.01);
      console->PushItem("strght-d-r", &servo_straight_kd_right, 0.001);
      console->PushItem("strght-p-l", &servo_straight_kp_left, 0.01);
      console->PushItem("strght-d-l", &servo_straight_kd_left, 0.001);
      console->PushItem("normal-p-r", &servo_normal_kp_right, 0.01);
      console->PushItem("normal-d-r", &servo_normal_kd_right, 0.001);
      console->PushItem("normal-p-l", &servo_normal_kp_left, 0.01);
      console->PushItem("normal-d-l", &servo_normal_kd_left, 0.001);
      console->PushItem("rndabt-p-r", &servo_roundabout_kp_right, 0.01);
      console->PushItem("rndabt-d-r", &servo_roundabout_kd_right, 0.001);
      console->PushItem("rndabt-p-l", &servo_roundabout_kp_left, 0.01);
      console->PushItem("rndabt-d-l", &servo_roundabout_kd_left, 0.001);
      console->PushItem("s_turn-p-r", &servo_sharp_turn_kp_right, 0.01);
      console->PushItem("s_turn-d-r", &servo_sharp_turn_kd_right, 0.001);
      console->PushItem("s_turn-p-l", &servo_sharp_turn_kp_left, 0.01);
      console->PushItem("s_turn-d-l", &servo_sharp_turn_kd_left, 0.001);
    }
    break;
    case 2: {
      using namespace algorithm::optimal::car2::TuningVar;

      // misc
      console->PushItem("lcd debug", &debug_flag::lcd_debug);
      console->PushItem("algo time", &show_algo_time, "yes", "no");
      console->PushItem("single test", &single_car_testing, "true", "false");
      console->PushItem("distance", &start_car_distance, 10);
      console->PushItem("overt_sel", &roundabout_overtake_flag, "y", "n");
      console->PushItem("rndabt sel", &roundabout_shortest_flag, "l", "r");
      console->PushItem("over_time", &overtake_interval_time, 5);
      console->PushItem("exit_dist", &exit_action_dist, 1);
      console->PushItem("rnd_off", &roundabout_offset, 1);
      console->PushItem("rndE_off", &round_exit_offset, 1);
//    console->PushItem("corner_size", &corner_range, 1);
//    console->PushItem("corner_min", &corner_min, 1);
//    console->PushItem("corner_max", &corner_max, 1);

      // speed
      console->PushItem("target spd:");
      console->PushItem("slow", &targetSpeed_slow, 5);
      console->PushItem("strght", &targetSpeed_straight, 5);
      console->PushItem("normal", &targetSpeed_normal, 5);
      console->PushItem("rndabt", &targetSpeed_round, 5);
      console->PushItem("s_turn", &targetSpeed_sharp_turn, 5);
      console->PushItem("insideRnd", &targetSpeed_inside, 5);

      // servo
      console->PushItem("servo pid:");
      console->PushItem("strght-p-r", &servo_straight_kp_right, 0.01);
      console->PushItem("strght-d-r", &servo_straight_kd_right, 0.001);
      console->PushItem("strght-p-l", &servo_straight_kp_left, 0.01);
      console->PushItem("strght-d-l", &servo_straight_kd_left, 0.001);
      console->PushItem("normal-p-r", &servo_normal_kp_right, 0.01);
      console->PushItem("normal-d-r", &servo_normal_kd_right, 0.001);
      console->PushItem("normal-p-l", &servo_normal_kp_left, 0.01);
      console->PushItem("normal-d-l", &servo_normal_kd_left, 0.001);
      console->PushItem("rndabt-p-r", &servo_roundabout_kp_right, 0.01);
      console->PushItem("rndabt-d-r", &servo_roundabout_kd_right, 0.001);
      console->PushItem("rndabt-p-l", &servo_roundabout_kp_left, 0.01);
      console->PushItem("rndabt-d-l", &servo_roundabout_kd_left, 0.001);
      console->PushItem("s_turn-p-r", &servo_sharp_turn_kp_right, 0.01);
      console->PushItem("s_turn-d-r", &servo_sharp_turn_kd_right, 0.001);
      console->PushItem("s_turn-p-l", &servo_sharp_turn_kp_left, 0.01);
      console->PushItem("s_turn-d-l", &servo_sharp_turn_kd_left, 0.001);
    }
    break;
    case 3: {
      using namespace algorithm::optimal::car3::TuningVar;

      // misc
      console->PushItem("lcd debug", &debug_flag::lcd_debug);
      console->PushItem("algo time", &show_algo_time, "true", "false");
      console->PushItem("single test", &single_car_testing, "true", "false");
      console->PushItem("overt_sel", &roundabout_overtake_flag, "y", "n");
      console->PushItem("rndabt sel", &roundabout_shortest_flag, "l", "r");
      console->PushItem("over_time", &overtake_interval_time, 5);
//      console->PushItem("corner_size", &corner_range, 1);
//      console->PushItem("corner_min", &corner_min, 1);
//      console->PushItem("corner_max", &corner_max, 1);

      // speed
      console->PushItem("target spd:");
      console->PushItem("slow", &targetSpeed_slow, 5);
      console->PushItem("strght", &targetSpeed_straight, 5);
      console->PushItem("normal", &targetSpeed_normal, 5);
      console->PushItem("rndabt", &targetSpeed_round, 5);
      console->PushItem("s_turn", &targetSpeed_sharp_turn, 5);

      // servo
      console->PushItem("servo pid:");
      console->PushItem("strght-p", &servo_straight_kp, 0.1);
      console->PushItem("strght-d", &servo_straight_kd, 0.001);
      console->PushItem("normal-p", &servo_normal_kp, 0.1);
      console->PushItem("normal-d", &servo_normal_kd, 0.001);
      console->PushItem("rndabt-p", &servo_roundabout_kp, 0.1);
      console->PushItem("rndabt-d", &servo_roundabout_kd, 0.001);
      console->PushItem("s_turn-p", &servo_sharp_turn_kp, 0.1);
      console->PushItem("s_turn-d", &servo_sharp_turn_kd, 0.001);
    }
    break;
    case 4: {
      using namespace algorithm::optimal::car4::TuningVar;

      // misc
      console->PushItem("lcd debug", &debug_flag::lcd_debug);
      console->PushItem("algo time", &show_algo_time, "true", "false");
      console->PushItem("single test", &single_car_testing, "true", "false");
      console->PushItem("distance", &start_car_distance, 10);
      console->PushItem("overt_sel", &roundabout_overtake_flag, "y", "n");
      console->PushItem("rndabt sel", &roundabout_shortest_flag, "l", "r");
      console->PushItem("over_time", &overtake_interval_time, 5);
//      console->PushItem("corner_size", &corner_range, 1);
//      console->PushItem("corner_min", &corner_min, 1);
//      console->PushItem("corner_max", &corner_max, 1);

      // speed
      console->PushItem("target spd:");
      console->PushItem("slow", &targetSpeed_slow, 5);
      console->PushItem("strght", &targetSpeed_straight, 5);
      console->PushItem("normal", &targetSpeed_normal, 5);
      console->PushItem("rndabt", &targetSpeed_round, 5);
      console->PushItem("s_turn", &targetSpeed_sharp_turn, 5);

      // servo
      console->PushItem("servo pid:");
      console->PushItem("strght-p", &servo_straight_kp, 0.1);
      console->PushItem("strght-d", &servo_straight_kd, 0.001);
      console->PushItem("normal-p", &servo_normal_kp, 0.1);
      console->PushItem("normal-d", &servo_normal_kd, 0.001);
      console->PushItem("rndabt-p", &servo_roundabout_kp, 0.1);
      console->PushItem("rndabt-d", &servo_roundabout_kd, 0.001);
      console->PushItem("s_turn-p", &servo_sharp_turn_kp, 0.1);
      console->PushItem("s_turn-d", &servo_sharp_turn_kd, 0.001);

    }
    break;
    default:
      // not handled
      break;
  }
}

uint16_t debug(bool call_reset) {

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

  if(call_reset){
	  DebugConsole resetConfirmConsole(&joystick, &lcd, &writer);
	  resetConfirmConsole.PushItem("Sure Reset?",&call_reset,"yes","no");
	  resetConfirmConsole.PushItem("Confirm");
	  Item item = resetConfirmConsole.GetItem(1);
	  item.listener = &confirmCar;
	  resetConfirmConsole.SetItem(1,item);
	  resetConfirmConsole.ListItems();
	  while(!confirm){
		  resetConfirmConsole.Listen();
	  }
	  confirm=false;
  }


  Flash::Config flash_config;
  Flash flash(flash_config);

  DebugConsole console(&joystick, &lcd, &writer);

  console.SetFlash(&flash);
  console.PushItem("confirm: car", &car, 1);
  Item item = console.GetItem(0);
  item.listener = &confirmCar;
  console.SetItem(0, item);
  console.Load();

  if (call_reset) {
    car = 0;
  }
  while (car < 1 || car > 4) {
    confirm = false;
    call_reset = true;
    console.ListItems();
    while (!confirm) {
      console.Listen();
      console.Save();
    }
  }
  confirm = false;
  item.type = VarType::kNan;
  switch (car) {
    case 1:
      item.text = ">>run car1<<";
      break;
    case 2:
      item.text = ">>run car2<<";
      break;
    case 3:
      item.text = ">>run stb1<<";
      break;
    case 4:
      item.text = ">>run stb2<<";
      break;
    default:
      // all cases covered
      break;
  }
  console.SetItem(0, item);

  loadItems(&console);

  if (!call_reset) console.Load();

  console.ListItems();
  while (!confirm) {
    console.Listen();
    console.Save();
  }

  return car;
}

#endif /* INC_DEBUG_H_ */
