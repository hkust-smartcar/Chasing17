/*
 * main.cpp
 *
 * Author: Peter Tse (mcreng)
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Entry point for the program, and houses the main loop.
 *
 */

#include "algorithm/receiver.h"

#include "libsc/lcd_console.h"
#include "libsc/led.h"
#include "libsc/system.h"
#include "libsc/k60/jy_mcu_bt_106.h"

#include "bluetooth.h"

using namespace libsc;

namespace algorithm {

void receiver() {
  Led::Config led_config;
  led_config.is_active_low = true;
  led_config.id = 0;
  Led led1(led_config);
  led_config.id = 1;
  Led led2(led_config);
  led_config.id = 2;
  Led led3(led_config);
  led_config.id = 3;
  Led led4(led_config);
  led1.SetEnable(false);
  led2.SetEnable(false);
  led3.SetEnable(false);
  led4.SetEnable(true);

  k60::JyMcuBt106::Config ConfigBT;
  ConfigBT.id = 0;
  ConfigBT.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
  BTComm bt(ConfigBT);

  bt.led_ptr = &led2;

  St7735r::Config lcd_config;
  lcd_config.is_revert = true;
  St7735r lcd(lcd_config);
  lcd.Clear();

  LcdConsole::Config console_config;
  console_config.lcd = &lcd;
  LcdConsole console(console_config);

  led4.SetEnable(false);

  Timer::TimerInt time_img = 0;

  /* Main Loop */
  while (1) {
    while (time_img != System::Time()) {
      time_img = System::Time();

      led1.SetEnable(time_img % 500 >= 250);

      if (time_img % 500) {
        console.SetCursorRow(0);
        std::string s;
//        s += "speed = " + std::to_string(bt.getBufferSpeed()) + "\n";
//        s += "slope = " + std::to_string(bt.getBufferSlopeDeg()) + "\n";
//        s += "servo = " + std::to_string(bt.getBufferSlopeDeg()) + "\n";
//        s += "featu = " + std::to_string(static_cast<int>(bt.getBufferFeature())) + "\n";
//        s += "side  = " + std::to_string(static_cast<int>(bt.getBufferSide())) + "\n";
        console.WriteString(s.c_str());
      }
    }
  }
}

}  // namespace algorithm
