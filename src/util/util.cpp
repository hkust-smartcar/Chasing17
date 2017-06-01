/*
 * util.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 *
 * Function implementations for util.h
 *
 */

#include "util/util.h"

#include "libsc/lcd_console.h"
#include "libsc/led.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libsc/k60/ov7725.h"

#include <array>
#include <cstring>
#include <string>
#include <vector>

#include "libbase/misc_types.h"

using libsc::LcdConsole;
using libsc::Led;
using libsc::St7735r;
using libsc::System;
using libsc::Timer;
using libsc::k60::JyMcuBt106;
using libsc::k60::Ov7725;
using std::array;
using std::memcpy;
using std::size_t;
using std::string;
using std::vector;

namespace util {
float CalcLinearRegressionSlope(const vector<int>& x, const vector<int>& y) {
  if (x.size() != y.size()) {
    return std::numeric_limits<float>::infinity();
  }
  array<array<int, 2>, 2> lhs_matrix{};
  array<int, 2> rhs_matrix{};

  // least squares method approximation
  for (unsigned int i = 0; i < x.size(); ++i) {
    lhs_matrix.at(0).at(0) += (x.at(i) * x.at(i));
    lhs_matrix.at(0).at(1) += x.at(i);
    lhs_matrix.at(1).at(0) += x.at(i);
    lhs_matrix.at(1).at(1) += 1;
    rhs_matrix.at(0) += (x.at(i) * y.at(i));
    rhs_matrix.at(1) += y.at(i);
  }

  // cramer's rule
  float det = (lhs_matrix.at(0).at(0) * lhs_matrix.at(1).at(1)) - (lhs_matrix.at(1).at(0) * lhs_matrix.at(0).at(1));
  float m = ((rhs_matrix.at(0) * lhs_matrix.at(1).at(1)) - (lhs_matrix.at(0).at(1) * rhs_matrix.at(1))) / det;

  return m;
}

void BtSendImage() {
  Led::Config led_config;
  led_config.id = 0;
  led_config.is_active_low = true;
  Led led1(led_config);
  led_config.id = 1;
  Led led2(led_config);
  led_config.id = 2;
  Led led3(led_config);
  led_config.id = 3;
  Led led4(led_config);

  led4.SetEnable(true);

  constexpr unsigned int kCameraWidth = 128;
  constexpr unsigned int kCameraHeight = 480;
  constexpr unsigned int kImageSize = kCameraWidth * kCameraHeight / 8;
  Ov7725::Config camera_config;
  camera_config.id = 0;
  camera_config.w = kCameraWidth;
  camera_config.h = kCameraHeight;
  Ov7725 camera(camera_config);
  camera.Start();

  St7735r::Config lcd_config;
  lcd_config.is_revert = true;
  St7735r lcd(lcd_config);

  JyMcuBt106::Config bt_config;
  bt_config.id = 0;
  bt_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
  JyMcuBt106 bt(bt_config);

  LcdConsole::Config console_config;
  console_config.lcd = &lcd;
  LcdConsole console(console_config);
  lcd.Clear();

  led4.SetEnable(false);

  Timer::TimerInt time_img = 0;
  constexpr Byte st = 170;
  unsigned int i = 0;

  while (true) {
    if (time_img != System::Time()) {
      time_img = System::Time();

      led1.SetEnable(System::Time() % 500 >= 250);
      if (time_img % 1000 == 0) {
        console.SetCursorRow(9);
//        console.WriteString(("Runtime: " + std::to_string(time_img / 1000) + "s").c_str());

        const Byte* image_ptr = camera.LockBuffer();
        array<Byte, kImageSize> image{};
        memcpy(image.data(), image_ptr, kImageSize);
        camera.UnlockBuffer();

      }
      if (time_img % 5000 == 0) {
        const Byte* image_ptr = camera.LockBuffer();
        array<Byte, kImageSize> image{};
        memcpy(image.data(), image_ptr, kImageSize);
        camera.UnlockBuffer();

        led3.SetEnable(true);

        led4.SetEnable(bt.SendBuffer(&st, 1));
        led2.SetEnable(bt.SendBuffer(image.data(), kImageSize));
        console.SetCursorRow(8);
//        console.WriteString(("Packet " + std::to_string(i++)).c_str());

        led3.SetEnable(false);
      }
    }
  }
}

void Int16To2ByteArray(const uint16_t num, array<Byte, 2>& bytes) {
  bytes.at(0) = static_cast<Byte>(num >> 8);
  bytes.at(1) = static_cast<Byte>(num);
}
}  // namespace util
