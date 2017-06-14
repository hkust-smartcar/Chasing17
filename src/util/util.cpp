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
#include <cstdio>
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
using std::sprintf;
using std::size_t;
using std::string;
using std::vector;

namespace util {
constexpr uint8_t kToStringBufferSize = 32;

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
        console.WriteString(("Runtime: " + to_string(time_img / 1000) + "s").c_str());

        const Byte* image_ptr = camera.LockBuffer();
        array<Byte, kImageSize> image{};
        memcpy(image.data(), image_ptr, kImageSize);
        camera.UnlockBuffer();

      }
      if (time_img % 5000 == 0) {
        const Byte* image_ptr = camera.LockBuffer();
        array<Byte, kImageSize> image{};
        CopyByteArray(image_ptr, image);
        camera.UnlockBuffer();

        led3.SetEnable(true);

        led4.SetEnable(bt.SendBuffer(&st, 1));
        led2.SetEnable(bt.SendBuffer(image.data(), kImageSize));
        console.SetCursorRow(8);
        console.WriteString(("Packet " + to_string(i++)).c_str());

        led3.SetEnable(false);
      }
    }
  }
}

void Int16To2ByteArray(const uint16_t num, array<Byte, 2>& bytes) {
  bytes.at(0) = static_cast<Byte>(num >> 8);
  bytes.at(1) = static_cast<Byte>(num);
}

void ConsoleWriteString(LcdConsole* const console, const string& s) {
  console->WriteString(s.c_str());
}

void ConsoleClearRow(LcdConsole* const console, const uint8_t row) {
  console->SetCursorRow(row);
  ConsoleWriteString(console, "\n");
  console->SetCursorRow(row);
}

string to_string(int val) {
  array<char, kToStringBufferSize> str;
  sprintf(str.data(), "%d", val);
  return string(str.data());
}

string to_string(unsigned val) {
  array<char, kToStringBufferSize> str;
  sprintf(str.data(), "%u", val);
  return string(str.data());
}

string to_string(long val) {
  array<char, kToStringBufferSize> str;
  sprintf(str.data(), "%ld", val);
  return string(str.data());
}

string to_string(unsigned long val) {
  array<char, kToStringBufferSize> str;
  sprintf(str.data(), "%lu", val);
  return string(str.data());
}

string to_string(long long val) {
  array<char, kToStringBufferSize> str;
  sprintf(str.data(), "%lld", val);
  return string(str.data());
}

string to_string(unsigned long long val) {
  array<char, kToStringBufferSize> str;
  sprintf(str.data(), "%llu", val);
  return string(str.data());
}

string to_string(float val) {
  array<char, kToStringBufferSize> str;
  sprintf(str.data(), "%f", val);
  return string(str.data());
}

string to_string(double val) {
  array<char, kToStringBufferSize> str;
  sprintf(str.data(), "%f", val);
  return string(str.data());
}

string to_string(long double val) {
  array<char, kToStringBufferSize> str;
  sprintf(str.data(), "%Lf", val);
  return string(str.data());
}
}  // namespace util
