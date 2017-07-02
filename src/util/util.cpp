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
