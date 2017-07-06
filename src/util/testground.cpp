/*
 * testground.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Implementation for testground.h
 *
 */

#include "util/testground.h"

#include <sstream>

#include "libsc/alternate_motor.h"
#include "libsc/dir_encoder.h"
#include "libsc/futaba_s3010.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libsc/lcd_console.h"
#include "libsc/led.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"
#include "libsc/k60/ov7725.h"
#include "libsc/joystick.h"

#include "bluetooth.h"
#include "car_manager.h"
#include "util/util.h"
#include "fc_yy_us_v4.h"

using namespace libsc;
using namespace libsc::k60;

namespace util {
namespace testground {

uint16_t config = 0;

TestVals vals = {0, 0.0};

TestVals t1 = {1, 1.0};
TestVals t2 = {2, 2.0};

void Inflate() {
  switch (config) {
    case 1:
      vals = t1;
      break;
    case 2:
      vals = t2;
      break;
    default:
      // retain values
      break;
  }
}

void main() {
  Inflate();

  Led::Config ConfigLed;
  ConfigLed.is_active_low = true;
  ConfigLed.id = 0;
  Led led0(ConfigLed);
  ConfigLed.id = 1;
  Led led1(ConfigLed);
  ConfigLed.id = 2;
  Led led2(ConfigLed);
  ConfigLed.id = 3;
  Led led3(ConfigLed);

  led0.SetEnable(true);
  led1.SetEnable(false);
  led2.SetEnable(false);
  led3.SetEnable(false);

  k60::Ov7725::Config cameraConfig;
  cameraConfig.id = 0;
  cameraConfig.w = 80;
  cameraConfig.h = 60;
  cameraConfig.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
  k60::Ov7725 camera(cameraConfig);

  led1.SetEnable(true);

  FutabaS3010::Config ConfigServo;
  ConfigServo.id = 0;
  auto servo = make_unique<FutabaS3010>(ConfigServo);

  DirEncoder::Config ConfigEncoder;
  ConfigEncoder.id = 0;
  auto encoder0 = make_unique<DirEncoder>(ConfigEncoder);
  ConfigEncoder.id = 1;
  auto encoder1 = make_unique<DirEncoder>(ConfigEncoder);

  AlternateMotor::Config ConfigMotor;
  ConfigMotor.id = 0;
  auto motor0 = make_unique<AlternateMotor>(ConfigMotor);
  ConfigMotor.id = 1;
  auto motor1 = make_unique<AlternateMotor>(ConfigMotor);

  led2.SetEnable(true);

  St7735r::Config lcdConfig;
  lcdConfig.is_revert = true;
  St7735r lcd(lcdConfig);

  LcdConsole::Config console_config;
  console_config.lcd = &lcd;
  LcdConsole console(console_config);

  JyMcuBt106::Config bt_config;
  bt_config.id = 0;
  bt_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
//  bt_config.rx_isr = &BluetoothListener;
  JyMcuBt106 bt(bt_config);

  FcYyUsV4 usir(libbase::k60::Pin::Name::kPtb0);

  Timer::TimerInt time_img = 0;

  ConsoleWriteString(&console, "i: " + to_string(vals.i));
  console.SetCursorRow(1);
  ConsoleWriteString(&console, "f: " + to_string(vals.f));

  while (true);
}

} //namesapce testground
} //namespace util
