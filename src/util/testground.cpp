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

#include "img.h"

using namespace libsc;
using namespace libsc::k60;

namespace util {
namespace testground {

void main() {
  Led::Config ConfigLed;
  ConfigLed.id = 0;
  Led led0(ConfigLed);
  ConfigLed.id = 1;
  Led led1(ConfigLed);
  ConfigLed.id = 2;
  Led led2(ConfigLed);
  ConfigLed.id = 3;
  Led led3(ConfigLed);

//  k60::Ov7725::Config cameraConfig;
//  cameraConfig.id = 0;
//  cameraConfig.w = 80;
//  cameraConfig.h = 60;
//  cameraConfig.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
//  k60::Ov7725 camera(cameraConfig);

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

  auto mpc_dual = make_unique<MpcDual>(motor0.get(), motor1.get(), encoder0.get(), encoder1.get());

  k60::JyMcuBt106::Config ConfigBT;
  ConfigBT.id = 0;
  ConfigBT.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
  k60::JyMcuBt106 bt(ConfigBT);

  St7735r::Config lcdConfig;
  lcdConfig.is_revert = true;
  St7735r lcd(lcdConfig);

  LcdConsole::Config console_config;
  console_config.lcd = &lcd;
  LcdConsole console(console_config);

  Joystick::Config joystick_config;
  joystick_config.id = 0;
  joystick_config.is_active_low = true;
  Joystick joystick(joystick_config);

  CarManager::Config car_config;
  car_config.servo = std::move(servo);
  car_config.epc = std::move(mpc_dual);
  car_config.car = CarManager::Car::kCar1;
  CarManager::Init(std::move(car_config));

  CarManager::SetOverrideProtection(true);
  CarManager::SetTargetSpeed(6000);
  CarManager::SetTargetAngle(CarManager::kBoundsCar1.kCenter);

  FcYyUsV4 usir(libbase::k60::Pin::Name::kPtb0);

  Timer::TimerInt time_img = 0;

  char testChar[15] = {};

  while (true) {
    if (time_img != System::Time()) {
      time_img = System::Time();
      if (time_img % 100 == 0) led0.Switch();
		if (time_img % 15 == 0){
		  int32_t s = usir.GetDistance();
		  sprintf(testChar, "%.1f,%d,%.1f=%.1f\n", 1.0, s, 0.0, 1.0);
		  std::string testStr = testChar;
		  const Byte testByte = 85;
		  bt.SendBuffer(&testByte, 1);
		  bt.SendStr(testStr);
		}
    }
  }
}

} //namesapce testground
} //namespace util
