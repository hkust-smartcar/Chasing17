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

struct {
  std::string input = "";
  bool tune = false;
  std::vector<double> vars{};
} tuning_vars;

bool BluetoothListener(const Byte* data, const std::size_t size) {
  if (data[0] == 't') {
    tuning_vars.tune = true;
    tuning_vars.input.clear();
  }

  if (tuning_vars.tune) {
    unsigned int i = 0;
    while (i < size) {
      if (data[i] != 't' && data[i] != '\n') {
        tuning_vars.input.push_back(data[i]);
      } else if (data[i] == '\n') {
        tuning_vars.tune = false;
        break;
      }
      i++;
    }

    if (!tuning_vars.tune) {
      tuning_vars.vars.clear();
      char* pch = strtok(&tuning_vars.input[0], ",");
      while (pch != nullptr) {
        double constant;
        std::stringstream(pch) >> constant;
        tuning_vars.vars.push_back(constant);
        pch = strtok(nullptr, ",");
      }

      // variable assignments here
      CarManager::kMotorPidCar1.kP[0] = tuning_vars.vars.at(0);
      CarManager::kMotorPidCar1.kI[0] = tuning_vars.vars.at(1);
      CarManager::kMotorPidCar1.kD[0] = tuning_vars.vars.at(2);

      CarManager::kMotorPidCar1.kP[1] = tuning_vars.vars.at(3);
      CarManager::kMotorPidCar1.kI[1] = tuning_vars.vars.at(4);
      CarManager::kMotorPidCar1.kD[1] = tuning_vars.vars.at(5);
    }
  }

  return true;
}

void main() {
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

  auto mpc_dual = make_unique<MpcDual>(motor0.get(), motor1.get(), encoder0.get(), encoder1.get());
  auto mpc_dual_debug = make_unique<MpcDualDebug>(mpc_dual.get());

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
  bt_config.rx_isr = &BluetoothListener;
  JyMcuBt106 bt(bt_config);

  led3.SetEnable(true);

  CarManager::Config car_config;
  car_config.servo = std::move(servo);
//  car_config.epc = std::move(mpc_dual);
  car_config.car = CarManager::Car::kCar2;
  CarManager::Init(std::move(car_config));

//  CarManager::SetOverrideProtection(true);
  mpc_dual->SetForceOverride(true);
//  CarManager::SetTargetSpeed(10000);
  CarManager::SetTargetAngle(0);

  led0.SetEnable(false);
  led1.SetEnable(false);
  led2.SetEnable(false);
  led3.SetEnable(false);
  System::DelayMs(250);
  led0.SetEnable(true);
  led1.SetEnable(true);
  led2.SetEnable(true);
  led3.SetEnable(true);
  System::DelayMs(250);
  led0.SetEnable(false);
  led1.SetEnable(false);
  led2.SetEnable(false);
  led3.SetEnable(false);
  System::DelayMs(250);
  led0.SetEnable(true);
  led1.SetEnable(true);
  led2.SetEnable(true);
  led3.SetEnable(true);
  System::DelayMs(250);
  led0.SetEnable(false);
  led1.SetEnable(false);
  led2.SetEnable(false);
  led3.SetEnable(false);

  Timer::TimerInt time_img = 0;
  char speedStr[32];

  while (true) {
    if (time_img != System::Time()) {
      time_img = System::Time();
      if (time_img % 250 == 0) led0.Switch();
      if (time_img % 10 == 0) {
//        CarManager::SetTargetSpeed(10000);
        mpc_dual->SetTargetSpeed(10000);
        CarManager::UpdateParameters();

//        mpc_dual_debug->OutputPidValues(&console);
//        mpc_dual_debug->OutputEncoderMotorValues(&console, MpcDual::MotorSide::kBoth);
        mpc_dual_debug->OutputLastEncoderValues(&console, MpcDual::MotorSide::kBoth);

        sprintf(speedStr, "%.1f,%ld,%.1f\n", 1.0, mpc_dual->GetCurrentSpeed(MpcDual::MotorSide::kLeft), 10000.0);
        std::string spdStr = std::string(speedStr);
        const Byte speedByte = 85;
        bt.SendBuffer(&speedByte, 1);
        bt.SendStr(speedStr);
      }
    }
  }
}

} //namesapce testground
} //namespace util
