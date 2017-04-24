/*
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
#include "libsc/dir_encoder.h"
#include "libsc/alternate_motor.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libsc/futaba_s3010.h"

#include "algorithm/bt-demo.h"
#include "algorithm/receiver.h"
#include "algorithm/king/main.h"
#include "algorithm/leslie/main.h"
#include "algorithm/peter/main.h"

#include "util/mpc_dual.h"
#include "util/util.h"

#include "math.h"

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
  kPeter,
  kReceiver,
  kBluetoothTest,
  kKingReceive
};

int main() {
  System::Init();

  BatteryMeter::Config ConfigBM;
  ConfigBM.voltage_ratio = 0.4;
  BatteryMeter bm(ConfigBM);
  //Battery Check
  while (bm.GetVoltage() <= 7.4);

  FutabaS3010::Config ConfigServo;
  ConfigServo.id = 0;
  unique_ptr<FutabaS3010> servo(new FutabaS3010(ConfigServo));

  DirEncoder::Config ConfigEncoder;
  ConfigEncoder.id = 0;
  DirEncoder encoder0(ConfigEncoder);
  ConfigEncoder.id = 1;
  DirEncoder encoder1(ConfigEncoder);

  AlternateMotor::Config ConfigMotor;
  ConfigMotor.id = 0;
  AlternateMotor motor0(ConfigMotor);
  ConfigMotor.id = 1;
  AlternateMotor motor1(ConfigMotor);

  util::MpcDual mpc(&motor0, &motor1, &encoder0, &encoder1);

  CarManager::Config ConfigMgr;
  ConfigMgr.servo = std::move(servo);
  ConfigMgr.car = CarManager::Car::kOld;
//  ConfigMgr.epc = std::move(mpc);
  CarManager::Init(std::move(ConfigMgr));

  CarManager::SetTargetAngle(CarManager::old_car.kCenter);

  JyMcuBt106::Config ConfigBT;
  ConfigBT.id = 0;
  ConfigBT.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
  JyMcuBt106 bt(ConfigBT);

  Timer::TimerInt time_img = 0;
  float t = 0;

  char speedChar[15] = {};

  while (1){
	  while (time_img != System::Time()){
		  time_img = System::Time();
		  if (time_img % 5 == 0){
//			  encoder.Update();
//			  int t = encoder.GetCount();
//			  mpc.DoCorrection();
		  }
		  if (time_img % 15 == 6){
			  CarManager::UpdateParameters();
			  int32_t s = mpc.GetCurrentSpeed(util::MpcDual::MotorSide::kRight);
			  int32_t de = 6000 * sin(t) * sin(t);
			  mpc.SetTargetSpeed(de,  true);
			  t += 0.02;
			  sprintf(speedChar, "%.1f,%d,%.1f=%.1f\n", 1.0, s, -6000.0 * sin(t) * sin(t), 1.0);
			  std::string speedStr = speedChar;
			  const Byte speedByte = 85;
			  bt.SendBuffer(&speedByte, 1);
			  bt.SendStr(speedStr);
		  }
	  }
  }

  // modify next line to switch between algorithms
  constexpr Algorithm a = Algorithm::kKing;

  // modify next line to enable/disable encoder
  constexpr bool has_encoder = false;

  // modify next line to change which car we're working with
  CarManager::Car c = CarManager::Car::kNew;

  CarManager::ServoBounds s = c == CarManager::Car::kOld ? CarManager::old_car : CarManager::new_car;
  switch (a) {
    case Algorithm::kKing:
      algorithm::king::main(has_encoder, s);
      break;
    case Algorithm::kLeslie:
      algorithm::leslie::main(has_encoder);
      break;
    case Algorithm::kPeter:
      algorithm::peter::main(has_encoder, s);
      break;
    case Algorithm::kReceiver:
      algorithm::receiver();
      break;
    case Algorithm::kBluetoothTest:
      algorithm::BluetoothDemo(has_encoder);
      break;
    case Algorithm::kKingReceive:
      algorithm::king::main_receive(has_encoder, s);
      break;
    default:
      // not handled
      break;
  }

  while (true) {
  }

  return 0;
}
