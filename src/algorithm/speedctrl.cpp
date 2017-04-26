#include <cmath>

#include "libsc/dir_encoder.h"
#include "libsc/alternate_motor.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libsc/futaba_s3010.h"

#include "util/mpc.h"
#include "util/util.h"
#include "car_manager.h"
#include "algorithm/speedctrl.h"

using namespace libsc;

namespace algorithm {

void SpeedCtrl() {
  FutabaS3010::Config ConfigServo;
  ConfigServo.id = 0;
  std::unique_ptr<FutabaS3010> servo(new FutabaS3010(ConfigServo));

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

<<<<<<< HEAD
  util::Mpc mpc(&encoder0, &motor0, true);
||||||| merged common ancestors
  util::MpcDual mpc(&motor0, &motor1, &encoder0, &encoder1);

  CarManager::Config ConfigMgr;
  ConfigMgr.servo = std::move(servo);
  ConfigMgr.car = CarManager::Car::kOld;
//  ConfigMgr.epc = std::move(mpc);
  CarManager::Init(std::move(ConfigMgr));

  CarManager::SetTargetAngle(CarManager::old_car.kLeftBound);
=======
  St7735r::Config lcd_config;
  St7735r lcd(lcd_config);

  LcdConsole::Config console_config;
  console_config.lcd = &lcd;
  LcdConsole console(console_config);

  util::MpcDual mpc(&motor0, &motor1, &encoder0, &encoder1);
  util::MpcDualDebug mpc_d(&mpc);
  mpc.SetTargetSpeed(6000);

  CarManager::Config ConfigMgr;
  ConfigMgr.servo = std::move(servo);
  ConfigMgr.car = CarManager::Car::kNew;
//  ConfigMgr.epc = std::move(mpc);
  CarManager::Init(std::move(ConfigMgr));

  CarManager::SetTargetAngle(CarManager::new_car.kCenter);
>>>>>>> 55d7a494724c61af514cfaf6fa3034d418edbbbb

  k60::JyMcuBt106::Config ConfigBT;
  ConfigBT.id = 0;
  ConfigBT.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
  k60::JyMcuBt106 bt(ConfigBT);

  Timer::TimerInt time_img = 0;
  float t = 0;

  char speedChar[15] = {};

<<<<<<< HEAD
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
			  int32_t s = mpc.GetCurrentSpeed();
			  int32_t de = 6000;
			  mpc.SetTargetSpeed(de,  true);
			  t += 0.02;
			  sprintf(speedChar, "%.1f,%d,%.1f=%.1f\n", 1.0, s, 6000.0, 1.0);
			  std::string speedStr = speedChar;
			  const Byte speedByte = 85;
			  bt.SendBuffer(&speedByte, 1);
			  bt.SendStr(speedStr);
		  }
	  }
||||||| merged common ancestors
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
			  int32_t de = 6000;
			  mpc.SetTargetSpeed(de,  true);
			  t += 0.02;
			  sprintf(speedChar, "%.1f,%d,%.1f=%.1f\n", 1.0, s, -6000.0, 1.0);
			  std::string speedStr = speedChar;
			  const Byte speedByte = 85;
			  bt.SendBuffer(&speedByte, 1);
			  bt.SendStr(speedStr);
		  }
	  }
=======
  while (1) {
    while (time_img != System::Time()) {
      time_img = System::Time();
      if (time_img % 10 == 0) {
        CarManager::UpdateParameters();

        mpc.SetTargetSpeed(6000);

        mpc_d.OutputEncoderMotorValues(&console, util::MpcDual::MotorSide::kBoth);
//        mpc_d.OutputLastEncoderValues(&console, util::MpcDual::MotorSide::kBoth);
      }
    }
>>>>>>> 55d7a494724c61af514cfaf6fa3034d418edbbbb
  }
}
}
