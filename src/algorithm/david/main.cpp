//
// Created by david on 9/5/2017.
//

#include "algorithm/david/main.h"

#include <memory>

#include "libsc/alternate_motor.h"
#include "libsc/dir_encoder.h"
#include "libsc/futaba_s3010.h"
#include "libsc/lcd_console.h"
#include "libsc/led.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"

#include "car_manager.h"
#include "util/mpc_dual.h"
#include "util/util.h"

using libsc::AlternateMotor;
using libsc::DirEncoder;
using libsc::FutabaS3010;
using libsc::LcdConsole;
using libsc::Led;
using libsc::St7735r;
using libsc::System;
using std::unique_ptr;
using util::make_unique;
using util::MpcDual;
using util::MpcDualDebug;

namespace algorithm {
namespace david {

void main() {
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

  AlternateMotor::Config motor_config;
  motor_config.id = 0;
  auto motor1 = make_unique<AlternateMotor>(motor_config);
  motor_config.id = 1;
  auto motor2 = make_unique<AlternateMotor>(motor_config);

  DirEncoder::Config encoder_config;
  encoder_config.id = 0;
  auto encoder1 = make_unique<DirEncoder>(encoder_config);
  encoder_config.id = 1;
  auto encoder2 = make_unique<DirEncoder>(encoder_config);

  auto mpc_dual = make_unique<MpcDual>(motor1.get(), motor2.get(), encoder1.get(), encoder2.get());
  auto mpc_dual_debug = make_unique<MpcDualDebug>(mpc_dual.get());

  FutabaS3010::Config servo_config;
  servo_config.id = 0;
  auto servo = make_unique<FutabaS3010>(servo_config);

  St7735r::Config lcd_config;
  lcd_config.fps = 10;
  lcd_config.is_revert = true;
  auto lcd = make_unique<St7735r>(lcd_config);
  lcd->Clear();

  LcdConsole::Config console_config;
  console_config.lcd = lcd.get();
  auto console = make_unique<LcdConsole>(console_config);

  CarManager::Config car_config;
  car_config.servo = std::move(servo);
  car_config.epc = std::move(mpc_dual);
  car_config.car = CarManager::Car::kOld;
  CarManager::Init(std::move(car_config));

  CarManager::SetTargetAngle(CarManager::old_car.kCenter);

  auto time_img = System::Time();

  led4.SetEnable(false);

  while (true) {
    if (time_img != System::Time()) {
      time_img = System::Time();
      led1.SetEnable(time_img % 500 >= 250);
      if (time_img % 10 == 0) {
//        led2.SetEnable(true);
        CarManager::UpdateParameters();
//        led2.SetEnable(false);
//        led3.SetEnable(true);
        CarManager::SetTargetSpeed(CarManager::MotorSide::kBoth, 8000);
//        led3.SetEnable(false);

//        mpc_dual->SetTargetSpeed(8000);
//        mpc_dual_debug->OutputEncoderMotorValues(console.get(), MpcDual::MotorSide::kBoth);
      }
    }
  }
}

}  // namespace david
}  // namespace algorithm
