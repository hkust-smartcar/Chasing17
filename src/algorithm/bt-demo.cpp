//
// Created by david on 20/4/2017.
//

#include "algorithm/bt-demo.h"

#include "libsc/alternate_motor.h"
#include "libsc/dir_encoder.h"
#include "libsc/futaba_s3010.h"
#include "libsc/led.h"
#include "bluetooth.h"

using namespace libsc;
using namespace std;

namespace algorithm {
void BluetoothDemo(bool has_encoder) {
  Led::Config config_led;
  config_led.is_active_low = true;
  config_led.id = 0;
  Led led1(config_led);
  config_led.id = 1;
  Led led2(config_led);
  config_led.id = 2;
  Led led3(config_led);
  config_led.id = 3;
  Led led4(config_led);

  AlternateMotor::Config config_motor;
  config_motor.id = 0;
  AlternateMotor motor_left(config_motor);
  config_motor.id = 1;
  AlternateMotor motor_right(config_motor);

  FutabaS3010::Config config_servo;
  config_servo.id = 0;
  FutabaS3010 servo(config_servo);

  DirEncoder::Config config_encoder;
  config_encoder.id = 0;
  DirEncoder encoder_left(config_encoder);
  config_encoder.id = 1;
  DirEncoder encoder_right(config_encoder);

  k60::JyMcuBt106::Config config_bluetooth;
  config_bluetooth.id = 0;
  config_bluetooth.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
  BTComm bluetooth(config_bluetooth);
//  k60::JyMcuBt106 bt(config_bluetooth);

  motor_left.SetPower(200);
  motor_left.SetClockwise(false);
  motor_right.SetPower(200);
  motor_right.SetClockwise(false);

  servo.SetDegree(550);

  Timer::TimerInt time{System::Time()};

  while (true) {
    if (time != System::Time()) {
      time = System::Time();
      if (time % 100 == 0) {
        encoder_left.Update();
        encoder_right.Update();
        bluetooth.sendSpeed(abs(encoder_left.GetCount()) / 10);
        bluetooth.sendSpeed(abs(encoder_right.GetCount()) / 10);
        bluetooth.sendSlopeDeg((servo.GetDegree() - 900) / 10);
//        bluetooth.sendByte(100);
//        bt.SendStr("Hello World");
        led1.Switch();
      }
    }
  }
}
}
