#include "algorithm/king/main.h"

#include "libsc/alternate_motor.h"
#include "libsc/dir_encoder.h"
#include "libsc/futaba_s3010.h"
#include "libsc/lcd_console.h"
#include "libsc/led.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"
#include "libsc/k60/ov7725.h"

#include "algorithm/king/Moving.h"

using libsc::AlternateMotor;
using libsc::DirEncoder;
using libsc::FutabaS3010;
using libsc::LcdConsole;
using libsc::Led;
using libsc::St7735r;
using libsc::System;
using libsc::Timer;
using libsc::k60::Ov7725;

namespace algorithm {
namespace king {
void main(bool has_encoder) {
  // initialize LEDs
  Led::Config led_config;
  led_config.is_active_low = true;
  led_config.id = 0;
  Led led1(led_config);  // main loop
  led_config.id = 1;
  Led led2(led_config);  // unused
  led_config.id = 2;
  Led led3(led_config);  // unused
  led_config.id = 3;
  Led led4(led_config);  // unused

  led1.SetEnable(false);
  led2.SetEnable(false);
  led3.SetEnable(false);
  led4.SetEnable(false);

  // initialize camera
  Ov7725::Config camera_config;
  camera_config.id = 0;
  camera_config.w = 80;
  camera_config.h = 60;
  Ov7725 camera(camera_config);

  // initialize LCD
  St7735r::Config lcd_config;
  St7735r lcd(lcd_config);

  // initialize LCD console
  LcdConsole::Config console_config;
  console_config.lcd = &lcd;
  LcdConsole console(console_config);

  // initialize Servo console
  FutabaS3010::Config servo_config;
  servo_config.id = 0;
  FutabaS3010 servo(servo_config);

  // initialize Encoder
  DirEncoder::Config encoder_config;
  encoder_config.id = 0;
  DirEncoder encoderA(encoder_config);
  encoder_config.id = 1;
  DirEncoder encoderB(encoder_config);

  AlternateMotor::Config motor_config;
  motor_config.multiplier = 100;
  motor_config.id = 0;
  AlternateMotor motor_left(motor_config);
  motor_config.id = 1;
  AlternateMotor motor_right(motor_config);

  motor_left.SetClockwise(true);
  motor_right.SetClockwise(false);

  motor_left.SetPower(200);
  motor_right.SetPower(200);

  camera.Start();
  while (!camera.IsAvailable()) {
  }

  //Initiate a car
  Moving car;
  Timer::TimerInt timeImg = System::Time();  // current execution time
  Timer::TimerInt startTime;  // starting time for read+copy buffer
  const Timer::TimerInt time_ms = 5;  // testing case in ms
  lcd.Clear();
  servo.SetDegree(ServoLeftBoundary);
  System::DelayMs(1000);
  servo.SetDegree(ServoRightBoundary);
  System::DelayMs(1000);
  servo.SetDegree(ServoStraightDegree);
  System::DelayMs(1000);

  // main loop
  while (true) {
    //car.Printing4Frames(lcd);
    if (timeImg != System::Time()) {
      timeImg = System::Time();
      // attempt to refresh the buffer at every 10th millisecond
      if ((timeImg % time_ms) == 0) {
        /*Motor Protection*/
        if (has_encoder) {
          encoderA.Update();
          encoderB.Update();
          if (encoderA.GetCount() == 0 || encoderB.GetCount() == 0) {
            motor_left.SetPower(0);
            motor_right.SetPower(0);
          }
        }
        startTime = System::Time();

        const Byte* camBuffer = camera.LockBuffer();
        led1.Switch();
        // unlock the buffer now that we have the data

        car.extract_cam(camBuffer);
        camera.UnlockBuffer();
        car.NormalMovingTestingVersion2(servo, lcd);
      }
    }
  }
}
}
}
