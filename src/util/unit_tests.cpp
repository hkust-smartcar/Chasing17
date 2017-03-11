/*
 * unit_tests.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 */

#include "util/unit_tests.h"

#include <memory>

#include "libsc/alternate_motor.h"
#include "libsc/dir_encoder.h"
#include "libsc/futaba_s3010.h"
#include "libsc/lcd_console.h"
#include "libsc/led.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"
#include "libsc/k60/ov7725.h"

#include "util/util.h"
#include "util/encoder_proportional_controller.h"

using libsc::AlternateMotor;
using libsc::DirEncoder;
using libsc::FutabaS3010;
using libsc::Lcd;
using libsc::LcdConsole;
using libsc::Led;
using libsc::St7735r;
using libsc::System;
using libsc::Timer;
using libsc::k60::Ov7725;
using std::unique_ptr;

namespace util {

void LedTest() {
  Led::Config config;
  config.is_active_low = true;
  config.id = 0;
  Led led1(config);
  config.id = 1;
  Led led2(config);
  config.id = 2;
  Led led3(config);
  config.id = 3;
  Led led4(config);
  led1.SetEnable(true);
  led2.SetEnable(false);
  led3.SetEnable(false);
  led4.SetEnable(true);

  while (true) {
    led1.Switch();
    led2.Switch();
    led3.Switch();
    led4.Switch();
    System::DelayMs(250);
  }
}

void LcdTest() {
  St7735r::Config config;
  config.fps = 10;
  config.is_revert = true;
  unique_ptr<St7735r> lcd(new St7735r(config));

  while (true) {
    lcd->SetRegion(Lcd::Rect(0, 0, 128, 80));
    lcd->FillColor(Lcd::kWhite);
    lcd->SetRegion(Lcd::Rect(0, 80, 128, 80));
    lcd->FillColor(Lcd::kBlack);
  }

  lcd.reset(nullptr);
}

void CameraTest() {
  // initialize camera
  Ov7725::Config camera_config;
  camera_config.id = 0;
  camera_config.w = 80;  // downscale the width to 80
  camera_config.h = 60;  // downscale the height to 60
  unique_ptr<Ov7725> camera(new Ov7725(camera_config));
  camera->Start();
  constexpr Uint kBufferSize = 80 * 60 / 8;
  if (kBufferSize != camera->GetBufferSize()) {
    return;
  }

  // initialize LCD
  St7735r::Config lcd_config;
  lcd_config.fps = 50;
  lcd_config.is_revert = true;
  unique_ptr<St7735r> lcd(new St7735r(lcd_config));

  while (!camera->IsAvailable()) {}

  while (true) {
    const Byte *pBuffer = camera->LockBuffer();
    std::array<Byte, kBufferSize> buffer_arr{};
    CopyByteArray(*pBuffer, &buffer_arr);

    camera->UnlockBuffer();
    lcd->SetRegion(Lcd::Rect(0, 0, 80, 60));
    lcd->FillBits(Lcd::kBlack, Lcd::kWhite, buffer_arr.data(), kBufferSize * 8);
  }

  camera->Stop();
  camera.reset(nullptr);
  lcd.reset(nullptr);
}

void ServoTest() {
  FutabaS3010::Config config;
  config.id = 0;
  FutabaS3010 servo(config);

  while (true) {
    servo.SetDegree(450);
    System::DelayMs(2000);
    servo.SetDegree(1350);
    System::DelayMs(2000);
  }
}

void AltMotorTest() {
  AlternateMotor::Config config;
  config.multiplier = 100;
  config.id = 0;
  AlternateMotor motor_left(config);
  config.id = 1;
  AlternateMotor motor_right(config);

  motor_left.SetClockwise(true);
  motor_right.SetClockwise(true);

  while (true) {
    motor_left.SetPower(100);
    motor_right.SetPower(100);
  }
}

void DirEncoderTest() {
  AlternateMotor::Config motor_config;
  motor_config.multiplier = 100;
  motor_config.id = 0;
  AlternateMotor motor_left(motor_config);
  motor_config.id = 1;
  AlternateMotor motor_right(motor_config);

  DirEncoder::Config encoder_config;
  encoder_config.id = 0;
  DirEncoder encoder_left(encoder_config);
  encoder_config.id = 1;
  DirEncoder encoder_right(encoder_config);

  motor_left.SetClockwise(true);
  motor_right.SetClockwise(true);

  St7735r::Config lcd_config;
  lcd_config.fps = 10;
  lcd_config.is_revert = true;
  St7735r lcd(lcd_config);

  LcdConsole::Config console_config;
  console_config.lcd = &lcd;
  LcdConsole console(console_config);

  while (true) {
    motor_left.SetPower(100);
    motor_right.SetPower(100);

    encoder_left.Update();
    encoder_right.Update();

    std::string s = std::to_string(encoder_left.GetCount()) + " " + std::to_string(encoder_right.GetCount()) + "\n";
    console.WriteString(s.c_str());

    System::DelayMs(100);
  }
}

void EncoderPControllerTest() {
  Led::Config config;
  config.is_active_low = true;
  config.id = 0;
  Led led1(config);
  config.id = 1;
  Led led2(config);
  config.id = 2;
  Led led3(config);
  config.id = 3;
  Led led4(config);
  led1.SetEnable(false);
  led2.SetEnable(false);
  led3.SetEnable(false);
  led4.SetEnable(true);

  AlternateMotor::Config motor_config;
  motor_config.id = 0;
  unique_ptr<AlternateMotor> motor1(new AlternateMotor(motor_config));
  motor_config.id = 1;
  unique_ptr<AlternateMotor> motor2(new AlternateMotor(motor_config));

  FutabaS3010::Config servo_config;
  servo_config.id = 0;
  unique_ptr<FutabaS3010> servo(new FutabaS3010(servo_config));

  DirEncoder::Config encoder_config;
  encoder_config.id = 0;
  unique_ptr<DirEncoder> encoder1(new DirEncoder(encoder_config));
  encoder_config.id = 1;
  unique_ptr<DirEncoder> encoder2(new DirEncoder(encoder_config));

  St7735r::Config lcd_config;
  lcd_config.fps = 10;
  lcd_config.is_revert = true;
  unique_ptr<St7735r> lcd(new St7735r(lcd_config));
  lcd->Clear();

  LcdConsole::Config console_config;
  console_config.lcd = lcd.get();
  unique_ptr<LcdConsole> console(new LcdConsole(console_config));

  unique_ptr<EncoderPController> epc1(new EncoderPController(encoder1.get(), motor1.get()));
  unique_ptr<EncoderPController> epc2(new EncoderPController(encoder2.get(), motor2.get()));
  unique_ptr<EncoderPControllerDebug> epc1_d(new EncoderPControllerDebug(epc1.get()));
  unique_ptr<EncoderPControllerDebug> epc2_d(new EncoderPControllerDebug(epc2.get()));

  led4.SetEnable(false);

  while (true) {
    epc1->SetTargetSpeed(4000);
    epc2->SetTargetSpeed(-4000);
    servo->SetDegree(665);

    epc1_d->OutputEncoderMotorValues(console.get());
    epc2_d->OutputEncoderMotorValues(console.get());

    System::DelayMs(100);
  }
}
}  // namespace util
