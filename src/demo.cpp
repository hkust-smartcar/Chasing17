/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 */

#include <string>

#include "libbase/k60/adc.h"
#include "libbase/k60/dac.h"
#include "libbase/k60/dma.h"
#include "libbase/k60/gpio.h"
#include "libbase/k60/ftm_pwm.h"
#include "libbase/k60/soft_i2c_master.h"
#include "libbase/k60/spi_master.h"
#include "libbase/k60/uart.h"
#include "libsc/alternate_motor.h"
#include "libsc/button.h"
#include "libsc/dir_motor.h"
#include "libsc/futaba_s3010.h"
#include "libsc/lcd_console.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/led.h"
#include "libsc/simple_buzzer.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"
#include "libsc/tower_pro_mg995.h"
#include "libsc/trs_d05.h"
#include "libsc/k60/ftdi_ft232r.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libsc/k60/ov7725.h"
#include "libutil/kalman_filter.h"
#include "libutil/positional_pid_controller.h"
#include "libutil/remote_var_manager.h"
#include "libutil/string.h"

using libbase::k60::FtmPwm;
using libbase::k60::Gpi;
using libbase::k60::Gpo;
using libbase::k60::Pin;
using libbase::k60::Pwm;
using libbase::k60::SoftI2cMaster;
using libbase::k60::SpiMaster;
using libsc::Button;
using libsc::DirMotor;
using libsc::k60::FtdiFt232r;
using libsc::Lcd;
using libsc::LcdConsole;
using libsc::LcdTypewriter;
using libsc::Led;
using libsc::SimpleBuzzer;
using libsc::St7735r;
using libsc::System;
using libsc::TowerProMg995;

// libbase/k60/gpio.h
void mcu_gpio() {
  Gpi::Config iConfig;
  iConfig.pin = Pin::Name::kPta0;
  Gpi gpi(iConfig);

  if (gpi.Get()) {
    // pin at high state
  } else {
    // pin at low state
  }

  Gpo::Config oConfig;
  oConfig.pin = Pin::Name::kPta0;
  oConfig.is_high = false;  // starts with low
  Gpo gpo(oConfig);

  gpo.Set(true);   // set to high state
  gpo.Set(false);  // set to low state
}

// libbase/k60/ftm_pwm.h
void mcu_pwm() {
  FtmPwm::Config config;
  config.pin = Pin::Name::kPta3;
  config.period = 100000;  // 10KHz
  config.pos_width = 0;
  config.precision = Pwm::Config::Precision::kNs;
  FtmPwm pwm(config);

  pwm.SetPosWidth(10000);  // 10% duty cycle
}

// libbase/k60/soft_i2c_master.h
void mcu_12c() {
  SoftI2cMaster::Config config;
  config.scl_pin = Pin::Name::kPtb0;
  config.sda_pin = Pin::Name::kPta1;
  SoftI2cMaster i2c(config);

  // i2c.SendByte(slave_addr, reg_addr, 0xFF);
  Byte data;
  // i2c.GetByte(slave_addr, reg_addr, &data);
}

// libbase/k60/spi_master.h
void mcu_spi() {
  SpiMaster::Config config;
  config.sout_pin = Pin::Name::kPta16;
  config.sck_pin = Pin::Name::kPta15;
  config.baud_rate_khz = 15000;
  config.frame_size = 8;
  config.is_sck_idle_low = true;
  config.is_sck_capture_first = true;
  config.is_msb_firt = true;
  config.slaves[0].cs_pin = Pin::Name::kPta14;
  config.slaves[1].cs_pin = Pin::Name::kPta13;
  SpiMaster spi(config);

  uint8_t receive = static_cast<uint8_t>(spi.ExchangeData(1, 0xFF));
}

// libutil/string.h
void util_string() {
  int value = 123;
  std::string str = libutil::String::Format("value: %d", value);
}

// libutil/positional_pid_controller.h
void util_pos_pid() {
  libutil::PositionalPidController<float, int16_t> pid(0, 0.25f, 0.0f, 0.0001f);
  // float input = sensor.GetValue();
  // int16_t output = pid.Calc(input);
}

// libutil/incremental_pid_controller.h
void util_inc_pid() {
  // libutil::IncrementalPidController<float, int16_t> pid(0, 0.25f, 0.0f, 0.0001f);
  // float input = sensor.GetValue();
  // int16_t output = pid.Calc(input);
}

// libutil/remote_var_manager.h
void util_rvm() {
  using libutil::RemoteVarManager;

  // RemoteVarManager rvm(&uart, 2);
  // RemoteVarManager::Var *sp = rvm.Register("sp", RemoteVarManager::Var::Type::kReal);
  // RemoteVarManager::Var *hehe = rvm.Register(“hehe”, RemoteVarManager::Var::Type::kInt);
  // rvm.Start();
  // float sp_val = sp->GetReal();
  // int hehe_val = hehe->GetInt();
  // Change value in PC
  // sp_val = sp->GetReal();
}

// libsc/button.h
void sc_button() {
  Button::Config config;
  config.id = 0;
  config.is_active_low = true;
  Button btn(config);

  while (true) {
    if (btn.IsDown()) {
      // do something
    }
  }
}

// libsc/led.h
void sc_led() {
  Led::Config config;
  config.id = 0;
  Led led(config);

  led.SetEnable(true);
  while (true) {
    led.Switch();
    System::DelayMs(250);
  }
}

// libsc/simple_buzzer.h
void sc_buzzer() {
  SimpleBuzzer::Config config;
  config.id = 0;
  SimpleBuzzer buzzer(config);

  buzzer.SetBeep(true);
}

// libsc/alternate_motor.h
// libsc/dir_motor.h
void sc_motor() {
  // DirMotor can be substituted with AlternateMotor
  DirMotor::Config config;
  config.id = 0;
  DirMotor motor(config);

  motor.SetPower(500);
}

// libsc/futaba_s3010.h
// libsc/tower_pro_mg995.h
// libsc/trs_d05.h
void sc_servo() {
  TowerProMg995::Config config;
  config.id = 0;
  TowerProMg995 servo(config);

  servo.SetDegree(900);
}

// libsc/st7735r.h
// libsc/lcd_typewriter.h
void sc_lcd() {
  St7735r::Config lcdConfig;
  lcdConfig.is_revert = false;
  lcdConfig.is_bgr = false;
  St7735r lcd(lcdConfig);

  lcd.Clear(0);
  lcd.SetRegion(Lcd::Rect(5, 10, 6, 6));
  lcd.FillColor(libutil::GetRgb565(0, 0, 255));

  LcdTypewriter::Config lcdTyperConfig;
  lcdTyperConfig.lcd = &lcd;
  LcdTypewriter writer(lcdTyperConfig);

  lcd.SetRegion(Lcd::Rect(0, 0, 128, 160));
  writer.WriteString("hello world");
  lcd.SetRegion(Lcd::Rect(100, 0, 50, 160));
  writer.WriteString("hello world");

  LcdConsole::Config lcdConsoleConfig;
  lcdConsoleConfig.lcd = &lcd;
  lcdConsoleConfig.region = Lcd::Rect(0, 0, 128, 80);
  LcdConsole console(lcdConsoleConfig);

  console.WriteString("SmartCar 2015\n");
  console.WriteString("hello world\n");
}

// libsc/k60/ftdi_ft232r.h
// libsc/k60/jy_mcu_bt_106.h
void sc_uart() {
  FtdiFt232r::Config config;
  config.id = 0;
  config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
  FtdiFt232r com(config);

  com.SendStr("hello world\n");
  std::string str = "C++ string\n";
  com.SendStr(str);
  com.SendStrLiteral("string literal\n");
}
