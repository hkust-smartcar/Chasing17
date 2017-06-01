/*
 * testground.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Peter Tse (mcreng)
 *
 * Testground for whatever you want.
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

#include "debug_console.h"
#include "img.h"

namespace util{
namespace testground{

void main(){
	System::Init();

	Led::Config ConfigLed;
	ConfigLed.id = 0;
	Led led0(ConfigLed);
	ConfigLed.id = 1;
	Led led1(ConfigLed);
	ConfigLed.id = 2;
	Led led2(ConfigLed);
	ConfigLed.id = 3;
	Led led3(ConfigLed);

	k60::Ov7725::Config cameraConfig;
	cameraConfig.id = 0;
	cameraConfig.w = 80;
	cameraConfig.h = 60;
	cameraConfig.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
	k60::Ov7725 camera(cameraConfig);

	FutabaS3010::Config ConfigServo;
	ConfigServo.id = 0;
	FutabaS3010 servo(ConfigServo);

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

	k60::JyMcuBt106::Config ConfigBT;
	ConfigBT.id = 0;
	ConfigBT.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	k60::JyMcuBt106 bt(ConfigBT);

	St7735r::Config lcdConfig;
	lcdConfig.is_revert = true;
	St7735r lcd(lcdConfig);

	LcdTypewriter::Config writerConfig;
	writerConfig.lcd = &lcd;
	LcdTypewriter writer(writerConfig);

	Joystick::Config joystick_config;
	joystick_config.id = 0;
	joystick_config.is_active_low = true;
	Joystick joystick(joystick_config);

	DebugConsole console(&joystick, &lcd, &writer);

	Timer::TimerInt time_img = 0;

	//testground starts here
	servo.SetDegree(755);

}

} //namesapce testground
} //namespace util
