/*
 * main.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 */

#include "main.h"

using namespace libsc;

int ServoAngle = 900;

void JSListener(uint8_t id, Joystick::State which){
	switch (which){
	case Joystick::State::kLeft:
		ServoAngle++;
		break;
	case Joystick::State::kRight:
		ServoAngle--;
		break;
	}
}

int main() {
  System::Init();

  Led::Config ConfigLed;
  ConfigLed.id = 0;
  Led led1(ConfigLed);

  St7735r::Config ConfigLCD;
  ConfigLCD.is_revert=true;
  St7735r LCD(ConfigLCD);

  LcdConsole::Config ConfigLCDConsole;
  ConfigLCDConsole.lcd = &LCD;
  LcdConsole LCDCon(ConfigLCDConsole);

  FutabaS3010::Config ConfigServo;
  ConfigServo.id = 0;
  FutabaS3010 Servo(ConfigServo);
  Servo.SetDegree(ServoAngle);

  Joystick::Config ConfigJoystick;
  ConfigJoystick.id = 0;
  ConfigJoystick.is_active_low = true;
  ConfigJoystick.dispatcher = JSListener;
  Joystick joystick(ConfigJoystick);

  Timer::TimerInt time_img = 0;

  while (true) {
	  while (time_img != System::Time()){
		  time_img = System::Time();
		  if (time_img % 100 == 0){
			  led1.Switch();
			  Servo.SetDegree(ServoAngle);
			  LCDCon.WriteString(std::to_string(ServoAngle).c_str());

		  }
	  }
  }
  return 0;
}
