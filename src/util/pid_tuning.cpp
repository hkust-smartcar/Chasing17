/*
 * pid_tuning.h
 *
 * Author: Peter Tse (mcreng)
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * PID tuning with libutil PID classes and MK Wing's Processing tuning code.
 *
 */

#include "util/pid_tuning.h"

#include <sstream>

#include "libsc/dir_motor.h"
#include "libsc/dir_encoder.h"
#include "libsc/futaba_s3010.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libsc/lcd_console.h"
#include "libsc/led.h"
#include "libsc/system.h"
#include "libutil/incremental_pid_controller.h"

#include "bluetooth.h"
#include "util/util.h"

using namespace libsc;
using namespace libsc::k60;
using namespace libutil;

namespace util {
namespace pid_tuning {

std::string inputStr;
bool tune = false;
std::vector<double> constVector;
uint8_t now_angle = 0;

float Kp = 0, Ki = 0, Kd = 0;
float left_motor_target = 0, right_motor_target = 0;

/*
 * @brief: bluetooth listener for processing tuning
 * */
bool bluetoothListener(const Byte *data, const size_t size) {
	if (data[0] == 'P') {
			//space
	}

	if (data[0] == 't') {
		tune = 1;
		inputStr = "";
	}
	if (tune) {

		unsigned int i = 0;
		while (i<size) {
			if (data[i] != 't' && data[i] != '\n') {
				inputStr += (char)data[i];
			} else if (data[i] == '\n') {
				tune = 0;
				break;
			}
			i++;
		}
		if (!tune) {
			constVector.clear();
			char * pch;
			pch = strtok(&inputStr[0], ",");
			while (pch != NULL){
				double constant;
				std::stringstream(pch) >> constant;
				constVector.push_back(constant);
				pch = strtok (NULL, ",");
			}

			Kp = constVector[0];
			Ki = constVector[1];
			Kd = constVector[2];
			left_motor_target = constVector[3];
			right_motor_target = constVector[4];

		}
	}
	return 1;
}

void main() {
  Led::Config ConfigLed;
  ConfigLed.is_active_low = true;
  ConfigLed.id = 0;
  Led led0(ConfigLed);

  led0.SetEnable(true);

  FutabaS3010::Config ConfigServo;
  ConfigServo.id = 0;
  FutabaS3010 servo(ConfigServo);

  DirEncoder::Config ConfigEncoder;
  ConfigEncoder.id = 0;
  DirEncoder encoder0(ConfigEncoder);
  ConfigEncoder.id = 1;
  DirEncoder encoder1(ConfigEncoder);

  DirMotor::Config ConfigMotor;
  ConfigMotor.id = 0;
  DirMotor motor0(ConfigMotor);
  ConfigMotor.id = 1;
  DirMotor motor1(ConfigMotor);
  motor1.SetClockwise(false);

  JyMcuBt106::Config bt_config;
  bt_config.id = 0;
  bt_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
  bt_config.rx_isr = &bluetoothListener;
  JyMcuBt106 bt(bt_config);
  Timer::TimerInt time_img = 0;

  IncrementalPidController<float, float> pid_left(0,0,0,0);
  pid_left.SetOutputBound(-500, 500);
  IncrementalPidController<float, float> pid_right(0,0,0,0);
  pid_right.SetOutputBound(-500, 500);

  while (true){
	  while (System::Time() != time_img){
		  time_img = System::Time();
		  if (time_img % 10 == 0){
			  pid_left.SetKp(Kp);
			  pid_right.SetKp(Kp);
			  pid_left.SetKi(Ki);
			  pid_right.SetKi(Ki);
			  pid_left.SetKd(Kd);
			  pid_right.SetKd(Kd);
			  pid_left.SetSetpoint(left_motor_target);
			  pid_right.SetSetpoint(right_motor_target);

			  encoder0.Update();
			  encoder1.Update();

			  int curr_left = -encoder0.GetCount();
			  int curr_right = encoder1.GetCount();

			  char speedChar[100]{};
			  sprintf(speedChar, "%.1f,%d,%.2f,%d,%.2f\n", 1.0, curr_left, left_motor_target, curr_right, right_motor_target);
			  const Byte speedByte = 85;
			  bt.SendBuffer(&speedByte, 1);
			  bt.SendStr(speedChar);

			  motor0.AddPower(pid_left.Calc(curr_left));
			  motor1.AddPower(pid_right.Calc(curr_right));
		  }
		  if (time_img % 500 == 0) led0.Switch();
	  }
  }
}

} //namesapce pid_tuning
} //namespace util
