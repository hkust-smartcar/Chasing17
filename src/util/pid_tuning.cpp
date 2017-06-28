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

#include "libsc/alternate_motor.h"
#include "libsc/dir_encoder.h"
#include "libsc/futaba_s3010.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libsc/lcd_console.h"
#include "libsc/led.h"
#include "libsc/system.h"
#include "libsc/k60/ov7725.h"

#include "bluetooth.h"
#include "util/util.h"

using namespace libsc;
using namespace libsc::k60;

namespace util {
namespace pid_tuning {

std::string inputStr;
bool tune = false;
std::vector<double> constVector;
uint8_t now_angle = 0;

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

//			Kp = constVector[0];
//			Ki = constVector[1];
//			Kd = constVector[2];
//			target_enc_val_left = constVector[3];
//			target_enc_val_right = constVector[4];
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

  AlternateMotor::Config ConfigMotor;
  ConfigMotor.id = 0;
  AlternateMotor motor0(ConfigMotor);
  ConfigMotor.id = 1;
  AlternateMotor motor1(ConfigMotor);

  JyMcuBt106::Config bt_config;
  bt_config.id = 0;
  bt_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
  bt_config.rx_isr = &bluetoothListener;
  JyMcuBt106 bt(bt_config);
}

} //namesapce testground
} //namespace util
