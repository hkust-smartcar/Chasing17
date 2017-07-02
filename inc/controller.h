/*
 * controller.h
 *
 *  Created on: Jul 2, 2017
 *      Author: dipsy
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "libsc/dir_motor.h"
#include "libsc/dir_encoder.h"
#include "libsc/futaba_s3010.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libsc/lcd_console.h"
#include "libsc/st7735r.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/led.h"
#include "libsc/system.h"
#include "libutil/incremental_pid_controller.h"
#include "libsc/joystick.h"
#include "libutil/positional_pid_controller.h"
#include "libbase/k60/pit.h"

#include "bluetooth.h"
#include "util/util.h"

using namespace libsc;
using namespace libsc::k60;
using namespace libutil;
using namespace libbase::k60;


class Controller{

public :

	//car = 1 is car1, else is car 2
	Controller(int car, DirMotor* pMotor0,DirMotor* pMotor1,DirEncoder* pEncoder0,DirEncoder* pEncoder1,FutabaS3010* pServo);

	/*
	 * @brief set motor target
	 * target: target encoder value for both encoder in each duty cycle
	 */
	void SetMotorTarget(float target);

	/*
	 * @brief get motor power, negative is going backward
	 * id: motor id, which is 0 or 1
	 */
	int GetMotorPower(int id);

	/*
	 * @brief : replace all servo->SetDegree with this, this will do software differential automatically
	 */
	void SetServoDegree(uint16_t Target);

	/*
	 * delta = (target - servo mid)/10
	 */
	void SetServoDelta(float delta);

	/*
	 * @brief sync the speed pid and differential
	 * this should be called once and only once each 10ms
	 */
	void Sync(Pit* pit);

	void debug(Lcd* pLcd, LcdTypewriter* pWriter);

	int32_t GetEncoder(){
		return (encoder_val0+encoder_val1)/2;
	}

	void ClearEncoder(){
		encoder_val0=0;
		encoder_val1=0;
	}

private:

	int m_car = 0;

	uint8_t now_angle = 0;
	DirMotor* pMotor0 = nullptr;
	DirMotor* pMotor1 = nullptr;
	DirEncoder* pEncoder0 = nullptr;
	DirEncoder* pEncoder1 = nullptr;
	FutabaS3010* pServo = nullptr;
	IncrementalPidController<float, float> *pLeft = nullptr, *pRight = nullptr;
	//JyMcuBt106* pBt = nullptr;
	int servo_angle=845;
	int start_time=0, resume_time=0;

	float m_delta = 0;//servo angle param calculation
	int m_target = 0;//normal speed target

	int servo_left, servo_mid,servo_right;//servo bounds

	int32_t encoder_val0 = 0, encoder_val1 = 0;

	float Kp = 2.5, Ki = 0.02, Kd = 0;
	int left_motor_target = 0, right_motor_target = 0;


	/*
	 * @brief set motor power
	 * power: direction and magnitude; negative is going backward
	 * id: motor id, which is 0 or 1
	 *
	 * PS(if you want to set the speed of whole, you should use SetMotorTarget Instead)
	 */
	void SetMotorPower(int power,int id);

	/*
	 * calculate differential
	 */
	void CalculateSpeed();
};


#endif /* INC_CONTROLLER_H_ */
