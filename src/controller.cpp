/*
 * controller.cpp
 *
 *  Created on: Jul 2, 2017
 *      Author: dipsy
 */

#include "controller.h"

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

Controller::Controller(int car, DirMotor* pMotor0,DirMotor* pMotor1,DirEncoder* pEncoder0,DirEncoder* pEncoder1,FutabaS3010* pServo):
	m_car(car),
	pMotor0(pMotor0),
	pMotor1(pMotor1),
	pEncoder0(pEncoder0),
	pEncoder1(pEncoder1),
	pServo(pServo){

		IncrementalPidController<float, float> pid_left(0,Kp,Ki,Kd);
		pid_left.SetOutputBound(-1000, 1000);
		IncrementalPidController<float, float> pid_right(0,Kp,Ki,Kd);
		pid_right.SetOutputBound(-1000, 1000);

		pLeft = &pid_left;
		pRight = &pid_right;

		start_time = System::Time();

		if(m_car == 1){
			servo_left = 1040;
			servo_mid = 755;
			servo_right = 470;
		}
		else{
			servo_left = 1145;
			servo_mid = 845;
			servo_right = 545;
		}

//		SetMotorTarget(0);
}

void Controller::SetMotorTarget(float target){
	m_target = target;
//	left_motor_target = target;
//	right_motor_target = target;
	CalculateSpeed();
}

void Controller::SetMotorPower(int power,int id){
	int pw = (power>0?power:-power);	//abs power
	bool direction = (power>0);			//positive is true
	pw = libutil::Clamp<int>(0,pw,1000);
	switch(id){
	case 0:
		pMotor0->SetPower(pw);
		pMotor0->SetClockwise(direction);	//true is forward
		break;
	case 1:
		pMotor1->SetPower(pw);
		pMotor1->SetClockwise(!direction);	//false is forward
		break;
	}
}

int Controller::GetMotorPower(int id){
	int power;
	switch(id){
	case 0:
		power = pMotor0->GetPower();
		return (pMotor0->IsClockwise() ? power : -power);//true is forward
		break;
	case 1:
		power = pMotor1->GetPower();
		return (pMotor1->IsClockwise() ? -power : power);//false is forward
		break;
	}
	return 0;
}

void Controller::SetServoDegree(uint16_t target){
	pServo->SetDegree(util::clamp<uint16_t>(servo_right,target,servo_left));
	m_delta = (target-servo_mid)/10.0f;
	CalculateSpeed();
}

void Controller::SetServoDelta(float delta){
	m_delta = delta;
	int target = delta*10+servo_mid;
	pServo->SetDegree(util::clamp<uint16_t>(servo_right,target,servo_left));
	CalculateSpeed();
}

void Controller::Sync(Pit*){

	int time_img=System::Time();

	pEncoder0->Update();
	pEncoder1->Update();

	int curr_left = pEncoder0->GetCount();
	int curr_right = -pEncoder1->GetCount();

//	if(time_img-start_time<10000){
//		Ki = (10000-0.98*(time_img-start_time))/10000.0;
//	}
//	else{
//		Ki = 0.02;
//	}
//	pLeft->SetKp(Kp);
//	pRight->SetKp(Kp);
//	pLeft->SetKi(Ki);
//	pRight->SetKi(Ki);
//	pLeft->SetKd(Kd);
//	pRight->SetKd(Kd);
	if(m_car==1){
		pLeft->SetSetpoint(m_target * (1.00716 - 0.00776897*m_delta));
		pRight->SetSetpoint(m_target * (1.00716 + 0.00776897*m_delta));
	}
	else{
		pLeft->SetSetpoint(m_target * (0.996595 - 0.00862696*m_delta));
		pRight->SetSetpoint(m_target * (0.996595 + 0.00862696*m_delta));
	}

//	pLeft->SetSetpoint(m_target);
//	pRight->SetSetpoint(m_target);

//
////	char speedChar[100]{};
////	sprintf(speedChar, "%.1f,%d,%.2f,%d,%.2f\n", 1.0, curr_left, left_motor_target, curr_right, right_motor_target);
////	const Byte speedByte = 85;
////	pBt->SendBuffer(&speedByte, 1);
////	pBt->SendStr(speedChar);
	SetMotorPower(GetMotorPower(0)+pLeft->Calc(curr_left),0);
	SetMotorPower(GetMotorPower(1)+pRight->Calc(curr_right),1);

	encoder_val0+=curr_left;
	encoder_val1+=curr_right;

}

void Controller::debug(Lcd* pLcd, LcdTypewriter* pWriter){
	char buff[100];
	sprintf(buff,"%f\n%f",left_motor_target,right_motor_target);
//	sprintf(buff,"%d\n%d",GetMotorPower(0),GetMotorPower(1));
//				  sprintf(buff,"%d\n%d",GetMotorPower(0),GetMotorPower(1));
//				  sprintf(buff,"%.3lf \n%.3lf \n%.3lf \n%.3lf \n%.3lf \n%.3lf ",pid_left.GetKp(),pid_left.GetKi(),pid_left.GetKd(),pid_right.GetKp(),pid_right.GetKi(),pid_right.GetKd());
	pLcd->SetRegion(Lcd::Rect(0,0,128,160));
	pWriter->WriteBuffer(buff,100);
}

void Controller::CalculateSpeed(){

	left_motor_target = m_target;
	right_motor_target = m_target;
	return;

	if(m_car==1){
		left_motor_target = int(m_target * (1.00716 - 0.00776897*m_delta));
		right_motor_target = int(m_target * (1.00716 + 0.00776897*m_delta));
	}else{
		left_motor_target = int(m_target * (0.996595 - 0.00862696*m_delta));
		right_motor_target = int(m_target * (0.996595 + 0.00862696*m_delta));
	}
}
