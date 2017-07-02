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

#include "controller.h"

using namespace libsc;
using namespace libsc::k60;
using namespace libutil;
using namespace libbase::k60;

namespace util {
namespace pid_tuning {

std::string inputStr;
bool tune = false;
std::vector<double> constVector;
uint8_t now_angle = 0;
DirMotor* pMotor0 = nullptr;
DirMotor* pMotor1 = nullptr;
DirEncoder* pEncoder0 = nullptr;
DirEncoder* pEncoder1 = nullptr;
FutabaS3010* pServo = nullptr;
IncrementalPidController<float, float> *pLeft = nullptr, *pRight = nullptr;
JyMcuBt106* pBt = nullptr;
int servo_angle=845;
int start_time=0, resume_time=0;


float Kp = 2.5, Ki = 0.02, Kd = 0;
float left_motor_target = 000, right_motor_target = 000;

/*
 * @brief set motor power
 * power: direction and magnitude; negative is going backward
 * id: motor id, which is 0 or 1
 */
void SetMotorPower(int power,int id){
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

/*
 * @brief get motor power, negative is going backward
 * id: motor id, which is 0 or 1
 */
int GetMotorPower(int id){
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

void SyncMotor(Pit* pit){

	int time_img=System::Time();

	pEncoder0->Update();
	pEncoder1->Update();

	int curr_left = pEncoder0->GetCount();
	int curr_right = -pEncoder1->GetCount();

	if(time_img-start_time<10000){
		Ki = (10000-0.98*(time_img-start_time))/10000.0;
	}
	else{
		Ki = 0.02;
	}
	pLeft->SetKp(Kp);
	pRight->SetKp(Kp);
	pLeft->SetKi(Ki);
	pRight->SetKi(Ki);
	pLeft->SetKd(Kd);
	pRight->SetKd(Kd);
	pLeft->SetSetpoint(left_motor_target);
	pRight->SetSetpoint(right_motor_target);
	char speedChar[100]{};
	sprintf(speedChar, "%.1f,%d,%.2f,%d,%.2f\n", 1.0, curr_left, left_motor_target, curr_right, right_motor_target);
	const Byte speedByte = 85;
	pBt->SendBuffer(&speedByte, 1);
	pBt->SendStr(speedChar);
	SetMotorPower(GetMotorPower(0)+pLeft->Calc(curr_left),0);
	SetMotorPower(GetMotorPower(1)+pRight->Calc(curr_right),1);

}

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

	start_time=System::Time();

	Led::Config ConfigLed;
	ConfigLed.is_active_low = true;
	ConfigLed.id = 0;
	Led led0(ConfigLed);
	ConfigLed.id=1;
	Led led1(ConfigLed);

	led0.SetEnable(true);

	FutabaS3010::Config ConfigServo;
	ConfigServo.id = 0;
	FutabaS3010 servo(ConfigServo);
	servo.SetDegree(servo_angle);
	pServo = &servo;

	DirEncoder::Config ConfigEncoder;
	ConfigEncoder.id = 0;
	DirEncoder encoder0(ConfigEncoder);
	ConfigEncoder.id = 1;
	DirEncoder encoder1(ConfigEncoder);
	pEncoder0 = &encoder0;
	pEncoder1 = &encoder1;

	DirMotor::Config ConfigMotor;
	ConfigMotor.id = 0;
	DirMotor motor0(ConfigMotor);
	ConfigMotor.id = 1;
	DirMotor motor1(ConfigMotor);
	motor1.SetClockwise(false);
	pMotor0=&motor0;
	pMotor1=&motor1;

	JyMcuBt106::Config bt_config;
	bt_config.id = 0;
	bt_config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	bt_config.rx_isr = &bluetoothListener;
	JyMcuBt106 bt(bt_config);
	pBt = &bt;

	Timer::TimerInt time_img = 0;


	IncrementalPidController<float, float> pid_left(0,0,0,0);
	//  PositionalPidController<float,float> pid_left(0,0,0,0);
	pid_left.SetOutputBound(-1000, 1000);
	IncrementalPidController<float, float> pid_right(0,0,0,0);
	//  PositionalPidController<float,float> pid_right(0,0,0,0);
	pid_right.SetOutputBound(-1000, 1000);

	pLeft = &pid_left;
	pRight = &pid_right;

	Controller control(2,pMotor0,pMotor1,pEncoder0,pEncoder1,pServo);
	control.SetMotorTarget(100);


	St7735r::Config lcd_config;
	lcd_config.is_revert = true;
	St7735r lcd(lcd_config);
	lcd.Clear();

	LcdTypewriter::Config writerconfig;
	writerconfig.lcd = &lcd;
	LcdTypewriter writer(writerconfig);

	Joystick::Config joystick_config;
	joystick_config.id = 0;
	joystick_config.is_active_low = true;
	Joystick joystick(joystick_config);
//	while(1){
//		if(System::Time()%10==0)
//			control.Sync();
//		if(joystick.GetState()==Joystick::State::kSelect){
//			control.debug(&lcd,&writer);
//			}
//	}

//	Pit::Config pitConfig;
//	pitConfig.channel = 0;
//	pitConfig.count = 1000000;
//	pitConfig.isr = &control.Sync;
//	Pit pit(pitConfig);

  while (true){
	  while (System::Time() != time_img){
		  int curr_left=0, curr_right=0;
		  time_img = System::Time();


//		  if(time_img>resume_time){
//			  Ki=0.02;
//		  }

		  //test case
		  int start=time_img-start_time;
		  if(start>1000 && start<7000){
			  left_motor_target=0;
			  right_motor_target=0;
			  control.SetMotorTarget(0);
			  if(start>2500){
				  left_motor_target=400;
				  right_motor_target=400;
				  control.SetMotorTarget(400);
				  if(start>3000){
					  left_motor_target=200;
					  right_motor_target=200;
					  control.SetMotorTarget(200);
					  if(start>4500){
						  left_motor_target=0;
						  right_motor_target=0;
						  control.SetMotorTarget(0);
						  if(start>5000){
							  left_motor_target=100;
							  right_motor_target=100;
							  control.SetMotorTarget(100);
						  }
					  }
				  }
			  }
		  }

//		  char buff[10];
//		  sprintf(buff,"%d",control.GetEncoder());
//
//		  if(time_img%1000==0)
//		  writer.WriteBuffer(buff,10);

//		  control.debug(&lcd,&writer);
		  if(time_img % 10 == 0)
			  control.Sync(nullptr);

		  if(joystick.GetState()==Joystick::State::kLeft)
			  servo_angle--;
		  if(joystick.GetState()==Joystick::State::kRight)
			  servo_angle++;

		  control.SetServoDegree(servo_angle);

//		  if (time_img % 10 == 0){
//
//			  encoder0.Update();
//			  encoder1.Update();
//
//			  curr_left = encoder0.GetCount();
//			  curr_right = -encoder1.GetCount();
//
//			  if(time_img-start_time<10000){
//				  float temp = (10000-0.98*(time_img-start_time))/10000.0;
//				  if(std::abs(left_motor_target - curr_left)>20){
//					  Ki=temp;
//				  }
//				  else{
////					  Ki=0.02;
//				  }
//				  pid_left.SetKi(Ki);
//				  if(std::abs(right_motor_target - curr_right)>20){
//					  Ki=temp;
//				  }
//				  else{
////					  Ki=0.02;
//				  }
//				  pid_right.SetKi(Ki);
//			  }
//			  else{
//				  Ki=0.02;
//				  pid_left.SetKi(Ki);
//				  pid_right.SetKi(Ki);
//			  }
//			  pid_left.SetKp(Kp);
//			  pid_right.SetKp(Kp);
//			  pid_left.SetKd(Kd);
//			  pid_right.SetKd(Kd);
//			  pid_left.SetSetpoint(left_motor_target);
//			  pid_right.SetSetpoint(right_motor_target);
//
//			  char speedChar[100]{};
//			  sprintf(speedChar, "%.1f,%d,%.2f,%d,%.2f\n", 1.0, curr_left, left_motor_target, curr_right, right_motor_target);
//			  const Byte speedByte = 85;
//			  bt.SendBuffer(&speedByte, 1);
//			  bt.SendStr(speedChar);
//
////			  motor0.AddPower(pid_left.Calc(curr_left));
////			  motor1.AddPower(pid_right.Calc(curr_right));
//
//			  SetMotorPower(GetMotorPower(0)+pid_left.Calc(curr_left),0);
//			  SetMotorPower(GetMotorPower(1)+pid_right.Calc(curr_right),1);
//
////			  SetMotorPower(pid_left.Calc(curr_left),0);
////			  SetMotorPower(pid_right.Calc(curr_right),1);
//
//			  led1.SetEnable(time_img/250);
//		  }
//		  if (time_img % 500 == 0) {
//			  led0.Switch();
//			  if(joystick.GetState()==Joystick::State::kSelect){
//				  char buff[100];
//				  sprintf(buff,"kp:%.5lf \nki:%.5lf \nkd:%.5lf \nleft:%.5lf \n right:%.5lf\n left%.5lf\nright%.5lf\n%d\n%d",Kp,Ki,Kd,left_motor_target,right_motor_target,pid_left.Calc(curr_left),pid_right.Calc(curr_right),GetMotorPower(0),GetMotorPower(1));
////				  sprintf(buff,"%d\n%d",GetMotorPower(0),GetMotorPower(1));
////				  sprintf(buff,"%.3lf \n%.3lf \n%.3lf \n%.3lf \n%.3lf \n%.3lf ",pid_left.GetKp(),pid_left.GetKi(),pid_left.GetKd(),pid_right.GetKp(),pid_right.GetKi(),pid_right.GetKd());
//				  lcd.SetRegion(Lcd::Rect(0,0,128,160));
//				  writer.WriteBuffer(buff,100);
//			  }
//		  }
	  }
  }
}

} //namesapce pid_tuning
} //namespace util
