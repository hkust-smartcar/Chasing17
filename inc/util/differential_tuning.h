/*
 * differential_tuning.h
 *
 *  Created on: Jun 30, 2017
 *      Author: dipsy
 */

#ifndef INC_UTIL_DIFFERENTIAL_TUNING_H_
#define INC_UTIL_DIFFERENTIAL_TUNING_H_

#include <sstream>

#include "libsc/dir_encoder.h"
#include "libsc/futaba_s3010.h"
#include "libsc/st7735r.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/led.h"
#include "libsc/system.h"
#include "libsc/joystick.h"
#include "debug_console.h"

#include "bluetooth.h"
#include "util/util.h"

using namespace libsc;
using namespace libsc::k60;
using namespace libutil;


namespace util {
namespace differential_tuning {


int32_t encoder_value0 = 0,encoder_value1 = 0;
uint16_t servoAngle = 0;
int car = 0;

void Reset(){
	if (car==0){
		servoAngle = 755;
	}
	else{
		servoAngle = 845;
	}

	encoder_value0 = 0;
	encoder_value1 = 0;


}

void Clear(){
	encoder_value0 = 0;
	encoder_value1 = 0;
}


void main(int c){
	car = c;
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
	  servo.SetDegree(servoAngle);

	  Reset();

	  DirEncoder::Config ConfigEncoder;
	  ConfigEncoder.id = 0;
	  DirEncoder encoder0(ConfigEncoder);
	  ConfigEncoder.id = 1;
	  DirEncoder encoder1(ConfigEncoder);
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

	  DebugConsole console(&joystick,&lcd,&writer,10);
	  console.PushItem("servo_angle",&servoAngle,1);
	  console.PushItem("encoder0",&encoder_value0,float(0.0));
	  console.PushItem("encoder1",&encoder_value1,float(0.0));
	  console.PushItem("reset",(bool*)(nullptr),"","");
	  Item item = console.GetItem(3);
	  item.listener = &Clear;
	  console.SetItem(3,item);

	  console.ListItems();

	  while (true){

		  console.Listen();
		  encoder0.Update();
		  encoder1.Update();
		  int dx=encoder0.GetCount(),dy=encoder1.GetCount();
		  if(dx>1000||dx<-1000||dy>1000||dy<-1000)continue;
		  encoder_value0 += dx;
		  encoder_value1 += dy;
		  if(System::Time()%500==0){
		  console.ListItems();
		  }

		  servo.SetDegree(servoAngle);
	  }

	  return;
}

}
}




#endif /* INC_UTIL_DIFFERENTIAL_TUNING_H_ */
