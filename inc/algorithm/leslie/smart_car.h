#ifndef CHASING17_ALGORITHM_LESLIE_SMART_CAR_H_
#define CHASING17_ALGORITHM_LESLIE_SMART_CAR_H_

#include <cassert>
#include <cstring>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>

#include <sstream>

#include <functional>

//Looper Header File----------------------------------------------------------------------------------------------
#include <libutil/looper.h>

//Button Header File--------------------------------------------------------------------------------------------------------
#include<libsc/button.h>

//led Header File-----------------------------------------------------------------------------------------------------------
#include <libsc/led.h>

//5 way switch HeaderFile------------------------------------------------------------------------------------------
#include <libsc/joystick.h>

//LCD Header File-----------------------------------------------------------------------------------------------------------
#include<libsc/st7735r.h>
#include<libsc/lcd_console.h>

//Camera Header File--------------------------------------------------------------------------------------------------------
#include<libsc/k60/ov7725.h>

//Servo Header File---------------------------------------------------------------------------------------------------------
#include<libsc/futaba_s3010.h>

//Motor Header File---------------------------------------------------------------------------------------------------------
#include<libsc/alternate_motor.h>
#include<libsc/motor.h>

//Dir Encoder Header File
#include<libsc/dir_encoder.h>

//Camera CPP------------------------------------------------------------------------------------------------------
#include "algorithm/leslie/camera.h"

//namespace-----------------------------------------------------------------------------------------------------------------
using namespace libsc;
using namespace libsc::k60;
using namespace libbase::k60;
using namespace libutil;
using namespace std;

int smart_car(Joystick* FiveWaySwitch,
              St7735r* LCD,
              LcdConsole* Console,
              Ov7725* Cam,
              FutabaS3010* Servo,
              AlternateMotor* MotorA,
              AlternateMotor* MotorB,
              DirEncoder* EncoderA,
              DirEncoder* EncoderB,
              bool has_encoder);

#endif  // CHASING17_ALGORITHM_LESLIE_SMART_CAR_H_
