/*
 * main.cpp
 *
 * Author: Leslie
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include "../../../inc/algorithm/leslie/smart_car.h"

#include <libsc/k60/jy_mcu_bt_106.h>

//Global variable-----------------------------------------------------------------------------------------------------------
const Uint CamHeight = 60;
const Uint CamWidth = 80;
const Uint MotorPower = 25;
const Uint MotorStSpeed = 25;
const Uint MotorSlowDownPower = 25;

#define ServoConstant 1.2;
#define MotorConstant 0.001;
extern Uint Cam2DArray[CamHeight][CamWidth];

//Speed PID control---------------------------------------------------------------------------------------------------------
void SpeedPID(AlternateMotor* motorA,
              AlternateMotor* motorB,
              DirEncoder* encoderA,
              DirEncoder* encoderB,
              int speedA,
              int speedB,
              LcdConsole* console) {

#define Kp 0.02
  bool breakA = false;
  bool breakB = false;
  Timer::TimerInt t1 = 0;
  Timer::TimerInt t2 = 0;
  int TimeDiff = 0;

  while (1) {
    encoderA->Update();
    encoderB->Update();
    t2 = System::Time();
    TimeDiff = t2 - t1;
    t1 = t2;
    int errorA = std::abs(encoderA->GetCount()) * 1000 / TimeDiff;
    int errorB = std::abs(encoderB->GetCount()) * 1000 / TimeDiff;
    char buff[20];
    char buff1[20];
    sprintf(buff, "%d ", errorA);
    sprintf(buff1, "%d ", errorB);
    console->WriteString(buff);
    console->WriteString(buff1);
    if (errorA == speedA) {
      breakA = true;
    }
    if (errorB == speedB) {
      breakB = true;
    }
    if (breakA == true && breakB == true) {
      break;
    }
    errorA = (speedA - errorA) * Kp;
    errorB = (speedB - errorB) * Kp;
    motorA->AddPower(errorA);
    motorB->AddPower(errorB);
  }
}

AlternateMotor* motorAPt, * motorBPt;
FutabaS3010* servoPt;
LcdConsole* console;
JyMcuBt106* BtPt;

//Main Program--------------------------------------------------------------------------------------------------------------
int smart_car(Joystick* FiveWaySwitch,
              St7735r* LCD,
              LcdConsole* Console,
              Ov7725* Cam,
              FutabaS3010* Servo,
              AlternateMotor* MotorA,
              AlternateMotor* MotorB,
              DirEncoder* EncoderA,
              DirEncoder* EncoderB,
              bool has_encoder) {
  System::Init();

  MotorA->SetClockwise(false);
  motorAPt = MotorA;
  MotorA->SetPower(250);
  MotorB->SetClockwise(true);
  motorBPt = MotorB;
  MotorB->SetPower(250);

  servoPt = Servo;
  Servo->SetDegree(800);//Servo 0 degree turned

  console = Console;
  Timer::TimerInt t = 0;

  Led::Config configLed1;
  configLed1.id = 1;
  Led led1(configLed1);

  Cam->Start();
  while (true) {
    while (t != System::Time()) {
      t = System::Time();
      if (t % 5 == 0) {
        const Byte* camPtr;
        const Byte tempInt = 49;
        const Byte* temp = &tempInt;
        camPtr = Cam->LockBuffer();
        CameraPrint(LCD, Cam);
        moveAlgo(camPtr, LCD, Servo);
        if (has_encoder) {
          EncoderA->Update();
          EncoderB->Update();
          if (EncoderA->GetCount() > 10 || EncoderB->GetCount() > 10) {
            MotorA->SetPower(150);
            MotorB->SetPower(150);
          } else {
            MotorA->SetPower(150);
            MotorB->SetPower(150);
          }
        }

        Cam->UnlockBuffer();
        led1.Switch();
      }
    }
  }
}
