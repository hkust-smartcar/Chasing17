/*
 * Moving.h
 *
 *  Created on: Mar 31, 2017
 *      Author: Mr.King
 */

#ifndef CHASING17_ALGORITHM_KING_MOVING_H_
#define CHASING17_ALGORITHM_KING_MOVING_H_

#include <cassert>
#include <cstring>
#include <cstdint>
#include <algorithm>
#include <math.h>
#include "bluetooth.h"
#include "car_manager.h"

#include <libbase/k60/mcg.h>
#include <libbase/k60/gpio.h>
#include <libbase/k60/pin.h>
#include <libsc/system.h>
#include <libsc/led.h>
//CAMERA
#include <libsc/k60/ov7725.h>
#include <libsc/k60/ov7725_configurator.h>
//LCD
#include <libsc/lcd.h>
#include <libsc/st7735r.h>
#include <libsc/lcd_console.h>
//SERVO
#include <libsc/futaba_s3010.h>
//MOTOR
#include <libsc/alternate_motor.h>
//Joystick
#include "libsc/joystick.h"

//SELF_DEFINED VARIABLES in .h
enum Status { kRoundIn, kRoundOut, kCrossing, kNormal, kSshape, Fail };
const int ServoLeftBoundary = 1050; //Boundary 1050 980
const int ServoRightBoundary = 450; //Boundary 450 390
const int ServoStraightDegree = 800; // 800 720

using namespace libsc;
using namespace libsc::k60;
using namespace libbase::k60;
using namespace std;
/*-----------------------------------------------------------------Below start the declaration*/
class Moving {
 public:
  const int W = 80;
  const int H = 60;
  /*
   * 2: others
   * 0: road
   * 1: edge
   */
  int ext_camptr[80][60];
  int Left_edge[60]; //store the x_cor in each layer
  int Right_edge[60];
  int Center[60];

  Status RoadSituation(); /*Update Left_edge, Right_edge and Center, return current Status */
  bool HasCornerTesting();
  void NormalMovingTestingVersion1(FutabaS3010& servo, St7735r& lcd); /*Move based on midpoint only*/
  void NormalMovingTestingVersion2(FutabaS3010& servo, St7735r& lcd); /*Add roundabout as well as crossing judgement*/
  void NormalMovingTestingVersion3(FutabaS3010& servo, St7735r& lcd, AlternateMotor& motor_right, AlternateMotor& motor_left); /*Improve the performance of roundabout recognition*/
  void NormalMovingTestingVersion4(FutabaS3010& servo, St7735r& lcd, CarManager::Feature& feature); /*Moving + Feature output for Bluetooth testing*/
  void NormalMovingTestingVersion5(FutabaS3010& servo, St7735r& lcd, AlternateMotor& motor_right, AlternateMotor& motor_left); /*Speed will change - Target: Back up before vary ServoP*/
  void NormalMovingTestingVersionOld(FutabaS3010& servo, St7735r& lcd, AlternateMotor& motor_right, AlternateMotor& motor_left); /*Speed and Angle change based on ERROR*/
  void NormalMovingTestingVersionNew(FutabaS3010& servo, St7735r& lcd, AlternateMotor& motor_right, AlternateMotor& motor_left); /*Speed and Angle change based on ERROR*/

  /*Moving function*/
  bool NormalMoving(FutabaS3010& servo); /*Use center point method to control the car*/
  bool RoundMoving(FutabaS3010& servo); /*Turn left when encounter round road*/
  bool CrossingMoving(FutabaS3010& servo); /*Go straight when encounter crossing*/
  //bool SshapeMoving(FutabaS3010::Servo & servo);

  /*Indicator function*/
  bool HasRoad(); /*Detect is there a road in front*/
  bool HasCorner(const int layer, const bool find_L, const bool find_R); /*Detect the corner*/

  /*Printing function*/
  bool printCameraImage(const Byte* image, St7735r& lcd); /*Print the original camBuffer*/
  bool Print2Darray(Led& led, St7735r& lcd); /*Print the info in ext_camptr to LCD*/

  /*Helping function*/
  void extract_cam(const Byte* camBuffer); /*Extract the camBuffer from Byte to Bit and store into ext_camptr*/
  void Med_Filter(); /*Median Filter*/
  bool DoubleCheckRound(int center_xcor,
                        int center_ycor); /*Double check if the car has encountered round road or just sharp turning*/
  bool DoubleCheckCrossing(int center_xcor,
                           int center_ycor);/*Double check if the car has encountered crossing by looking ahead*/

  /*Testing + Tuning function*/
  void ServoAngleTuning(FutabaS3010& servo, St7735r& lcd); /*Use Joystick to adjust the servo angle*/
  void PrintingFrame(St7735r& lcd, int x, int y, int w, int h); /*Print a rectangle in (x,y) with size of w*h */
  void Printing4Frames(St7735r& lcd); /*Print 4 rectangles for testing*/
  void Printing6Frames(St7735r& lcd); /*Print 6 rectangles for testing*/

};
#endif /* CHASING17_ALGORITHM_KING_MOVING_H_ */
