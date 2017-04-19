#ifndef CHASING17_ALGORITHM_LESLIE_CAMERA_H_
#define CHASING17_ALGORITHM_LESLIE_CAMERA_H_

//Vector Header File-----------------------------------------------------------------------------------------------
#include <vector>

//Looper Header File-----------------------------------------------------------------------------------------------
#include <libutil/looper.h>

//led Header File-----------------------------------------------------------------------------------------------------------
#include <libsc/led.h>

//5 way switch Header File-----------------------------------------------------------------------------------------
#include <libsc/joystick.h>

//LCD Header File-----------------------------------------------------------------------------------------------------------
#include <libsc/st7735r.h>
#include <libsc/lcd_console.h>

//Servo Header File---------------------------------------------------------------------------------------------------------
#include <libsc/futaba_s3010.h>

//Camera Header File-----------------------------------------------------------------------------------------------
#include <libsc/k60/ov7725.h>

using namespace libsc;
using namespace libsc::k60;
void Camera2DConverter(const Byte* CameraBuffer);
void CameraFilter();
void PathWidthFinder(Joystick* FiveWaySwitch, LcdConsole* console);
void EdgeFinder(const Byte* camBuffer, St7735r* lcd, FutabaS3010* servo);
void moveAlgo(const Byte* camBuffer, St7735r* lcd, FutabaS3010* servo);
void PathFinder();
void CenterLine();
void camera();
void CameraPrint(St7735r* lcd, Ov7725* Cam);
void Camera2DArrayPrint(St7735r* lcd);
void Camera2DArrayPrintTest(St7735r* lcd, Ov7725* Cam);
int featureIdentify(const Byte* camBuffer);

#endif  // CHASING17_ALGORITHM_LESLIE_CAMERA_H_
