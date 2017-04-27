/*
 * main.h
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Peter Tse (mcreng), Dipsy Wong
 *
 */

#include "algorithm/optimal/main.h"

#include "libsc/alternate_motor.h"
#include "libsc/dir_encoder.h"
#include "libsc/futaba_s3010.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libsc/lcd_console.h"
#include "libsc/led.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"
#include "libsc/k60/ov7725.h"
#include "libsc/joystick.h"

#include "bluetooth.h"
#include "car_manager.h"
#include "util/util.h"

using namespace libsc;

namespace algorithm{
namespace optimal{

Edges left_edge;
Edges right_edge;
Edges path;
const Byte* CameraBuf;
k60::Ov7725* pCamera = nullptr;
St7735r* pLcd = nullptr;

int max(int a, int b) {return (a>b ? a : b);}
int min(int a, int b) {return (a<b ? a : b);}

// for edge finding
const int dx[8]={ 0,-1,-1,-1, 0, 1, 1, 1};
const int dy[8]={-1,-1, 0, 1, 1, 1, 0,-1};

/**
 * @brief To fetch filtered bit
 * @param buff Camera buffer
 * @param (x,y): coordinate system in which bottom left corner := (0, 0)
 * @return filtered bit
 */
int getFilteredBit(const Byte* buff, int x, int y){
	if (x<0 || x>CameraSize.w-1 || y < 0 || y > CameraSize.h-1) return -1; // Out of bound = black
	y = CameraSize.h - 1 - y; // set y = 0 at bottom

	return buff[y*CameraSize.w/8+x/8] & 0x80>>x%8;

	// Median Filter
	int count=0,total=0;

	for(int i=max(0,x-1); i<min(CameraSize.w-1,x+1); i++){
		for(int j=max(0,y-1); j<min(CameraSize.h-1,y+1); j++){
			total++;
			count+=buff[j*CameraSize.w/8+i/8] & 0x80>>i%8;
		}
	}
	// Median Filter
	return (count>total/2) ? 1 : 0;
}

/**
 * @brief Consider the left/right of vector at current point pointing to next point
 * @param j direction
 * @param d original left/right-ness
 * @return new left/right-ness
 */
int FindDirection(int j, int d){
	j %= 8;
	if (j>0&&j<4) return -1;
	if (j>4)return 1;
	if (j==4)return d;
	return 4;
}

/**
 * @brief capture picture until two base points are identified.
 */
void Capture(){
	bool failed, found_left, found_right;
	int left_x = -1, left_y = -1;
	int right_x = -1, right_y = -1;
	do{
		failed = false; found_left = false, found_right = false;
//		while(!pCamera->IsAvailable());
		CameraBuf = pCamera->LockBuffer();
		pCamera->UnlockBuffer();


		//Search horizontally
		for (int i = CameraSize.w/2; i >= 0; i--){
			if (getFilteredBit(CameraBuf, i, 0) == 1){
				left_x = i;
				left_y = 0;
				found_left = true;
				break;
			}
		}


		// Search vertically
		if (!found_left){
			for (int i = 0; i < CameraSize.h; i ++){
				if (getFilteredBit(CameraBuf, 0, i) == 1){
					left_y = i;
					left_x = 0;
					found_left = true;
					break;
				}
			}
		}

//		if (!found_left) failed = true;

		//Search horizontally
		for (int i = CameraSize.w/2; i < CameraSize.w; i++){
			if (getFilteredBit(CameraBuf, i, 0) == 1){
				right_x = i;
				right_y = 0;
				found_right = true;
				break;
			}
		}
		if (!found_right){
			//Search vertically
			for (int i = 0; i < CameraSize.h; i++){
				if (getFilteredBit(CameraBuf, CameraSize.w - 1, i) == 1){
					right_y = i;
					right_x = CameraSize.w-1;
					found_right = true;
					break;
				}
			}
		}
//		if (!found_right) failed = true;
		if (left_x == right_x && left_y == right_y) failed = true;
	} while (failed);
	if (!failed){
		if (found_right){
			left_edge.push(left_x, left_y);
		}
		if (found_left){
			right_edge.push(right_x, right_y);
		}
	}
}

/**
 * Print image to LCD
 */
void PrintImage(){
	pLcd->SetRegion(Lcd::Rect(0,0,CameraSize.w,CameraSize.h));
	pLcd->FillBits(0x0000,0xFFFF,CameraBuf,pCamera->GetBufferSize()*8);
}

/**
 * Find edges
 */
void FindEdges(){
	int left_from = 0, right_from = 0; //to determine direction
	bool find_left = true; //for breaking

	for (int cnt = 0; cnt < 199; cnt++){

		int collision = -1;

		if (find_left){
			for (int i=left_from+1; i<left_from+9;i++){
				const int j=i%8;

				if(!getFilteredBit(CameraBuf, left_edge.points.back().first + dx[j] , left_edge.points.back().second + dy[j])){//if the point is white, it is a new point of edge
					left_edge.push(left_edge.points.back().first + dx[j], left_edge.points.back().second + dy[j]);
					left_from=j+4;

					auto a = left_edge.points.back();
					auto b = left_edge.points[left_edge.size() - 2];

					if ((a.first <= 3 && b.first > 3) || (a.first >= CameraSize.w - 4 && b.first < CameraSize.w - 4) || (a.second <= CameraSize.h - 4 && b.second > CameraSize.h - 4)){
						find_left = false;
					}

					break;
				}
			}
		}

	}

}

/**
 * Print edges
 */
void PrintEdge(Edges path){
	for (auto&& entry : path.points){
		pLcd->SetRegion(Lcd::Rect(entry.first, CameraSize.h - entry.second - 1, 2, 2));
		pLcd->FillColor(Lcd::kRed);
	}

}

/**
 * Identify feature
 */
void IdentifyFeat(){

}

/**
 * Path generation
 * 1. Weighted average path ("Center line")
 * 2. Naive psuedo-optimal path ("Curve fitting")
 */
void GenPath(){

}

void main(CarManager::ServoBounds servo_bounds){
	System::Init();

	k60::Ov7725::Config cameraConfig;
	cameraConfig.id = 0;
	cameraConfig.w = CameraSize.w;
	cameraConfig.h = CameraSize.h;
	cameraConfig.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
	k60::Ov7725 camera(cameraConfig);
	pCamera = &camera;

	FutabaS3010::Config ConfigServo;
	ConfigServo.id = 0;
	FutabaS3010 servo(ConfigServo);
//	std::unique_ptr<FutabaS3010> pServo(new FutabaS3010(ConfigServo));
	FutabaS3010* pServo = &servo;

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

	std::unique_ptr<util::MpcDual> pMpc(new util::MpcDual(&motor0, &motor1, &encoder0, &encoder1));

	k60::JyMcuBt106::Config ConfigBT;
	ConfigBT.id = 0;
	ConfigBT.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	k60::JyMcuBt106 bt(ConfigBT);

	St7735r::Config lcdConfig;
	lcdConfig.is_revert = true;
	St7735r lcd(lcdConfig);
	pLcd = &lcd;

	CarManager::Config ConfigMgr;
//	ConfigMgr.servo = std::move(pServo);
	ConfigMgr.epc = std::move(pMpc);
	CarManager::Init(std::move(ConfigMgr));

	Timer::TimerInt time_img = 0;

	bool found[CameraSize.w][CameraSize.h];
	for (int i = 0; i < CameraSize.w; i++){
		for (int j = 0; j < CameraSize.h; j++){
			found[i][j] = false;
		}
	}

	//Servo test

	pServo->SetDegree(servo_bounds.kLeftBound);
	System::DelayMs(1000);
	pServo->SetDegree(servo_bounds.kRightBound);
	System::DelayMs(1000);
	pServo->SetDegree(servo_bounds.kCenter);
	System::DelayMs(1000);

	camera.Start();

	while (true){
		while (time_img != System::Time()){
			time_img = System::Time();
			if (time_img % 50 == 0){
				Capture(); //Capture until two base points are identified
				PrintImage(); //Print LCD
//				FindEdges();
				PrintEdge(left_edge);
				PrintEdge(right_edge);
				IdentifyFeat();
				GenPath();
				PrintEdge(path);
				left_edge.points.clear();
				right_edge.points.clear();
			}
		}

	}



}
}
} // namespace algorithm::optimal
