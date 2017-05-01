/*
 * main.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Peter Tse (mcreng), Dipsy Wong
 *
 * Optimal Path Algorithm CPP File
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
 * @brief To fetch filtered bit, 1 = black; 0 = white
 * @param buff Camera buffer
 * @param (x,y): coordinate system in which bottom left corner := (0, 0)
 * @return filtered bit
 */
int getFilteredBit(const Byte* buff, int x, int y){
	if (x<=0 || x>CameraSize.w-1 || y <= 0 || y > CameraSize.h-1) return -1; //-1; // Out of bound = black
	y = CameraSize.h - 1 - y; // set y = 0 at bottom

	//return buff[y*CameraSize.w/8 + x/8] >> (7 - (x%8)) & 1;	//buff[y*CameraSize.w/8+x/8] & 0x80>>x%8;

	// Median Filter
	int count=0,total=0;

	for(int i=max(0,x-1); i<min(CameraSize.w-1,x+1); i++){
		for(int j=max(0,y-1); j<min(CameraSize.h-1,y+1); j++){
			total++;
			count+=buff[j*CameraSize.w/8 + i/8] >> (7 - (i%8)) & 1;
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
 * @brief Capture picture until two base points are identified.
 *
 * Algorithm:
 * 1. Search left starting point with decreasing x at (x_0, y_0) = (width/2, 0)
 * 2. If to no avail, search upwards with increasing y at (x_1, y_1) = (0, 0)
 * 3. If to no avail again, consider no left edge exists
 * 4. Search right starting point with increasing x at (x_0, y_0)
 * 5. If to no avail, search upwards with increasing y at (x_2, y_2) = (width-1, 0)
 * 6. If to no avail again, consider no right edge exists
 * 7. If both starting points are the same, consider failure and should capture again
 */
void Capture(){
	left_edge.points.clear();
	right_edge.points.clear();
	bool failed, found_left, found_right;
	int left_x = -1, left_y = -1;
	int right_x = -1, right_y = -1;
	do{
		failed = false; found_left = false, found_right = false;
//		while(!pCamera->IsAvailable());
		CameraBuf = pCamera->LockBuffer();
		pCamera->UnlockBuffer();

		//Search horizontally
		for (int i = CameraSize.w/2; i > 0; i--){ //TODO: Try to understand why at (0,0) it always return 1
			if (getFilteredBit(CameraBuf, i, 1) == 1){
				left_x = i + 1;
				left_y = 1;
				found_left = true;
				break;
			}
		}

		// Search vertically
		if (!found_left){
			for (int i = 1; i < CameraSize.h; i ++){
				if (getFilteredBit(CameraBuf, 1, i) == 1){
					left_y = i - 1;
					left_x = 1;
					found_left = true;
					break;
				}
			}
		}

		//Search horizontally
		for (int i = CameraSize.w/2; i < CameraSize.w; i++){
			if (getFilteredBit(CameraBuf, i, 1) == 1){
				right_x = i - 1;
				right_y = 1;
				found_right = true;
				break;
			}
		}
		if (!found_right){
			//Search vertically
			for (int i = 0; i < CameraSize.h; i++){
				if (getFilteredBit(CameraBuf, CameraSize.w - 1, i) == 1){
					right_y = i - 1;
					right_x = CameraSize.w-1;
					found_right = true;
					break;
				}
			}
		}
		if (left_x == right_x && left_y == right_y) failed = true;
	} while (failed);
	if (!failed){
		if (found_left){
			left_edge.push(left_x, left_y);
		}
		if (found_right){
			right_edge.push(right_x, right_y);
		}
	}
}

/**
 * @brief Print image to LCD
 */
void PrintImage(){
	pLcd->SetRegion(Lcd::Rect(0,0,CameraSize.w,CameraSize.h));
	pLcd->FillBits(0x0000,0xFFFF,CameraBuf,pCamera->GetBufferSize()*8);
}

/**
 * @brief Find edges
 *
 * Algorithm:
 * 1. Allow left edge and right edge to be found until they hit one of the boundaries
 * 2. Check where they ends
 * 	- If both ends at right/left --> done
 * 	- Otherwise, search again until both hits top
 * 	- TODO (mcreng): If the two edges meet, consider failure and should capture again
 */
void FindEdges(){
	int left_from = 0, right_from = 0; //to determine direction
	int left_ends = -1, right_ends = -1; // -1: not ended; 0: Left; 1: Top; 2: Right

	//Find left edge until hitting boundary
	if (left_edge.points.size() != 0){
		do {

			for (int i=left_from+1; i<left_from+9;i++){
				const int j=i%8;
				int size = left_edge.size();
				if(!getFilteredBit(CameraBuf, left_edge.points.back().first + dx[j] , left_edge.points.back().second + dy[j])){//if the point is white, it is a new point of edge
					left_edge.push(left_edge.points.back().first + dx[j], left_edge.points.back().second + dy[j]);
					left_from=j+4;
					break;
				}
			}

			auto a = left_edge.points.back();

			if (a.second == CameraSize.h - 1){
				left_ends = 1;
			} else if (a.first == 1){
				left_ends = 0;
			} else if (a.first == CameraSize.w - 1){
				left_ends = 2;
			};

		} while (left_ends == -1);
	}

	//Find right edge until hitting boundary
	if (right_edge.points.size() != 0){
		do {

			for (int i=right_from+7; i>=right_from;i--){
				const int j=i%8;

				int size = right_edge.size();

				if(!getFilteredBit(CameraBuf, right_edge.points.back().first + dx[j] , right_edge.points.back().second + dy[j])){//if the point is white, it is a new point of edge
					right_edge.push(right_edge.points.back().first + dx[j], right_edge.points.back().second + dy[j]);
					right_from=j+4;
					break;
				}
			}

			auto a = right_edge.points.back();

			if (a.second == CameraSize.h - 1){
				right_ends = 1;
			} else if (a.first == 1){
				right_ends = 0;
			} else if (a.first == CameraSize.w - 1){
				right_ends = 2;
			};

		} while (right_ends == -1);
	}
	return;
	if (left_ends == 0 && right_ends == 2) return;


	if (left_ends == right_ends){
		return; //Halt if both ends at the same boundary
	} else {

		//Find left edge until hitting top edge
		if (left_edge.points.size() != 0){
			while (left_ends != 1){

				for (int i=left_from+1; i<left_from+9;i++){
					const int j=i%8;

					if(!getFilteredBit(CameraBuf, left_edge.points.back().first + dx[j] , left_edge.points.back().second + dy[j])){//if the point is white, it is a new point of edge
						left_edge.push(left_edge.points.back().first + dx[j], left_edge.points.back().second + dy[j]);
						left_from=j+4;
						break;
					}
				}

				auto a = left_edge.points.back();

				if (a.second == CameraSize.h - 1){
					left_ends = 1;
				};
			}
		}

		//Find right edge until hitting top edge
		if (right_edge.points.size() != 0){
			while (right_ends != 1){

				for (int i=right_from+7; i>=right_from;i--){
					const int j=i%8;

					if(!getFilteredBit(CameraBuf, right_edge.points.back().first + dx[j] , right_edge.points.back().second + dy[j])){//if the point is white, it is a new point of edge
						left_edge.push(right_edge.points.back().first + dx[j], right_edge.points.back().second + dy[j]);
						right_from=j+4;
						break;
					}
				}

				auto a = right_edge.points.back();

				if (a.second == CameraSize.h - 1){
					right_ends = 1;
				};
			}
		}
	}
}

/**
 * @brief Print edges
 */
void PrintEdge(Edges path, uint16_t color){
	for (auto&& entry : path.points){
		pLcd->SetRegion(Lcd::Rect(entry.first, CameraSize.h - entry.second - 1, 2, 2));
		pLcd->FillColor(color);
	}

}

/**
 * @brief Identify feature
 */
CarManager::Feature IdentifyFeat(){


	return CarManager::Feature::kStraight;
}

/**
 * Path generation
 * 1. Weighted average path ("Center line")
 * 2. TODO (mcreng): Naive psuedo-optimal path ("Curve fitting")
 */
void GenPath(){
	int left_size = left_edge.size();
	int right_size = right_edge.size();

	if (left_size < right_size){
		for (int i = 0; i < right_edge.size(); i++){
			int temp_x = (left_edge.points[(left_size * i) / right_size].first + right_edge.points[i].first) / 2;
			int temp_y = (left_edge.points[(left_size * i) / right_size].second + right_edge.points[i].second) / 2;
			path.push(temp_x, temp_y);
		}
	} else {
		for (int i = 0; i < left_edge.size(); i++){
			int temp_x = (left_edge.points[i].first + right_edge.points[(right_size * i) / left_size].first) / 2;
			int temp_y = (left_edge.points[i].second + right_edge.points[(right_size * i) / left_size].second) / 2;
			path.push(temp_x, temp_y);
		}
	}

}

void main(CarManager::ServoBounds servo_bounds){
	System::Init();

	Led::Config ConfigLed;
	ConfigLed.id = 0;
	Led led0(ConfigLed);

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
			if (time_img % 100 == 0){
				Capture(); //Capture until two base points are identified
				PrintImage(); //Print LCD
				FindEdges(); //Find edges
				PrintEdge(left_edge, Lcd::kRed); //Print left_edge
				PrintEdge(right_edge, Lcd::kBlue); //Print right_edge
//				CarManager::Feature feature = IdentifyFeat(); //Idetnify feature
//				GenPath(); //Generate path
//				PrintEdge(path, Lcd::kGreen); //Print path
				led0.Switch();
			}
		}

	}



}
}
} // namespace algorithm::optimal
