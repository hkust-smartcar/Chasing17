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
Corners left_corners;
Corners right_corners;
const Byte* CameraBuf;
k60::Ov7725* pCamera = nullptr;
St7735r* pLcd = nullptr;
Led* pLed3 = nullptr;

int max(int a, int b) {return (a>b ? a : b);}
int min(int a, int b) {return (a<b ? a : b);}

// for edge finding, in CCW dir
const int dx[9] = { 0,-1,-1,-1, 0, 1, 1, 1, 0};
const int dy[9] = { 1, 1, 0,-1,-1,-1, 0, 1, 1};
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
 * 1. Search left starting point with decreasing x at (x_0, y_0) = (width/2, 1)
 * 2. If to no avail, choose (1, 1) as starting point
 * 3. Search right starting point with increasing x at (x_0, y_0)
 * 4. If to no avail, choose (width-1, 1) as starting point
 */
void Capture(){
	left_edge.points.clear();
	right_edge.points.clear();
	bool found_left = false, found_right = false;
	int left_x = -1, left_y = -1;
	int right_x = -1, right_y = -1;
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
	if (!found_left){
		left_x = 1;
		left_y = 1;
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
		right_x = CameraSize.w-1;
		right_y = 1;
	}

	pLed3->Switch();
	left_edge.push(left_x, left_y);
	right_edge.push(right_x, right_y);
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
 * 1. Find up to (height-1) points, would consider boundary as a part of the edge if necessary
 * 2. Fix the trespassing issues (i.e. Left edge got into right edge's way etc)
 *  - It seems to only happen when it is in a turn
 *  - For left edge, if the edge is suddenly turning right at (x, y), check if (width/2, y ~ height-1) consists mostly black
 *  - If so, it is probably a turn; otherwise, it is probably a cross road
 *  - If it is a turn, do not allow the edge to turn; otherwise it is fine
 *  - Do the same (but mirrored) actions to right edge
 */
bool FindEdges(){
	left_corners.points.clear();
	right_corners.points.clear();
	int error_cnt = -1;
	int prev_size = left_edge.points.size();

	//Find left edge for length 60
	if (left_edge.points.size() != 0){
		bool flag_break = false;
		do {
			if (prev_size == left_edge.points.size()){
				error_cnt++;
				if (error_cnt > 3){
					return false;
				}
			}
			prev_size = left_edge.points.size();
			int prev_x = left_edge.points.back().first;
			int prev_y = left_edge.points.back().second;


			if (getFilteredBit(CameraBuf, prev_x, prev_y+1) == 0){ //if white, find in CCW
				if (prev_x == 1){
					left_edge.push(prev_x, prev_y+1);
				} else {
					for (int i = 0; i < 8; i++){
						if(getFilteredBit(CameraBuf, prev_x+dx[i], prev_y+dy[i]) == 1){
							left_edge.push(prev_x+dx[i-1], prev_y+dy[i-1]);
							break;
						}
					}
				}
			} else {
				for (int i = 7; i >= 0; i--){
					if(getFilteredBit(CameraBuf, prev_x+dx[i], prev_y+dy[i]) == 0){
						if ((i == 6 || i == 5 || i == 7) && prev_x == 1){
							//need to check for trespassing
							int cnt_black = 0;
							for (int j = prev_y; j < CameraSize.h; j++){
								cnt_black += getFilteredBit(CameraBuf, CameraSize.w/2, j);
							}
							if (cnt_black > 0.5 * (CameraSize.h - prev_y)){
								flag_break = true;
								break;
							}
						}
						left_edge.push(prev_x+dx[i], prev_y+dy[i]);
						break;
					}
				}
			}
		if (left_edge.points.back() == left_edge.points[left_edge.size() - 2]){ //the edge start backtrack
			left_edge.points.pop_back();
			flag_break = true;
		}
		if (left_edge.points.back().second == CameraSize.h - 1){ //the edge reaches the top
			flag_break = true;
		}

		//Check corners
		{
			int CornerCheck = 0;
			int total = 0;
			auto last = left_edge.points.back();
			if (last.first - 3 <= 0 || last.first + 3 > CameraSize.w - 1 || last.second - 3 <= 0 || last.second +3 > CameraSize.h -1){
				continue;
			}
			for (int i = (last.first - 3); i < (last.first + 3); i++){
				for (int j = (last.second - 3); j < (last.second + 3); j++){
					CornerCheck += getFilteredBit(CameraBuf, i, j);
					total++;
				}
			}
			//if in this threshold, consider as corner
			if (CornerCheck > total * 0.01 && CornerCheck < total * 0.20){
				left_corners.push(last.first, last.second);
			}
		}
		} while (left_edge.points.size() <= CameraSize.h-1 && flag_break == false);
	}

	error_cnt = -1;
	prev_size = right_edge.points.size();

	//Find right edge for length 60
	if (right_edge.points.size() != 0){
		bool flag_break = false;
		do {
			if (prev_size == right_edge.points.size()){
				error_cnt++;
				if (error_cnt > 3){
					return false;
				}
			}
			prev_size = right_edge.points.size();
			int prev_x = right_edge.points.back().first;
			int prev_y = right_edge.points.back().second;

			if (getFilteredBit(CameraBuf, prev_x, prev_y+1) == 0){ //if white, find in CW
				if (prev_x == CameraSize.w - 1){
					right_edge.push(prev_x, prev_y+1);
				} else {
					for (int i = 7; i >= 0; i--){
						if(getFilteredBit(CameraBuf, prev_x+dx[i], prev_y+dy[i]) == 1){
							right_edge.push(prev_x+dx[i+1], prev_y+dy[i+1]);
							break;
						}
					}
				}
			} else {
				for (int i = 0; i < 8; i++){

					if(getFilteredBit(CameraBuf, prev_x+dx[i], prev_y+dy[i]) == 0){
						if ((i == 2 || i == 1 || i == 3) && prev_x == CameraSize.w-1){
							//need to check for trespassing
							int cnt_black = 0;
							for (int j = prev_y; j < CameraSize.h; j++){
								cnt_black += getFilteredBit(CameraBuf, CameraSize.w/2, j);
							}
							if (cnt_black > 0.5 * (CameraSize.h - prev_y)){
								flag_break = true;
								break;
							}
						}
						right_edge.push(prev_x+dx[i], prev_y+dy[i]);
						break;
					}
				}
			}

			if (right_edge.points.back() == right_edge.points[right_edge.size() - 2]){ //the edge start backtrack
				right_edge.points.pop_back();
				flag_break = true;
			}

			if (right_edge.points.back().second == CameraSize.h - 1){ //the edge reaches the top
				flag_break = true;
			}

			//Check corners
			{
				int CornerCheck = 0;
				int total = 0;
				auto last = right_edge.points.back();
				if (last.first - 3 <= 0 || last.first + 3 > CameraSize.w - 1 || last.second - 3 <= 0 || last.second +3 > CameraSize.h -1){
					continue;
				}
				for (int i = max(0, last.first - 3); i < min(CameraSize.w-1, last.first + 3); i++){
					for (int j = max(0, last.second - 3); j < min(CameraSize.h-1, last.second + 3); j++){
						CornerCheck += getFilteredBit(CameraBuf, i, j);
						total++;
					}
				}
				//if in this threshold, consider as corner
				if (CornerCheck > total * 0.01 && CornerCheck < total * 0.20){
					right_corners.push(last.first, last.second);
				}
			}
		} while (right_edge.points.size() <= CameraSize.h-1 && flag_break == false);
	}
	return true;
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
 * @brief Print corners
 */
void PrintCorner(Corners corners, uint16_t color){
	for (auto&& entry : corners.points){
		pLcd->SetRegion(Lcd::Rect(entry.first, CameraSize.h - entry.second - 1, 4, 4));
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
 *  - Would now only work when both (real) left edge and right edge are within camera
 *  - Otherwise, the path may go out of edge
 * 2. TODO (mcreng): Naive psuedo-optimal path ("Curve fitting")
 */
void GenPath(){
	int left_size = left_edge.size();
	int right_size = right_edge.size();

	path.points.clear();

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
	ConfigLed.id = 3;
	Led led3(ConfigLed);
	pLed3 = &led3;

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
	lcdConfig.is_revert = false;
	St7735r lcd(lcdConfig);
	pLcd = &lcd;

	CarManager::Config ConfigMgr;
//	ConfigMgr.servo = std::move(pServo);
	ConfigMgr.epc = std::move(pMpc);
	CarManager::Init(std::move(ConfigMgr));

	Timer::TimerInt time_img = 0;
/*
	//Servo test

	pServo->SetDegree(servo_bounds.kLeftBound);
	System::DelayMs(1000);
	pServo->SetDegree(servo_bounds.kRightBound);
	System::DelayMs(1000);
	pServo->SetDegree(servo_bounds.kCenter);
	System::DelayMs(1000);
*/
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
				PrintCorner(left_corners, Lcd::kPurple); //Print left_corner
				PrintCorner(right_corners, Lcd::kPurple); //Print right_corner
//				CarManager::Feature feature = IdentifyFeat(); //Identify feature
				GenPath(); //Generate path
				PrintEdge(path, Lcd::kGreen); //Print path
				led0.Switch(); //heart beat
			}
		}

	}

}
}
} // namespace algorithm::optimal
