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

#include "debug_console.h"
#include "algorithm/optimal/img_car1.h"
#include "algorithm/optimal/img_car2.h"

using namespace libsc;

namespace algorithm{
namespace optimal{

Edges left_edge;
Edges right_edge;
Edges path;
Corners left_corners;
Corners right_corners;
const Byte* CameraBuf;
//Byte WorldBuf[128*20];
std::unique_ptr<k60::Ov7725> pCamera = nullptr;
St7735r* pLcd = nullptr;
Led* pLed3 = nullptr;
LcdTypewriter* pWriter = nullptr;
CarManager::ServoBounds* pServoBounds = nullptr;
FutabaS3010* pServo = nullptr;

CarManager::ServoBounds servo_bounds;
CarManager::Car car;

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
	//y = CameraSize.h - 1 - y; // set y = 0 at bottom

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


//test distortion world view
//w_x,w_y world coordinate 128*160
//i_x,i_y image coordinate 128*480

//get bit value from camerabuf using camera coordinate system
bool getBit(int i_x, int i_y){
	if (i_x<=0 || i_x>CameraSize.w-1 || i_y <= 0 || i_y > CameraSize.h-1) return -1;
	return CameraBuf[i_y*CameraSize.w/8 + i_x/8] >> (7 - (i_x%8)) & 1;
}

/**
 * @brief To fetch filtered bit, 1 = black; 0 = white
 * @param (x,y): coordinate system in which bottom left corner := (0, 0)
 * @return world bit
 */
bool getWorldBit(int w_x, int w_y){
	switch (car){
	case CarManager::Car::kCar1:
	{
		w_y = 160 - w_y;
		int i_x,i_y;
		i_x = worldview::car1::transformMatrix[w_x][w_y][0];
		i_y = worldview::car1::transformMatrix[w_x][w_y][1];
		return getFilteredBit(CameraBuf, i_x,i_y);
	}
		break;
	case CarManager::Car::kCar2:
	{
		w_y = 160 - w_y;
		int i_x,i_y;
		i_x = worldview::car2::transformMatrix[w_x][w_y][0];
		i_y = worldview::car2::transformMatrix[w_x][w_y][1];
		return getFilteredBit(CameraBuf, i_x,i_y);
	}
		break;
	}

}

void PrintWorldImage(){
	Byte temp[128/8];
	for (int i=160; i>0; --i){
		for (int j=0; j<128; j++){
			temp[j/8]<<=1;
			temp[j/8]+=getWorldBit(j,i);
			//WorldBuf[i*128/8+j/8]<<=1;
			//WorldBuf[i*128/8+j/8]+=getWorldBit(j,i);
		}
		pLcd->SetRegion(Lcd::Rect(0,160-i,128,1));
		pLcd->FillBits(0x0000,0xFFFF,temp,128);
		//pLcd->FillColor(getWorldBit(j,i)?Lcd::kBlack:Lcd::kWhite);
	}
	return;
}

//end test distortion


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
	for (int i = WorldSize.w/2; i > 0; i--){
		if (getWorldBit( i, 20) == 1){
			left_x = i + 1;
			left_y = 20;
			found_left = true;
			break;
		}
	}
	if (!found_left){
		left_x = 1;
		left_y = 1;
	}

	//Search horizontally
	for (int i = WorldSize.w/2; i < WorldSize.w; i++){
		if (getWorldBit(i, 20) == 1){
			right_x = i - 1;
			right_y = 20;
			found_right = true;
			break;
		}
	}
	if (!found_right){
		right_x = WorldSize.w-1;
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


			if (getWorldBit( prev_x, prev_y+1) == 0){ //if white, find in CCW
				if (prev_x == 1){
					left_edge.push(prev_x, prev_y+1);
				} else {
					for (int i = 0; i < 8; i++){
						if(getWorldBit(prev_x+dx[i], prev_y+dy[i]) == 1){
							left_edge.push(prev_x+dx[i-1], prev_y+dy[i-1]);
							break;
						}
					}
				}
			} else {
				for (int i = 7; i >= 0; i--){
					if(getWorldBit(prev_x+dx[i], prev_y+dy[i]) == 0){
						if ((i == 6 || i == 5 || i == 7) && prev_x == 1){
							//need to check for trespassing
							int cnt_black = 0;
							for (int j = prev_y; j < WorldSize.h; j++){
								cnt_black += getWorldBit(WorldSize.w/2, j);
							}
							if (cnt_black > 0.5 * (WorldSize.h - prev_y)){
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
		if (left_edge.points.back().second == WorldSize.h - 1){ //the edge reaches the top
			flag_break = true;
		}

		//Check corners
		{
			int CornerCheck = 0;
			int total = 0;
			auto last = left_edge.points.back();
			if (last.first - 3 <= 0 || last.first + 3 > WorldSize.w - 1 || last.second - 3 <= 0 || last.second +3 > WorldSize.h -1){
				continue;
			}
			for (int i = (last.first - 3); i <= (last.first + 3); i++){
				for (int j = (last.second - 3); j <= (last.second + 3); j++){
					CornerCheck += getWorldBit(i, j);
					total++;
				}
			}
			//if in this threshold, consider as corner
			if (CornerCheck > total * TuningVar.corner_min / 100 && CornerCheck < total * TuningVar.corner_max / 100){
				left_corners.push(last.first, last.second);
			}
		}
		} while (left_edge.points.size() <= TuningVar.edge_length && flag_break == false);
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

			if (getWorldBit(prev_x, prev_y+1) == 0){ //if white, find in CW
				if (prev_x == WorldSize.w - 1){
					right_edge.push(prev_x, prev_y+1);
				} else {
					for (int i = 7; i >= 0; i--){
						if(getWorldBit(prev_x+dx[i], prev_y+dy[i]) == 1){
							right_edge.push(prev_x+dx[i+1], prev_y+dy[i+1]);
							break;
						}
					}
				}
			} else {
				for (int i = 0; i < 8; i++){

					if(getWorldBit(prev_x+dx[i], prev_y+dy[i]) == 0){
						if ((i == 2 || i == 1 || i == 3) && prev_x == WorldSize.w-1){
							//need to check for trespassing
							int cnt_black = 0;
							for (int j = prev_y; j < WorldSize.h; j++){
								cnt_black += getWorldBit(WorldSize.w/2, j);
							}
							if (cnt_black > 0.5 * (WorldSize.h - prev_y)){
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

			if (right_edge.points.back() == left_edge.points.back()){ //two edges meet
				for (int i = 0; i < 10; i++){ //discard last 10 points
					right_edge.points.pop_back();
					left_edge.points.pop_back();
				}
				flag_break = true;
			}

			if (right_edge.points.back().second == WorldSize.h - 1){ //the edge reaches the top
				flag_break = true;
			}

			//Check corners
			{
				int CornerCheck = 0;
				int total = 0;
				auto last = right_edge.points.back();
				if (last.first - 3 <= 0 || last.first + 3 > WorldSize.w - 1 || last.second - 3 <= 0 || last.second +3 > WorldSize.h -1){
					continue;
				}
				for (int i = max(0, last.first - 3); i <= min(WorldSize.w-1, last.first + 3); i++){
					for (int j = max(0, last.second - 3); j <= min(WorldSize.h-1, last.second + 3); j++){
						CornerCheck += getWorldBit(i, j);
						total++;
					}
				}

				//if in this threshold, consider as corner
				if (CornerCheck > total * TuningVar.corner_min / 100 && CornerCheck < total * TuningVar.corner_max / 100){
					right_corners.push(last.first, last.second);
				}
			}
		} while (right_edge.points.size() <= TuningVar.edge_length && flag_break == false);
	}
	return true;
}

/**
 * @brief Print edges
 */
void PrintEdge(Edges path, uint16_t color){
	for (auto&& entry : path.points){
		pLcd->SetRegion(Lcd::Rect(entry.first, WorldSize.h - entry.second - 1, 2, 2));
		pLcd->FillColor(color);
	}

}

/**
 * @brief Print corners
 */
void PrintCorner(Corners corners, uint16_t color){
	for (auto&& entry : corners.points){
		pLcd->SetRegion(Lcd::Rect(entry.first, WorldSize.h - entry.second - 1, 4, 4));
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
 *  - If in a range of y such that there exists no left edge: translate right edge towards the left
 *  - If in a range of y such that there exists no right edge: translate left edge towards the right
 *  - Otherwise, take the average of x as the the path
 *  - XXX: The path may be broken as different methods are used to plot the path
 * 2. TODO (mcreng): Naive psuedo-optimal path ("Curve fitting")
 * Points to take:
 * 1. Current position (width/2, 0)
 * 2. Start/End points of shifted curve due to LEFT_NULL or RIGHT_NULL
 * 3. Under no LEFT_NULL and RIGHT_NULL, the midpt's midpt
 */
void GenPath(){
	int left_size = left_edge.size();
	int right_size = right_edge.size();

	path.points.clear();

	if (!left_size && !right_size){ //simple validity check
		return;
	}

	if (left_size < right_size){
		for (int i = 0; i < right_edge.size(); i++){
			auto curr_left = left_edge.points[(left_size * i) / right_size];
			auto curr_right = right_edge.points[i];
			int shift_left_null = 0;
			int shift_right_null = 0;
			TranslateType translate_flag = TranslateType::kNone;

			if (curr_left.first == 0 && translate_flag == TranslateType::kNone){
				translate_flag = TranslateType::kLeftNull;
				shift_left_null = right_edge.points[i].first / 2;
			} else if (curr_right.first == 0 && translate_flag == TranslateType::kNone){
				translate_flag = TranslateType::kRightNull;
				shift_right_null = (WorldSize.w - left_edge.points[i].first) / 2;
			//^^^ Start Translation ^^^
			//vvv  Start Averaging  vvv
			} else if (curr_left.first != 0 && translate_flag != TranslateType::kNone){
				translate_flag = TranslateType::kNone;
			} else if (curr_right.first != 0 && translate_flag != TranslateType::kNone){
				translate_flag = TranslateType::kNone;
			}

			//if translate
			if (translate_flag == TranslateType::kLeftNull){
				path.push(curr_right.first - shift_left_null, curr_right.second);
			} else if (translate_flag == TranslateType::kRightNull){
				path.push(curr_left.first + shift_right_null, curr_left.second);
			} else {
			//if average
				int temp_x = (curr_left.first + curr_right.first) / 2;
				int temp_y = (curr_left.second + curr_right.second) / 2;
				path.push(temp_x, temp_y);
			}
		}
	} else {
		for (int i = 0; i < left_edge.size(); i++){
			auto curr_left = left_edge.points[i];
			auto curr_right = right_edge.points[(right_size * i) / left_size];
			int shift_left_null = 0;
			int shift_right_null = 0;
			TranslateType translate_flag = TranslateType::kNone;

			if (curr_left.first == 0 && translate_flag == TranslateType::kNone){
				translate_flag = TranslateType::kLeftNull;
				shift_left_null = right_edge.points[i].first / 2;
			} else if (curr_right.first == 0 && translate_flag == TranslateType::kNone){
				translate_flag = TranslateType::kRightNull;
				shift_right_null = (WorldSize.w - left_edge.points[i].first) / 2;
			//^^^ Start Translation ^^^
			//vvv  Start Averaging  vvv
			} else if (curr_left.first != 0 && translate_flag != TranslateType::kNone){
				translate_flag = TranslateType::kNone;
			} else if (curr_right.first != 0 && translate_flag != TranslateType::kNone){
				translate_flag = TranslateType::kNone;
			}

			//if translate
			if (translate_flag == TranslateType::kLeftNull){
				path.push(curr_right.first - shift_left_null, curr_right.second);
			} else if (translate_flag == TranslateType::kRightNull){
				path.push(curr_left.first + shift_right_null, curr_left.second);
			} else {
			//if average
				int temp_x = (curr_left.first + curr_right.first) / 2;
				int temp_y = (curr_left.second + curr_right.second) / 2;
				path.push(temp_x, temp_y);
			}

		}
	}

}

//turning bounded servo degree
void Turn(int degree){
	degree=max(pServoBounds->kRightBound,min(degree,pServoBounds->kLeftBound));
	pServo->SetDegree(degree);
}

//calculate the error in path
void Interprete(){
	int error=0, sum=0,degree=0;
	for (std::pair<int,int> point : path.points){
		if(sum>20) break;
		error+=(point.first-WorldSize.w/2);//*(point.second+24);
		sum++;
	}

	char buff[10];
	sprintf(buff,"%d , %d",error,degree);
	pLcd->SetRegion(Lcd::Rect(0,0,100,15));
	pWriter->WriteBuffer(buff,10);

	//Turn(degree);
}





void main(CarManager::Car c){
	car = c;
	servo_bounds = car == CarManager::Car::kCar1 ? CarManager::kBoundsCar1 : CarManager::kBoundsCar2;

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
	std::unique_ptr<k60::Ov7725> camera(new k60::Ov7725(cameraConfig));
	pCamera = std::move(camera);
	pCamera->Start();

	FutabaS3010::Config ConfigServo;
	ConfigServo.id = 0;
	FutabaS3010 servo(ConfigServo);
//	std::unique_ptr<FutabaS3010> pServo(new FutabaS3010(ConfigServo));
	pServo = &servo;

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

//	std::unique_ptr<util::MpcDual> pMpc(new util::MpcDual(&motor0, &motor1, &encoder0, &encoder1));
	util::MpcDual mpc(&motor0, &motor1, &encoder0, &encoder1);
	util::MpcDual* pMpc = &mpc;


	k60::JyMcuBt106::Config ConfigBT;
	ConfigBT.id = 0;
	ConfigBT.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	k60::JyMcuBt106 bt(ConfigBT);

	St7735r::Config lcdConfig;
	lcdConfig.is_revert = true;
	St7735r lcd(lcdConfig);
	pLcd = &lcd;

	LcdTypewriter::Config writerConfig;
	writerConfig.lcd = pLcd;
	LcdTypewriter writer(writerConfig);
	pWriter = &writer;

	Joystick::Config joystick_config;
	joystick_config.id = 0;
	joystick_config.is_active_low = true;
	Joystick joystick(joystick_config);

	DebugConsole console(&joystick,pLcd, &writer);


	/*
	CarManager::Config ConfigMgr;
	ConfigMgr.servo = std::move(pServo);
	ConfigMgr.epc = std::move(pMpc);
	CarManager::Init(std::move(ConfigMgr));
*/
	Timer::TimerInt time_img = 0;

	//Servo test

	pServo->SetDegree(servo_bounds.kLeftBound);
	System::DelayMs(1000);
	pServo->SetDegree(servo_bounds.kRightBound);
	System::DelayMs(1000);
	pServo->SetDegree(servo_bounds.kCenter);
	System::DelayMs(1000);

	/*while(true){


		if(System::Time()%50==0&&pCamera->IsAvailable()){

			Byte buffer[128*20];
			CameraBuf = pCamera->LockBuffer();
			PrintWorldImage();
			//pLcd->SetRegion(Lcd::Rect(0,0,128,160));
			//pLcd->FillBits(Lcd::kBlack,Lcd::kWhite,wholeImage(buffer),128*160);
			pCamera->UnlockBuffer();
		}
	}*/

	/*
		DebugConsole::Item item("set servo");
		item.setValuePtr(&servo_degree);
		//item.setListener(SELECT,&updateServo);
		item.setListener(DOWN_LEFT,&subServo);
		item.setListener(LONG_LEFT,&subServo);
		item.setListener(DOWN_RIGHT,&addServo);
		item.setListener(LONG_RIGHT,&addServo);
		console.pushItem(item);
		console.enterDebug();
		*/

	//pMpc->SetTargetSpeed(100);
	while (true){
		while (time_img != System::Time()){
			time_img = System::Time();

			if (time_img % 100 == 0){
				//pMpc->UpdateEncoder();
				Capture(); //Capture until two base points are identified
				//PrintImage(); //Print LCD
				PrintWorldImage();
				FindEdges(); //Find edges
				PrintEdge(left_edge, Lcd::kRed); //Print left_edge
				PrintEdge(right_edge, Lcd::kBlue); //Print right_edge
				PrintCorner(left_corners, Lcd::kPurple); //Print left_corner
				PrintCorner(right_corners, Lcd::kPurple); //Print right_corner
//				CarManager::Feature feature = IdentifyFeat(); //Identify feature
				GenPath(); //Generate path
				Interprete();
				PrintEdge(path, Lcd::kGreen); //Print path
				led0.Switch(); //heart beat
			}
		}

	}

}
}
} // namespace algorithm::optimal
