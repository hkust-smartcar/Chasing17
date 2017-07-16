/*
 * optimal.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Peter Tse (mcreng), Dipsy Wong, King Huang (XUHUAKing), Lee Chun Hei (LeeChunHei)
 *
 * Optimal Path Algorithm CPP File
 *
 */

#include "algorithm/optimal_car1.h"

#include <cmath>
#include <cstring>
#include <cstdio>
#include <sstream>
#include <vector>

#include "libsc/dir_motor.h"
#include "libsc/dir_encoder.h"
#include "libsc/futaba_s3010.h"
#include "libsc/joystick.h"
#include "libsc/lcd_console.h"
#include "libsc/led.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libsc/k60/ov7725.h"
#include "libutil/misc.h"
#include "libutil/incremental_pid_controller.h"

#include "bluetooth.h"
#include "car_manager.h"
#include "debug_console.h"
#include "algorithm/worldview/car1.h"
#include "util/util.h"
#include "fc_yy_us_v4.h"

typedef CarManager::Feature Feature;
typedef CarManager::ImageSize ImageSize;
typedef CarManager::ServoBounds ServoBounds;
typedef CarManager::ObstaclePos ObstaclePos;
typedef CarManager::PidSet PidSet;

using libsc::DirMotor;
using libsc::DirEncoder;
using libsc::FutabaS3010;
using libsc::Joystick;
using libsc::Lcd;
using libsc::LcdTypewriter;
using libsc::Led;
using libsc::St7735r;
using libsc::System;
using libsc::Timer;
using libsc::k60::JyMcuBt106;
using libsc::k60::Ov7725;
using libsc::k60::Ov7725Configurator;

using namespace libutil;

namespace algorithm {
namespace optimal {
namespace car1 {

const PidSet kStablePid = {
    // name
    "c1_stable",

    // servo values
    {0.80, 0, 0.010},   // ServoStraight
    {1.15, 0, 0.000},   // ServoNormal
    {1.30, 0, 0.000},   // ServoRoundabout
    {1.20, 0, 0.000},   // ServoSharpTurn

    // motor values
    150,                // SpeedStraight
    100,                // SpeedNormal
     90,                // SpeedRoundabout
     90,                // SpeedSharpTurn
    100                 // SpeedSlow
};

namespace TuningVar { //tuning var declaration
bool show_algo_time = false;
bool single_car_testing = false;// only for car2 roundabout detection
bool overtake_mode = true; // true: overtake with communication, false: no overtake WITH communication
bool obstacle_mode = true; // true: handle obstacle with communication, false: cancel obstacle handler
bool obsta_overtake_mode = false;

uint16_t starting_y = 13; //the starting y for edge detection
uint16_t edge_length = 159; //max length for an edge
uint16_t edge_hor_search_max = 4; //max for horizontal search of edge if next edge point cannot be found
uint16_t edge_min_worldview_bound_check = 30; //min for worldview bound check in edge finding
uint16_t corner_range = 7; //the square for detection would be in size corener_range*2+1
float corner_height_ratio = 2.9; //the max height for detection would be WorldSize.h/corner_height_ratio
uint16_t corner_min = 16, corner_max = 32; //threshold (in %) for corner detection
uint16_t min_corners_dist = 7; // Manhattan dist threshold for consecutive corners
uint16_t min_edges_dist = 7; // Manhattan dist threshold for edges
uint16_t track_width_threshold = 900; //track width threshold for consideration of sudden change (square)
uint16_t track_width_change_threshold = 350; //track width change threshold for consideration of sudden change
uint16_t testDist = 35; // The distance from which the image pixel should be tested and identify feature
uint16_t slowDownDist = 100; // the distance from which the image pixel should be tested and know whether it should slow down in advance
uint16_t straight_line_threshold = 45; // The threshold num. of equal width for straight line detection
uint16_t action_distance = 27; // The condition in which the car start handling this feature when meeting it
libsc::Timer::TimerInt feature_inside_time = 350; // freezing time for feature extraction, the time for entering the entrance
uint16_t cross_cal_start_num = 80;
uint16_t cross_cal_ratio = 80; //Look forward @cross_cal_start_num - encoder_total/@cross_cal_ratio to determine path
uint16_t general_cal_num = 20; //The num of path points considered for servo angle decision except crossing
uint16_t cross_encoder_count = 4000; // The hardcoded encoder count that car must reach in crossroad
uint16_t round_enter_offset = 15;
uint16_t min_dist_meet_crossing = 30;
uint16_t roundroad_min_size = 30; // When the edge is broken in roundabout, find until this threshold
uint16_t exit_action_dist = 35; // double check to avoid corner's sudden disappear inside roundabout
uint16_t roundabout_offset = 15; // half of road width
uint16_t round_exit_offset = 20;
uint16_t round_encoder_count = 2600;
uint16_t roundExit_encoder_count = 3700;
uint16_t obstacle_encoder_count = 5000;
uint16_t front_obstacle_overtake_encoder_count = 4000;
uint16_t back_obstacle_overtake_encoder_count = 5000;
int32_t roundabout_shortest_flag = 0b00011; //1 means turn left, 0 means turn right. Reading from left to right
int32_t roundabout_overtake_flag = 0b11111;
uint16_t nearest_corner_threshold = 128/2;
uint16_t overtake_interval_time = 1000;

// servo pid values
float servo_straight_kp = kStablePid.ServoStraight.kP;
float servo_straight_kd = kStablePid.ServoStraight.kD;
float servo_normal_kp = kStablePid.ServoNormal.kP;
float servo_normal_kd = kStablePid.ServoNormal.kD;
float servo_roundabout_kp = kStablePid.ServoRoundabout.kP;
float servo_roundabout_kd = kStablePid.ServoRoundabout.kD;
float servo_sharp_turn_kp = kStablePid.ServoSharpTurn.kP;
float servo_sharp_turn_kd = kStablePid.ServoSharpTurn.kD;


// target speed values
uint16_t targetSpeed_straight = kStablePid.SpeedStraight;
uint16_t targetSpeed_normal = kStablePid.SpeedNormal;//normal turning
uint16_t targetSpeed_round = kStablePid.SpeedRound;
uint16_t targetSpeed_sharp_turn = kStablePid.SpeedSharpTurn;
uint16_t targetSpeed_slow = kStablePid.SpeedSlow;

}  // namespace TuningVar


namespace {
//BT listener
std::string inputStr;
bool tune = false;
std::vector<double> constVector;

Edges left_edge;
Edges right_edge;
Edges path;
Corners left_corners;
Corners right_corners;
uint16_t start_y; //For crossing, store the last start point coordinate
uint16_t start_x;
uint16_t prev_corner_x; //store the latest corner coordinate appears last time during roundabout
uint16_t prev_corner_y;

/*FOR STARTING LINE*/
bool hadStoppingLine = false;
bool prevStoppingLine = false;

/*FOR OVERTAKING*/
bool is_front_car = true;
bool stop_before_roundexit = true;
bool overtake;

/*FOR OBSTACLE*/
ObstaclePos obsta_status = ObstaclePos::kNull;
bool sendFlag = true;
int obstacle_cnt = 0;// track the num of obstacle and cancel obstacle detection after once
bool stop_obsta_overtake = true;
int obsta_overtake_status = 0; // 0: not obstacle overtake 1: the previous obstacle is on the left 2: the previous obstacle is on the right
int encoder_total_obstacle_overtake = 0;


bool need_slow_down = false;
bool run =true;//for bluetooth stopping
bool debug = true;
bool is_straight_line = false;
bool exit_round_ready = false; // A flag storing corner status inside roundabout
int roundaboutStatus = 0; // 0: Before 1: Detected 2: Inside (After one corner)
int crossingStatus = 0; // 0: Before 1: Detected/Inside
int roundaboutExitStatus = 0; //0: Before 1: Detected/Inside Exit of Roundabout
uint16_t prev_track_width = 0;
int encoder_total_cross = 0; //for crossroad
int encoder_total_round = 0; // for roundabout
int encoder_total_exit = 0;
int encoder_total_obstacle = 0;
int roundabout_cnt = 0; // count the roundabout
//Timer::TimerInt feature_start_time;
std::pair<int, int> carMid {74, 0};
int roundabout_nearest_corner_cnt_left = pow(TuningVar::corner_range * 2 + 1, 2); // for finding the nearest corner point for roundabout
int roundabout_nearest_corner_cnt_right = pow(TuningVar::corner_range * 2 + 1, 2);
std::pair<int, int> roundabout_nearest_corner_left{0, 0};
std::pair<int, int> roundabout_nearest_corner_right{0, 0};

int prev_servo_error = 0;
int curr_enc_val_left = 0;
int curr_enc_val_right = 0;

const Byte* CameraBuf;

//pointers
std::unique_ptr<Ov7725> spCamera = nullptr;
St7735r* pLcd = nullptr;
Led* pLed3 = nullptr;
LcdTypewriter* pWriter = nullptr;
FutabaS3010* pServo = nullptr;
//JyMcuBt106* pBTovertake = nullptr;
BTComm* pBT = nullptr;
DirEncoder* pEncoder0 = nullptr;
DirEncoder* pEncoder1 = nullptr;
DirMotor* pMotor0 = nullptr;
DirMotor* pMotor1 = nullptr;
IncrementalPidController<float, float>* pid_left_p = nullptr;
IncrementalPidController<float, float>* pid_right_p = nullptr;
int CornerCheck_left = 0, CornerCheck_right = 0;


ServoBounds servo_bounds = {1040, 755, 470};
ImageSize CameraSize = {128, 480};
ImageSize WorldSize = {128, 160};

inline constexpr int max(int a, int b) {
	return (a > b) ? a : b;
}
inline constexpr int min(int a, int b) {
	return (a < b) ? a : b;
}

// for edge finding, in CCW dir
const int8_t dx[9] = { 0, -1, -1, -1, 0, 1, 1, 1, 0 };
const int8_t dy[9] = { 1, 1, 0, -1, -1, -1, 0, 1, 1 };

// prototype declarations
int16_t CalcAngleDiff();
void Capture(uint16_t y0 = TuningVar::starting_y);
Feature featureIdent_Corner();
bool FindStoppingLine();
bool FindEdges();
bool FindOneLeftEdge();
bool FindOneRightEdge();
void GenPath(Feature);
bool getWorldBit(int, int);
std::string InflatePidValues();
void PrintCorner(Corners, uint16_t);
void PrintEdge(Edges, uint16_t);
void PrintImage();
void PrintSuddenChangeTrackWidthLocation(uint16_t);
void PrintWorldImage();
int roundabout_shortest(uint32_t a, int pos);
int roundabout_overtake(uint32_t a, int pos);

std::string InflatePidValues() {
	using namespace TuningVar;

	PidSet p;

	switch (CarManager::pid_preset_) {
		case 1:
			p = kStablePid;
		break;
		default:
			return "Custom";
	}

	// inflate the pid values
	servo_straight_kp = p.ServoStraight.kP;
	servo_straight_kd = p.ServoStraight.kD;
	servo_normal_kp = p.ServoNormal.kP;
	servo_normal_kd = p.ServoNormal.kD;
	servo_roundabout_kp = p.ServoRoundabout.kP;
	servo_roundabout_kd = p.ServoRoundabout.kD;
	servo_sharp_turn_kp = p.ServoSharpTurn.kP;
	servo_sharp_turn_kd = p.ServoSharpTurn.kD;

	targetSpeed_straight = p.SpeedStraight;
	targetSpeed_normal = p.SpeedNormal;
	targetSpeed_round = p.SpeedRound;
	targetSpeed_sharp_turn = p.SpeedSharpTurn;
	targetSpeed_slow = p.SpeedSlow;

	return p.name;
}


/*
 * @brief: bluetooth listener for processing tuning
 * */
bool bluetoothListener(const Byte *data, const size_t size) {
	if (data[0] == 'P') {
		//space
		pid_left_p->SetSetpoint(0);
		pid_right_p->SetSetpoint(0);
		run = false;

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


			//			KP   = constVector[0];
			//			TuningVar::angle_div_error = constVector[0];
			//			now_angle = constVector[1];
		}
	}
	//	else if (data[0] == 'a') {
	//		servoPtr->SetDegree(900);
	//	} else if (data[0] == 'd') {
	//		servoPtr->SetDegree(430);
	//	} else if (data[0] == 'A' || data[0] == 'D') {
	//		servoPtr->SetDegree(700);
	//	}
	return 1;
}

/**
 * @brief To fetch filtered bit, 1 = black; 0 = white
 * @param buff Camera buffer
 * @param (x,y): coordinate system in which bottom left corner := (0, 0)
 * @return filtered bit
 * XXX: Suspecting median filter would crash the program occasionally
 */
int getFilteredBit(const Byte* buff, int x, int y) {
	if (x <= 0 || x > CameraSize.w - 1 || y <= 0 || y > CameraSize.h - 1)
		return 1; //-1; // Out of bound = black
	//y = CameraSize.h - 1 - y; // set y = 0 at bottom

	return buff[y * CameraSize.w / 8 + x / 8] >> (7 - (x % 8)) & 1; //buff[y*CameraSize.w/8+x/8] & 0x80>>x%8;

	// Median Filter
	int count = 0, total = 0;

	for (int i = max(0, x - 1); i < min(CameraSize.w - 1, x + 1); i++) {
		for (int j = max(0, y - 1); j < min(CameraSize.h - 1, y + 1); j++) {
			total++;
			count += buff[j * CameraSize.w / 8 + i / 8] >> (7 - (i % 8)) & 1;
		}
	}
	// Median Filter
	return (count > total / 2) ? 1 : 0;
}

//get bit value from camerabuf using camera coordinate system
/**
 * @brief To fetch bit directly from raw data
 */
bool getBit(int i_x, int i_y) {
	if (i_x <= 0 || i_x > CameraSize.w - 1 || i_y <= 0
			|| i_y > CameraSize.h - 1)
		return -1;
	return CameraBuf[i_y * CameraSize.w / 8 + i_x / 8] >> (7 - (i_x % 8)) & 1;
}

/**
 * @brief To fetch filtered bit from worldview corrected data, 1 = black; 0 = white
 * @param (x,y): coordinate system in which bottom left corner := (0, 0)
 * @return world bit
 */
bool getWorldBit(int w_x, int w_y) {
	if (w_x <= 0 || w_x > WorldSize.w - 1 || w_y <= 0 || w_y > WorldSize.h - 1)
		return 1;
	w_y = 160 - w_y;
	int i_x, i_y;
	i_x = worldview::car1::transformMatrix[w_x][w_y][0];
	i_y = worldview::car1::transformMatrix[w_x][w_y][1];
	return getFilteredBit(CameraBuf, i_x, i_y);

}

/**
 * @brief print worldview corrected image
 */
void PrintWorldImage() {
	Byte temp[128 / 8];
	for (int i = 160; i > 0; --i) {
		for (int j = 0; j < 128; j++) {
			temp[j / 8] <<= 1;
			temp[j / 8] += getWorldBit(j, i);
			//WorldBuf[i*128/8+j/8]<<=1;
			//WorldBuf[i*128/8+j/8]+=getWorldBit(j,i);
		}
		pLcd->SetRegion(Lcd::Rect(0, 160 - i, 128, 1));
		pLcd->FillBits(0x0000, 0xFFFF, temp, 128);
		//pLcd->FillColor(getWorldBit(j,i)?Lcd::kBlack:Lcd::kWhite);
	}
	return;
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
void Capture(uint16_t y0) {
	left_edge.points.clear();
	right_edge.points.clear();
	bool found_left = false, found_right = false;
	int left_x = -1, left_y = -1;
	int right_x = -1, right_y = -1;
	CameraBuf = spCamera->LockBuffer();
	spCamera->UnlockBuffer();

	//Search horizontally
	for (int i = carMid.first; i > 0; i--) {
		if (getWorldBit(i, y0) == 1) {
			left_x = i + 1;
			left_y = y0;
			found_left = true;
			break;
		}
	}
	if (!found_left) {
		left_x = 1;
		left_y = y0;
	}

	//Search horizontally
	for (int i = carMid.first; i < WorldSize.w; i++) {
		if (getWorldBit(i, y0) == 1) {
			right_x = i - 1;
			right_y = y0;
			found_right = true;
			break;
		}
	}
	if (!found_right) {
		right_x = WorldSize.w - 1;
		right_y = y0;
	}

	pLed3->Switch();
	left_edge.push(left_x, left_y);
	right_edge.push(right_x, right_y);
}

/**
 * @brief Print raw image to LCD
 */
void PrintImage() {
	pLcd->SetRegion(Lcd::Rect(0, 0, CameraSize.w, CameraSize.h));
	pLcd->FillBits(0x0000, 0xFFFF, CameraBuf, spCamera->GetBufferSize() * 8);
}

int findCorner_old(std::pair<uint16_t, uint16_t> last){
	int cnt = 0;
	for (int i = (last.first - TuningVar::corner_range);
			i <= (last.first + TuningVar::corner_range); i++) {
		for (int j = (last.second - TuningVar::corner_range);
				j <= (last.second + TuningVar::corner_range); j++) {
			cnt += getWorldBit(i, j);
		}
	}
	return cnt;
}

int findCorner_new(int old_cnt, std::pair<uint16_t, uint16_t> last2, std::pair<uint16_t, uint16_t> last){
	int size = TuningVar::corner_range;
	int new_x = last.first, new_y = last.second;
	int old_x = last2.first, old_y = last2.second;
	int dx = new_x - old_x;
	int dy = new_y - old_y;
	int add_v = 0, add_h = 0;
	int sub_v = 0, sub_h = 0;
	if (dx == 1 && dy == 1){
		for (int i = new_x - size; i < new_x + size; i++) add_h += getWorldBit(i, new_y+size);
		for (int j = new_y - size; j <= new_y + size; j++) add_v += getWorldBit(new_x+size, j);
		for (int i = old_x - size+1; i <= old_x + size; i++) sub_h += getWorldBit(i, old_y-size);
		for (int j = old_y - size; j <= old_y + size; j++) sub_v += getWorldBit(old_x-size, j);
		return old_cnt + add_v + add_h - sub_v - sub_h;
	} else if (dx == 1 && dy == 0){
		for (int j = new_y - size; j <= new_y + size; j++) add_v += getWorldBit(new_x+size, j);
		for (int j = old_y - size; j <= old_y + size; j++) sub_v += getWorldBit(old_x-size, j);
		return old_cnt + add_v - sub_v;
	} else if (dx == 1 && dy == -1){
		for (int i = new_x - size; i < new_x + size; i++) add_h += getWorldBit(i, new_y-size);
		for (int j = new_y - size; j <= new_y + size; j++) add_v += getWorldBit(new_x+size, j);
		for (int i = old_x - size; i <= old_x + size; i++) sub_h += getWorldBit(i, old_y+size);
		for (int j = old_y - size; j < old_y + size; j++) sub_v += getWorldBit(old_x-size, j);
		return old_cnt + add_v + add_h - sub_v - sub_h;
	} else if (dx == 0 && dy == 1){
		for (int i = new_x - size; i <= new_x + size; i++) add_h += getWorldBit(i, new_y+size);
		for (int i = old_x - size; i <= old_x + size; i++) sub_h += getWorldBit(i, old_y-size);
		return old_cnt + add_h - sub_h;
	} else if (dx == 0 && dy == -1){
		for (int i = new_x - size; i <= new_x + size; i++) add_h += getWorldBit(i, new_y-size);
		for (int i = old_x - size; i <= old_x + size; i++) sub_h += getWorldBit(i, old_y+size);
		return old_cnt + add_h - sub_h;
	} else if (dx == -1 && dy == 1){
		for (int i = new_x - size; i <= new_x + size; i++) add_h += getWorldBit(i, new_y+size);
		for (int j = new_y - size; j < new_y + size; j++) add_v += getWorldBit(new_x-size, j);
		for (int i = old_x - size; i < old_x + size; i++) sub_h += getWorldBit(i, old_y-size);
		for (int j = old_y - size; j <= old_y + size; j++) sub_v += getWorldBit(old_x+size, j);
		return old_cnt + add_v + add_h - sub_v - sub_h;
	} else if (dx == -1 && dy == 0){
		for (int j = new_y - size; j <= new_y + size; j++) add_v += getWorldBit(new_x-size, j);
		for (int j = old_y - size; j <= old_y + size; j++) sub_v += getWorldBit(old_x+size, j);
		return old_cnt + add_v - sub_v;
	} else if (dx == -1 && dy == -1){
		for (int i = new_x - size+1; i <= new_x + size; i++) add_h += getWorldBit(i, new_y-size);
		for (int j = new_y - size; j <= new_y + size; j++) add_v += getWorldBit(new_x-size, j);
		for (int i = old_x - size; i <= old_x + size; i++) sub_h += getWorldBit(i, old_y+size);
		for (int j = old_y - size; j < old_y + size; j++) sub_v += getWorldBit(old_x+size, j);
		return old_cnt + add_v + add_h - sub_v - sub_h;
	} else if (dx == 0 && dy == 0) return old_cnt;
	  else return findCorner_old(last);
}

bool FindOneLeftEdge() {
	if (left_edge.size() == 0) return false;
	uint16_t prev_x = left_edge.points.back().first;
	uint16_t prev_y = left_edge.points.back().second;
	uint16_t prev_size = left_edge.points.size();

	if (getWorldBit(prev_x, prev_y + 1) == 0) { //if white, find in CCW until black
		if (prev_x == 1) { //aligned at left bound, finds upwards
			left_edge.push(prev_x, prev_y + 1);
		} else {
			for (int i = 1; i < 8; i++) {
				if (getWorldBit(prev_x + dx[i], prev_y + dy[i]) == 1) { //if black
					left_edge.push(prev_x + dx[i - 1], prev_y + dy[i - 1]); //consider last point edge
					goto endleftsearch;
				}
			}
			//if still couldnt find next point, try search towards left
			for (int i = prev_x;
					i > max(prev_x - TuningVar::edge_hor_search_max, 1); i--) {
				if (getWorldBit(i, prev_y) == 1) {
					left_edge.push(i + 1, prev_y);
					break;
				}
			}
		}
	} else { //if black, find in CW until white
		for (int i = 7; i > 0; i--) {
			if (getWorldBit(prev_x + dx[i], prev_y + dy[i]) == 0) { //if white
				left_edge.push(prev_x + dx[i], prev_y + dy[i]);
				goto endleftsearch;
			}
		}
		//if still couldnt find next point, try search towards right
		for (int i = prev_x;
				i
				< min(prev_x + TuningVar::edge_hor_search_max,
						WorldSize.w - 1); i++) {
			if (getWorldBit(i, prev_y) == 1) {
				left_edge.push(i - 1, prev_y);
				break;
			}
		}

	}

	endleftsearch:
	if (left_edge.points.size() == prev_size)
		return false; //unchanged size
	if (left_edge.points.back()
			== left_edge.points.at(left_edge.points.size() - 2)) { //backtrack
		left_edge.points.pop_back();
		return false;
	}
	if (left_edge.points.back().second == WorldSize.h - 1)
		return false; //reaches top
	if (left_edge.points.back().first == 1)
		return false; //reaches left
	if (left_edge.points.back().first == WorldSize.w - 1)
		return false; //reaches right


	auto last = left_edge.points.back();
	if (last.first - TuningVar::corner_range <= 0
			|| last.first + TuningVar::corner_range > WorldSize.w - 1
			|| last.second - TuningVar::corner_range <= 0
			|| last.second + TuningVar::corner_range > WorldSize.h - 1)
		return true;
	if (left_edge.points.size() == 2) CornerCheck_left = findCorner_old(left_edge.points.back());
	else CornerCheck_left = findCorner_new(CornerCheck_left, left_edge.points[left_edge.points.size()-2],left_edge.points.back());
	int total = pow(TuningVar::corner_range*2+1,2);
	//find corners
	if (left_edge.points.back().second
			<= WorldSize.h / TuningVar::corner_height_ratio && left_corners.size() < 40) {

		//if in this threshold, consider as corner
		if (CornerCheck_left > total * TuningVar::corner_min / 100
				&& CornerCheck_left < total * TuningVar::corner_max / 100) {
			if (abs(last.first - left_corners.back().first)
					+ abs(last.second - left_corners.back().second)
					<= TuningVar::min_corners_dist) { //discard if too close
				return true;
			}
			left_corners.push_back({last.first, last.second});

		}
	}

	//check if the point is the point closest to corners
	if (CornerCheck_left < roundabout_nearest_corner_cnt_left && last.second <= TuningVar::nearest_corner_threshold) {
		roundabout_nearest_corner_cnt_left = CornerCheck_left;
		roundabout_nearest_corner_left = last;
	}
	return true;
}

bool FindOneRightEdge() {
	if (right_edge.size() == 0) return false;
	uint16_t prev_x = right_edge.points.back().first;
	uint16_t prev_y = right_edge.points.back().second;
	uint16_t prev_size = right_edge.points.size();

	if (getWorldBit(prev_x, prev_y + 1) == 0) { //if white, find in CW until black
		if (prev_x == WorldSize.w - 1) { //if align with left bound
			right_edge.push(prev_x, prev_y + 1);
		} else {
			for (int i = 7; i > 0; i--) {
				if (getWorldBit(prev_x + dx[i], prev_y + dy[i]) == 1) { //if white
					right_edge.push(prev_x + dx[i + 1], prev_y + dy[i + 1]); //consider the last point
					goto endrightsearch;
				}
			}
			//if still couldnt find next point, try search towards right
			for (int i = prev_x;
					i
					< min(prev_x + TuningVar::edge_hor_search_max,
							WorldSize.w - 1); i++) {
				if (getWorldBit(i, prev_y) == 1) {
					right_edge.push(i - 1, prev_y);
					break;
				}
			}
		}
	} else { //if white find in CCW until white
		for (int i = 1; i < 8; i++) {
			if (getWorldBit(prev_x + dx[i], prev_y + dy[i]) == 0) { //if black
				right_edge.push(prev_x + dx[i], prev_y + dy[i]);
				goto endrightsearch;
			}
		}
		//if still couldnt find next point, try search towards left
		for (int i = prev_x; i > max(prev_x - TuningVar::edge_hor_search_max, 1);
				i--) {
			if (getWorldBit(i, prev_y) == 1) {
				right_edge.push(i + 1, prev_y);
				break;
			}
		}
	}

	endrightsearch:
	if (right_edge.points.size() == prev_size)
		return false; //unchaged size
	if (right_edge.points.back()
			== right_edge.points.at(right_edge.points.size() - 2)) { //backtrack
		right_edge.points.pop_back();
		return false;
	}
	if (right_edge.points.back().second == WorldSize.h - 1)
		return false; //reaches top
	if (right_edge.points.back().first == 1)
		return false; //reaches left
	if (right_edge.points.back().first == WorldSize.w - 1)
		return false; //reaches right

	auto last = right_edge.points.back();
	if (last.first - TuningVar::corner_range <= 0
			|| last.first + TuningVar::corner_range > WorldSize.w - 1
			|| last.second - TuningVar::corner_range <= 0
			|| last.second + TuningVar::corner_range > WorldSize.h - 1)
		return true;
	if (right_edge.points.size() == 2) CornerCheck_right = findCorner_old(right_edge.points.back());
	else CornerCheck_right = findCorner_new(CornerCheck_right, right_edge.points[right_edge.points.size()-2],right_edge.points.back());
	int total = pow(TuningVar::corner_range*2+1,2);
	//find corners
	if (right_edge.points.back().second
			<= WorldSize.h / TuningVar::corner_height_ratio && right_corners.size() < 40) {

		//if in this threshold, consider as corner
		if (CornerCheck_right > total * TuningVar::corner_min / 100
				&& CornerCheck_right < total * TuningVar::corner_max / 100) {
			if (abs(last.first - right_corners.back().first)
					+ abs(last.second - right_corners.back().second)
					<= TuningVar::min_corners_dist) { //discard if too close
				return true;
			}
			right_corners.push_back({last.first, last.second});

		}

	}

	//check if the point is the point closest to corners
	if (CornerCheck_right < roundabout_nearest_corner_cnt_right && last.second <= TuningVar::nearest_corner_threshold) {
		roundabout_nearest_corner_cnt_right = CornerCheck_right;
		roundabout_nearest_corner_right = last;
	}

	return true;
}

/**
 * @brief Find edges
 * - Find one in left edge then one in right edge, alternatively
 * - Uses FindOneLeftEdge() and FindOneRightEdge()
 *   * Considers the north of current edge
 *   * Uses several filters
 *   * Includes corner detection
 *   * May refer to comments in above functions
 * - Stops if two edges are getting close
 */
bool FindEdges() {
	is_straight_line = false;
	left_corners.clear();
	right_corners.clear();
	bool flag_break_left = left_edge.points.size() == 0;
	bool flag_break_right = right_edge.points.size() == 0;
	uint16_t staright_line_edge_count = 0; // Track the num. of equal width
	roundabout_nearest_corner_cnt_left = pow(TuningVar::corner_range * 2 + 1, 2);
	roundabout_nearest_corner_cnt_right = pow(TuningVar::corner_range * 2 + 1, 2);
	while (left_edge.points.size() <= 50 && right_edge.points.size() <= 50
			&& (!flag_break_left || !flag_break_right)) {
		if (!flag_break_left)
			flag_break_left = !FindOneLeftEdge();
		if (!flag_break_right)
			flag_break_right = !FindOneRightEdge();

		//check if have corners
		if (left_corners.size() > 0) flag_break_left = true;
		if (right_corners.size() > 0) flag_break_right = true;

		//check if two edges are close
		uint16_t r_back_x = right_edge.points.back().first;
		uint16_t r_back_y = right_edge.points.back().second;
		uint16_t l_back_x = left_edge.points.back().first;
		uint16_t l_back_y = left_edge.points.back().second;
		bool status = (r_back_x == l_back_x - 1 || r_back_x == l_back_x
				|| r_back_x == l_back_x + 1)
        								&& (r_back_y == l_back_y - 1 || r_back_y == l_back_y
        										|| r_back_y == l_back_y + 1);
		if (abs(r_back_x - l_back_x) + abs(l_back_y - r_back_y)
				< TuningVar::min_edges_dist) { //two edges meet
			for (int i = 0;
					i
					< min(10,
							min(right_edge.size() / 2,
									left_edge.size() / 2)); i++) { //discard last 10 points
				right_edge.points.pop_back();
				left_edge.points.pop_back();
			}
			flag_break_left = flag_break_right = true;
		}

		// edges reach worldview boundaries

		if (worldview::car1::transformMatrix[min(
				left_edge.points.back().first + 1, WorldSize.w - 1)][WorldSize.h
																	  - left_edge.points.back().second][0] == -1) {
			flag_break_left = true;
		}
		if (worldview::car1::transformMatrix[max(
				left_edge.points.back().first - 1, 1)][WorldSize.h
													   - left_edge.points.back().second][0] == -1) {
			flag_break_left = true;
		}
		if (right_edge.points.back().second
				> TuningVar::edge_min_worldview_bound_check) {
			if (worldview::car1::transformMatrix[min(
					right_edge.points.back().first + 1, WorldSize.w - 1)][WorldSize.h
																		   - right_edge.points.back().second][0] == -1) {
				flag_break_right = true;
			}
			if (worldview::car1::transformMatrix[max(
					right_edge.points.back().first - 1, 1)][WorldSize.h
															- right_edge.points.back().second][0] == -1) {
				flag_break_right = true;
			}
		}
	}
	//check for obstacle
		//3 cases: 1. y=30~35 black, 2. size unchanged, 3. corner
		if (obsta_status == ObstaclePos::kNull){
			//case 1
			if (left_edge.size() >= 46 && getWorldBit(left_edge.points[45].first-1, left_edge.points[45].second) == 1){ //only for left of left edge is black
				int cnt_black = 0;
				for (int i = 40; i < 45; i++) cnt_black += getWorldBit(left_edge.points[i].first+7, left_edge.points[i].second);
				if (cnt_black == 5) {
					obsta_status = ObstaclePos::kLeft;
					goto obsta_status_end;
				}
			}
			if (right_edge.size() >= 46 && getWorldBit(right_edge.points[45].first-1, right_edge.points[45].second) == 1){ //only for right of right edge is black
				int cnt_black = 0;
				for (int i = 40; i < 45; i++) cnt_black += getWorldBit(right_edge.points[i].first-7, right_edge.points[i].second);
				if (cnt_black == 5) {
					obsta_status = ObstaclePos::kRight;
					goto obsta_status_end;
				}
			}
			//case 2
			if (getWorldBit(left_edge.points.back().first-1, left_edge.points.back().second) == 1 && !FindOneLeftEdge()){
				int cnt_black = 0;
				for (int i = left_edge.size()-5; i < left_edge.size(); i++) cnt_black += getWorldBit(left_edge.points[i].first+7, left_edge.points[i].second);
				if (cnt_black == 5) {
					obsta_status = ObstaclePos::kLeft;
					goto obsta_status_end;
				}
			} else left_edge.points.pop_back();
			if (getWorldBit(right_edge.points.back().first+1, right_edge.points.back().second) == 1 && !FindOneRightEdge()){
				int cnt_black = 0;
				for (int i = right_edge.size()-5; i < right_edge.size(); i++) cnt_black += getWorldBit(right_edge.points[i].first-7, right_edge.points[i].second);
				if (cnt_black == 5) {
					obsta_status = ObstaclePos::kRight;
					goto obsta_status_end;
				}
			} else right_edge.points.pop_back();
			//case 3
			if (left_corners.size() == 1 && right_corners.size() == 0 && getWorldBit(left_edge.points.back().first-1,left_edge.points.back().second)){
				int cnt_black = 0;
				for (int i = left_edge.size()-5; i < left_edge.size(); i++) cnt_black += getWorldBit(left_edge.points[i].first+7, left_edge.points[i].second);
				if (cnt_black == 5) {
					obsta_status = ObstaclePos::kLeft;
					goto obsta_status_end;
				}
			}
			if (right_corners.size() == 1 && left_corners.size() == 0 && getWorldBit(right_edge.points.back().first-1,right_edge.points.back().second)){
				int cnt_black = 0;
				for (int i = right_edge.size()-5; i < right_edge.size(); i++) cnt_black += getWorldBit(right_edge.points[i].first-7, right_edge.points[i].second);
				if (cnt_black == 5) {
					obsta_status = ObstaclePos::kRight;
					goto obsta_status_end;
				}
			}
		}
		obsta_status_end:

	//Straight line judgement
	if (staright_line_edge_count >= TuningVar::straight_line_threshold) {
		is_straight_line = true;
	}
	return true;
}

/**
 * @brief Feature Identification by corners
 *
 * Algorithm:
 * 1. Two corners connected together
 * 2. Perpendicular direction, search points @testDistance away.
 * 3. Black (1) is roundabout, White (0) is crossing
 *
 * @return: Feature: kCrossing, kRound
 * @note: Execute this function after calling FindEdges()
 */
Feature featureIdent_Corner() {
	bool temp_is_front;// don't change the is_front_car
	if (!hadStoppingLine) return Feature::kNormal;
	if(!overtake){
		temp_is_front = false;
		stop_before_roundexit = false;
	}
	else
		temp_is_front = is_front_car;

	// no feature detection during obstacle overtake
	if(obsta_overtake_status != 0){
		return Feature::kNormal;
	}
	//1. Straight line
	if (is_straight_line) {
		//    roundaboutStatus = 0;// Avoid failure of detecting roundabout exit
		return Feature::kStraight;
	}

	/*Only assume the first two corners are useful*/
	if (left_corners.size() > 0 && right_corners.size() > 0) {
		//3. More than two valid corner case
		uint16_t cornerMid_x = (left_corners.front().first
				+ right_corners.front().first) / 2; //corner midpoint x-cor
		uint16_t cornerMid_y = (left_corners.front().second
				+ right_corners.front().second) / 2; //corner midpoint y-cor
		/*For identifying the feature*/
		uint16_t test_y = cornerMid_y + TuningVar::testDist;
		uint16_t test_x =
				(test_y - cornerMid_y) * (right_corners.front().second - left_corners.front().second) / (left_corners.front().first - right_corners.front().first) + cornerMid_x;
		/*FOR DEBUGGING*/
		if (debug) {
			pLcd->SetRegion(Lcd::Rect(test_x, WorldSize.h - test_y - 1, 4, 4));
			pLcd->FillColor(Lcd::kYellow);
		}
		/*END OF DEBUGGING*/
//		bool is_round = false;
//		bool is_cross = false;
//		while (!getWorldBit(test_x, test_y) && (test_y >cornerMid_y)){test_y--;}
//		if(test_y >cornerMid_y){is_round = true;}// have one black
//		if(test_y == cornerMid_y){is_cross = true;}// all white
//		if (is_round && roundaboutStatus == 0
		if (getWorldBit(test_x, test_y)
				&& getWorldBit(test_x + 1, test_y)
				&& getWorldBit(test_x, test_y + 1)
				&& getWorldBit(test_x - 1, test_y)
				&& roundaboutStatus == 0
				&& crossingStatus == 0/*Temporary close*/) {
			//All black
			if (abs(carMid.second - cornerMid_y) < TuningVar::action_distance) {
				if(is_front_car || pBT->getBufferFeature() == Feature::kRoundabout || TuningVar::single_car_testing){// for single car running testing
					pBT->resetFeature();
					encoder_total_round = 0;
					roundaboutStatus = 1; //Detected
		//			feature_start_time = System::Time(); // Mark the startTime of latest enter time
					roundabout_cnt++;
					return Feature::kRoundabout;
				// judge as crossing
				}else{
					encoder_total_cross = 0;
					crossingStatus = 1; //Detected
					//		feature_start_time = System::Time(); // Mark the startTime of latest enter time
					//      roundaboutStatus = 0;// Avoid failure of detecting roundabout exit
					start_y = carMid.second
							+ (TuningVar::cross_cal_start_num
									- encoder_total_cross / TuningVar::cross_cal_ratio);
					start_x = (start_y - (left_corners.front().second + right_corners.front().second) / 2)
		        								  / (left_corners.front().first - right_corners.front().first)
												  * (right_corners.front().second
														  - left_corners.front().second)
														  + (left_corners.front().first
																  + right_corners.front().first) / 2;
					return Feature::kCross;
				}
			}
			else{
				need_slow_down = true;
			}

			//		} else if (is_cross && crossingStatus == 0
		} else if (!getWorldBit(test_x, test_y)
				&& !getWorldBit(test_x + 1, test_y)
				&& !getWorldBit(test_x, test_y + 1)
				&& !getWorldBit(test_x - 1, test_y)
				&& crossingStatus == 0
				&& roundaboutStatus == 0) // avoid double check for crossing when inside the crossing (encoder_total_cross<2500)
		{
			if (abs(carMid.second - cornerMid_y) < TuningVar::action_distance) {
				encoder_total_cross = 0;
				crossingStatus = 1; //Detected
				//		feature_start_time = System::Time(); // Mark the startTime of latest enter time
				//      roundaboutStatus = 0;// Avoid failure of detecting roundabout exit
				start_y = carMid.second
						+ (TuningVar::cross_cal_start_num
								- encoder_total_cross / TuningVar::cross_cal_ratio);
				start_x = (start_y - (left_corners.front().second + right_corners.front().second) / 2)
	        								  / (left_corners.front().first - right_corners.front().first)
											  * (right_corners.front().second
													  - left_corners.front().second)
													  + (left_corners.front().first
															  + right_corners.front().first) / 2;
				return Feature::kCross;
			}
		}
	}
	/*
	 * @Note:
	 * roundaboutStatus: Becomes 1 once detected, becomes 0 after exit
	 * exit_round_ready: Detects 1 corner after completely entering the roundabout
	 * roundaboutExitStatus: Becomes 1 when exit is ready and one corner disappear, becomes 0 after encoderExit is reached
	 * */
	//4. Only one corner case: Only one corner - Exit/Cross/Entering crossing & roundabout
	else if (left_corners.size() > 0
			|| right_corners.size() > 0) {
		/*TODO:Double check to handle only one corner case*/
		if (roundaboutStatus == 0 && crossingStatus == 0) { // avoid double check for crossing when inside the crossing (encoder_total_cross<2500)){ //Not inside roundabout, not Exit of Roundabout when encounter one corner case - CONDITION_1
			//	  //Both sides are break due to -1 - CONDITION_2
			// Only one corner - CONDITION_3
			bool crossing = false;
			//right edge touch right boundary + left corner
			if ((worldview::car1::transformMatrix[min(
					right_edge.points.back().first + 1, WorldSize.w - 1)][WorldSize.h
																		   - right_edge.points.back().second][0] == -1)
					&& left_corners.size() > 0) {
				//Only left corner + close enough
				if (abs(left_corners.front().second - carMid.second)
						<= TuningVar::min_dist_meet_crossing) {
					//push the midpoint of right edge into corner
					right_corners.push_back({
							(right_edge.points.front().first
									+ right_edge.points.back().first) / 2,
									(right_edge.points.front().second
											+ right_edge.points.back().second) / 2});
					crossing = true;
				}
			}
			//left edge touch left boundary)
			if ((worldview::car1::transformMatrix[max(
					left_edge.points.back().first - 1, 1)][WorldSize.h
														   - left_edge.points.back().second][0] == -1)
					&& right_corners.size() > 0) {
				//Only right corner
				if (abs(right_corners.front().second - carMid.second)
						<= TuningVar::min_dist_meet_crossing) {
					left_corners.push_back({
							(left_edge.points.front().first
									+ left_edge.points.back().first) / 2,
									(left_edge.points.front().second
											+ left_edge.points.back().second) / 2});
					crossing = true;
				}
			}

			if (crossing) {
				//Record the start midpoint for searching
				start_y = carMid.second
						+ (TuningVar::cross_cal_start_num
								- encoder_total_cross
								/ TuningVar::cross_cal_ratio);
				start_x = (start_y
						- (left_corners.front().second
								+ right_corners.front().second) / 2)
            								/ (left_corners.front().first
            										- right_corners.front().first)
													* (right_corners.front().second
															- left_corners.front().second)
															+ (left_corners.front().first
																	+ right_corners.front().first) / 2;
				//				pEncoder0->Update();
				crossingStatus = 1; //Detected
				encoder_total_cross = 0;
				return Feature::kCross;
			}
		}
	}
	/*Exit case handling: ready -> exit*///To avoid entrance double check for corner
	if(temp_is_front?!roundabout_shortest(TuningVar::roundabout_shortest_flag, roundabout_cnt - 1):roundabout_shortest(TuningVar::roundabout_shortest_flag, roundabout_cnt - 1)){
		if (left_corners.size() > 0 && roundaboutStatus == 1 && roundaboutExitStatus == 0
				&& abs(encoder_total_round) > TuningVar::round_encoder_count) {
			//		//keep updating until corner disappear
			//		if (left_corners.size() > 0) {
			//			prev_corner_x = left_corners.front().first;
			//			prev_corner_y = left_corners.front().second;
			//		}
			//		if (right_corners.size() > 0) {
			//			prev_corner_x = right_corners.front().first;
			//			prev_corner_y = right_corners.front().second;
			//		}
			need_slow_down = true;// slow down in advance
			exit_round_ready = true; // Detect one corner
		}
	}
	else{
		if (right_corners.size() > 0 && roundaboutStatus == 1 && roundaboutExitStatus == 0
				&& abs(encoder_total_round) > TuningVar::round_encoder_count) {
			need_slow_down = true;// slow down in advance
			exit_round_ready = true; // Detect one corner
		}
	}
	/*FOR DEBUGGING*/
	if (false) {
		//				char temp_1[100];
		//				sprintf(temp_1, "Ycor:%d", abs(roundabout_nearest_corner_right.second - carMid.second));
		//				pLcd->SetRegion(Lcd::Rect(0, 75, 128, 15));
		//				pWriter->WriteString(temp_1);
		pLcd->SetRegion(Lcd::Rect(0,30,128,15));
		exit_round_ready?pWriter->WriteString("ready"):pWriter->WriteString("Not ready");
		pLcd->SetRegion(Lcd::Rect(0,45,128,15));
		roundaboutStatus?pWriter->WriteString("RS = 1"):pWriter->WriteString("RS = 0");
		pLcd->SetRegion(Lcd::Rect(0,56,128,15));
		roundaboutExitStatus?pWriter->WriteString("RSE = 1"):pWriter->WriteString("RSE = 0");
	}
	/*END OF DEBUGGING*/
	//the moment when corner disappears + inside roundabout is Exit /*+ Front is black*/
	bool meet_exit = false;
	if (exit_round_ready && roundaboutStatus == 1
			&& abs(encoder_total_round) > TuningVar::round_encoder_count/*abs(System::Time() - feature_start_time) > TuningVar::feature_inside_time*/) {

		if(temp_is_front?!roundabout_shortest(TuningVar::roundabout_shortest_flag, roundabout_cnt - 1):roundabout_shortest(TuningVar::roundabout_shortest_flag, roundabout_cnt - 1)){
			/*FOR DEBUGGING*/
			if (debug) {
				pLcd->SetRegion(Lcd::Rect(roundabout_nearest_corner_left.first, WorldSize.h - roundabout_nearest_corner_left.second - 1, 4, 4));
				pLcd->FillColor(Lcd::kCyan);
			}
			/*END OF DEBUGGING*/
			// corner disappears && close enough
			meet_exit = abs(roundabout_nearest_corner_left.second - carMid.second) < TuningVar::exit_action_dist;
		} else {
			/*FOR DEBUGGING*/
			if (debug) {
				pLcd->SetRegion(Lcd::Rect(roundabout_nearest_corner_right.first, WorldSize.h - roundabout_nearest_corner_right.second - 1, 4, 4));
				pLcd->FillColor(Lcd::kCyan);
			}
			/*END OF DEBUGGING*/
			// corner disappears && close enough
			meet_exit = abs(roundabout_nearest_corner_right.second - carMid.second) < TuningVar::exit_action_dist;
		}
		//TODO: receive buffer message: only keep moving when receiving exit message
		if(!temp_is_front){//back car
			stop_before_roundexit = false;
			if (meet_exit) {
				//roundaboutStatus = 0;
				exit_round_ready = false;
				//				pEncoder0->Update();
				//				pEncoder1->Update();
				encoder_total_exit = 0;
				roundaboutExitStatus = 1;
				return Feature::kRoundaboutExit;
			}
		}

		else{//front car
			if (meet_exit) {
				//below part can handle the other case: back car has passed, no need to stop (not same as main.cpp)
				// case one: before front car meet exit, back car has finished overtake - no stop
				if(pBT->hasFinishedOvertake() && (System::Time() - pBT->getOvertakeTime())>TuningVar::overtake_interval_time ){
					stop_before_roundexit = false;
					// roundaboutStatus = 0;
					exit_round_ready = false;
					//					pEncoder0->Update();
					//					pEncoder1->Update();
					encoder_total_exit = 0;
					pBT->resetFinishOvertake();
				}
				// case two: when front car meets exit, back car haven't finished overtake
				// case three: when front car meets exit, back car has finished overtake but interval time is not enough
				else{
					encoder_total_exit = 0;//clear history data to avoid immediately judged as "finish exit"
					stop_before_roundexit = true;
				}
				roundaboutExitStatus = 1;
				return Feature::kRoundaboutExit;
			}

		}
		//	  }
	}
	//5. obstacle case
	//front car sends signal when pass half of obstacle
	// reset the obstacle case during overtake
	if(obsta_overtake_status != 0){
		// reset
		obsta_status = ObstaclePos::kNull;
	}

	if (is_front_car && sendFlag && obsta_status != ObstaclePos::kNull && abs(encoder_total_obstacle) >= TuningVar::obstacle_encoder_count/2){
		pBT->sendObstaclePos(obsta_status);
		sendFlag = false;
	}
	//back car keep status only when the front car sends the same feature to himself, and reset the feature after reading the feature
	if(!is_front_car && obsta_status != ObstaclePos::kNull){
		// not the same as front car situation
		if(pBT->getObstaclePos() != obsta_status){
			// reset
			obsta_status = ObstaclePos::kNull;
		}
	}


	//6. Nothing special: Return kNormal and wait for next testing
	return Feature::kNormal;
}

/**
 * @brief Print edges
 */
void PrintEdge(Edges path, uint16_t color) {
	for (auto&& entry : path.points) {
		pLcd->SetRegion(
				Lcd::Rect(entry.first, WorldSize.h - entry.second - 1, 2, 2));
		pLcd->FillColor(color);
	}

}

/**
 * @brief Print corners
 */
void PrintCorner(Corners corners, uint16_t color) {
	for (auto&& entry : corners) {
		pLcd->SetRegion(
				Lcd::Rect(entry.first, WorldSize.h - entry.second - 1, 4, 4));
		pLcd->FillColor(color);
	}
}

/**
 * Path generation
 * 1. kNormal/kStraight: Weighted average path ("Center line")
 *  - If in a range of y such that there exists no left edge: translate right edge towards the left
 *  - If in a range of y such that there exists no right edge: translate left edge towards the right
 *  - Otherwise, take the average of x as the the path
 * 2. kCross: Path = look ahead and search from midpoint
 * 3. kRoundabout:
 *  - Turn left: translate left edge
 *  - Turn right: translate right edge
 * 4. kRoundaboutExit (Exit of roundabout):
 *  - Turn right: translate right edge
 *  - Turn left: translate left edge
 * TODO (mcreng): Naive psuedo-optimal path ("Curve fitting")
 *
 * Points to take:
 * 1. Current position (width/2, 0)
 * 2. Start/End points of shifted curve due to LEFT_NULL or RIGHT_NULL
 * 3. Under no LEFT_NULL and RIGHT_NULL, the midpt's midpt
 */
void GenPath(Feature feature) {
	bool temp_is_front;// don't change the is_front_car
	if(!overtake){
		temp_is_front = false;
		stop_before_roundexit = false;
	}
	else
		temp_is_front = is_front_car;

	int left_size = left_edge.size();
	int right_size = right_edge.size();

	path.points.clear();

	if (!left_size && !right_size) { //simple validity check
		return;
	}
	/* CROSSING PASSING PART - Include kCross handling - Overwrite next normal kCross part
	 * @crossingStatus: for storing the status for a period
	 * @kCross: for opening the "switch"
	 * */
	if (crossingStatus == 1
			&& encoder_total_cross
			>= TuningVar::cross_encoder_count/*abs(System::Time() - feature_start_time) > TuningVar::feature_inside_time*/) { //&& encoder pass crossing
		crossingStatus = 0;
	}
	if (crossingStatus == 1
			&& encoder_total_cross < TuningVar::cross_encoder_count) { //Gen new path by searching midpoint
		//		pEncoder0->Update÷();
		encoder_total_cross += curr_enc_val_left;
		uint16_t new_right_x;
		uint16_t new_left_x;
		// find new right edge
		for (uint16_t i = start_x; i < WorldSize.w; i++) {
			if (getWorldBit(i, start_y) == 1) {
				new_right_x = i;
				break;
			}
		}
		// find new left edge
		for (uint16_t i = start_x; i > 0; i--) {
			if (getWorldBit(i, start_y) == 1) {
				new_left_x = i;
				break;
			}
		}
		/*FOR DEBUGGING*/
		if (debug) {
			pLcd->SetRegion(
					Lcd::Rect((new_left_x + new_right_x) / 2,
							WorldSize.h - start_y - 1, 4, 4));
			pLcd->FillColor(Lcd::kRed);
		}
		/*END OF DEBUGGING*/

		path.push(((new_left_x + new_right_x) / 2 + carMid.first) / 2, start_y); //follow the new midpoint
		//Update start_x and start_y
		start_y = carMid.second
				+ (TuningVar::cross_cal_start_num
						- encoder_total_cross / TuningVar::cross_cal_ratio);
		start_x = (new_left_x + new_right_x) / 2;
		//	path.push(carMid.first,carMid.second);
		return;
	}
	/*END OF CROSSING PASSING PART*/

	/*FOR DEBUGGING*/
	if (false) {
		char temp[100];
		sprintf(temp, "ExitEnc:%d", abs(encoder_total_exit));
		pLcd->SetRegion(Lcd::Rect(0, 0, 128, 15));
		pWriter->WriteString(temp);
		sprintf(temp, "EntEnc:%d", abs(encoder_total_round));
		pLcd->SetRegion(Lcd::Rect(0, 15, 128, 15));
		pWriter->WriteString(temp);
	}
	/*END OF DEBUGGING*/

	/*ROUNDABOUT ENTRANCE PASSING PART*/
	if (roundaboutStatus == 1) { feature = Feature::kRoundabout; }
	if (roundaboutStatus == 1
			&& abs(encoder_total_round) < TuningVar::round_encoder_count/*abs(System::Time() - feature_start_time) < TuningVar::feature_inside_time*/) {
		//		pEncoder0->Update();
		//		pEncoder1->Update();
		encoder_total_round += curr_enc_val_left/*(pEncoder0->GetCount() + pEncoder1->GetCount()) / 2*/;//Because for exit/enter, the car will first left then right which destroy the encoder
		//		feature = Feature::kRoundabout;
	}

	//When exiting the roundabout, keep exit method until completely exit
	if (roundaboutExitStatus == 1
			&& abs(encoder_total_exit) >= TuningVar::roundExit_encoder_count) {
		roundaboutExitStatus = 0;
		roundaboutStatus = 0;
		/*TODO: switch carID, sendBT to another car and set has_exited on the other side to true*/
		if(overtake){
			if(!is_front_car){//Back car
				//switch ID
				is_front_car = true;
				pBT->sendFinishOvertake();
			}
			//switch ID after using offset to exit for original front car
			else if(is_front_car){
				is_front_car = false;
			}
		}
		// update the overtake flag for next roundabout
		if(TuningVar::overtake_mode){
			overtake = roundabout_overtake(TuningVar::roundabout_overtake_flag, roundabout_cnt);
		}
		else{
			overtake = false;
		}
	}
	if (roundaboutExitStatus == 1
			&& abs(encoder_total_exit) < TuningVar::roundExit_encoder_count) {//TODO: Be care of back turning of motor when stop will affect encoder value
		//		pEncoder0->Update();
		//		pEncoder1->Update();
		encoder_total_exit += curr_enc_val_left/*(pEncoder0->GetCount() + pEncoder1->GetCount()) / 2*/;
		feature = Feature::kRoundaboutExit;
	}

	// obstacle case handling + overtake
	if (TuningVar::obstacle_mode){

		// reset the obsta_status when finish and count++
		if(abs(encoder_total_obstacle) >= TuningVar::obstacle_encoder_count && obsta_status == ObstaclePos::kLeft && obsta_overtake_status == 0){
			if(TuningVar::obsta_overtake_mode){
				obsta_overtake_status = 1;
			}
			else{
				sendFlag = true;
				obstacle_cnt++;
			}
			// clear encoder count
			encoder_total_obstacle = 0;
			encoder_total_obstacle_overtake = 0;
			obsta_status = ObstaclePos::kNull;
			pBT->resetObstaclePos();

		}
		else if (abs(encoder_total_obstacle) >= TuningVar::obstacle_encoder_count && obsta_status == ObstaclePos::kRight && obsta_overtake_status == 0){
			if(TuningVar::obsta_overtake_mode){
				obsta_overtake_status = 2;// 2 means obstacle is right
			}
			else{
				sendFlag = true;
				obstacle_cnt++;
			}
			// clear encoder count
			encoder_total_obstacle = 0;
			encoder_total_obstacle_overtake = 0;
			obsta_status = ObstaclePos::kNull;
			pBT->resetObstaclePos();

		}

		/*OVERTAKE PART*/
		// front car stops
		if(is_front_car && TuningVar::obsta_overtake_mode && obsta_overtake_status != 0){
			encoder_total_obstacle_overtake += curr_enc_val_left;
			// parking on the left
			if(obsta_overtake_status == 1){
				if(abs(encoder_total_obstacle_overtake) <= TuningVar::front_obstacle_overtake_encoder_count){
					stop_obsta_overtake = false;
					// follow new path
					/*path offset left*/
					for(int i =0; i < 20; i++) path.push(left_edge.points[i].first + 7, left_edge.points[i].second);
					return;

				}
				else if(stop_obsta_overtake && pBT->hasFinishedObstacleOvertake() && System::Time() - pBT->getObstacleOvertakeTime() >= TuningVar::overtake_interval_time){
					// start the car until receiving message from back car and reset the flag, overtake finished
					stop_obsta_overtake = false;
					is_front_car = false; // switch ID
					obsta_overtake_status = 0;
					sendFlag = true;
					obstacle_cnt++;
					pBT->resetFinishObstacleOvertake();
				}
				else{
					stop_obsta_overtake = true;// set the speed to 0
					// follow new path
					/*path offset left*/
					for(int i =0; i < 20; i++) path.push(left_edge.points[i].first + 7, left_edge.points[i].second);
					return;
				}
			}
			else if(obsta_overtake_status == 2){
				// parking on the right
				if(abs(encoder_total_obstacle_overtake) <= TuningVar::front_obstacle_overtake_encoder_count){
					stop_obsta_overtake = false;
					// follow new path
					/*path offset right*/
					for(int i =0; i < 20; i++) path.push(right_edge.points[i].first - 7, right_edge.points[i].second);
					return;

				}
				else if(stop_obsta_overtake && pBT->hasFinishedObstacleOvertake() && System::Time() - pBT->getObstacleOvertakeTime() >= TuningVar::overtake_interval_time){
					// start the car until receiving message from back car and reset the flag, overtake finished
					stop_obsta_overtake = false;
					is_front_car = false; // switch ID
					obsta_overtake_status = 0;
					sendFlag = true;
					obstacle_cnt++;
					pBT->resetFinishObstacleOvertake();
				}
				else{
					stop_obsta_overtake = true;// set the speed to 0
					// follow new path
					/*path offset right*/
					for(int i =0; i < 20; i++) path.push(right_edge.points[i].first - 7, right_edge.points[i].second);
					return;
				}
			}

		}
		// back car move
		if (!is_front_car && TuningVar::obsta_overtake_mode && obsta_overtake_status != 0){
			encoder_total_obstacle_overtake += curr_enc_val_left;
			// moving on the right
			if(obsta_overtake_status == 1){
				if(abs(encoder_total_obstacle_overtake) <= TuningVar::back_obstacle_overtake_encoder_count){
					// follow new path
					/*path offset right*/
					for(int i =0; i < 20; i++) path.push(right_edge.points[i].first - 7, right_edge.points[i].second);
					return;

				}
				else{
					// send message to front car
					is_front_car = true;
					pBT->sendFinishObstacleOvertake();
					obsta_overtake_status = 0;
					sendFlag = true;
					obstacle_cnt++;
				}
			}
			// moving on the left
			if(obsta_overtake_status == 2){
				if(abs(encoder_total_obstacle_overtake) <= TuningVar::back_obstacle_overtake_encoder_count){
					// follow new path
					/*path offset left*/
					for(int i =0; i < 20; i++) path.push(left_edge.points[i].first + 7, left_edge.points[i].second);
					return;

				}
				else{
					// send message to front car
					is_front_car = true;
					pBT->sendFinishObstacleOvertake();
					obsta_overtake_status = 0;
					sendFlag = true;
					obstacle_cnt++;
				}
			}
		}


		if(obsta_status == ObstaclePos::kLeft && abs(encoder_total_obstacle) < TuningVar::obstacle_encoder_count){
			encoder_total_obstacle += curr_enc_val_left;
			/*path offset right*/
			for(int i =0; i < 20; i++) path.push(right_edge.points[i].first - 8, right_edge.points[i].second);
			return;
		}
		else if (obsta_status == ObstaclePos::kRight && abs(encoder_total_obstacle) < TuningVar::obstacle_encoder_count){
			encoder_total_obstacle += curr_enc_val_left;
			/*path offset left*/
			for(int i =0; i < 20; i++) path.push(left_edge.points[i].first + 8, left_edge.points[i].second);
			return;
		}
	}

	/*OTHER CASE*/
	switch (feature) {
	case Feature::kRoundabout: {
		// Turning left at the entrance
		if (temp_is_front?!roundabout_shortest(TuningVar::roundabout_shortest_flag, roundabout_cnt - 1):roundabout_shortest(TuningVar::roundabout_shortest_flag, roundabout_cnt - 1)) {
		  			//ensure the size of left is large enough for turning, size of left will never be 0
			//ensure the size of left is large enough for turning, size of left will never be 0
			while ((left_edge.points.size() < TuningVar::roundroad_min_size) && FindOneLeftEdge()) {}
			//translate right
			for (int i = 0; i < left_edge.points.size(); i++) {
				path.push(left_edge.points[i].first + TuningVar::roundabout_offset, left_edge.points[i].second);
			}

			/*OLD METHOD - Refer to commit before July 2nd*/
		}
		// Turning right at the entrance
		else {
			while ( (right_edge.points.size()<TuningVar::roundroad_min_size) && FindOneRightEdge()) {}
			for(int i=0; i<right_edge.points.size(); i++){
				path.push(right_edge.points[i].first - TuningVar::roundabout_offset , right_edge.points[i].second);
			}
		}
		break;
	}
	case Feature::kRoundaboutExit: {
		// Turning left at the entrance - Turning left at the exit
		if (temp_is_front?!roundabout_shortest(TuningVar::roundabout_shortest_flag, roundabout_cnt - 1):roundabout_shortest(TuningVar::roundabout_shortest_flag, roundabout_cnt - 1)) {
		  			//ensure the size of left is large enough for turning, size of left will never be 0
			//ensure the size of left is large enough for turning, size of left will never be 0
			while ((left_edge.points.size() < TuningVar::roundroad_min_size) && FindOneLeftEdge()) {}
			//translate right
			for(int i=0; i<left_edge.points.size(); i++){
				path.push(left_edge.points[i].first + TuningVar::round_exit_offset, left_edge.points[i].second);
			}
		}
		else {
			while ((right_edge.points.size() < TuningVar::roundroad_min_size) && FindOneRightEdge()) {}
			for(int i=0; i<right_edge.points.size(); i++){
				path.push(right_edge.points[i].first -TuningVar::round_exit_offset, right_edge.points[i].second);
			}
		}

		/*OLD METHOD - Refer to commit before July 2nd*/
		break;
	}
	//	case Feature::kNormal:
	//	case Feature::kStraight: {
	default: {
		if (left_size < right_size) {
			for (int i = 0; i < right_edge.size(); i++) {
				auto curr_left = left_edge.points[(left_size * i) / right_size];
				auto curr_right = right_edge.points[i];
				int shift_left_null = 0;
				int shift_right_null = 0;
				TranslateType translate_flag = TranslateType::kNone;

				if (curr_left.first == 0
						&& translate_flag == TranslateType::kNone) {
					translate_flag = TranslateType::kLeftNull;
					shift_left_null = right_edge.points[i].first / 2;
				} else if (curr_right.first == 0
						&& translate_flag == TranslateType::kNone) {
					translate_flag = TranslateType::kRightNull;
					shift_right_null =
							(WorldSize.w - left_edge.points[i].first) / 2;
					//^^^ Start Translation ^^^
					//vvv Start Averaging  vvv
				} else if (curr_left.first != 0
						&& translate_flag != TranslateType::kNone) {
					translate_flag = TranslateType::kNone;
				} else if (curr_right.first != 0
						&& translate_flag != TranslateType::kNone) {
					translate_flag = TranslateType::kNone;
				}

				//if translate
				if (translate_flag == TranslateType::kLeftNull) {
					path.push(right_edge.points[i].first - shift_left_null,
							right_edge.points[i].second);
				} else if (translate_flag == TranslateType::kRightNull) {
					path.push(left_edge.points[i].first + shift_right_null,
							left_edge.points[i].second);
				} else {
					//if average
					int temp_x = (curr_left.first + curr_right.first) / 2;
					int temp_y = (curr_left.second + curr_right.second) / 2;
					path.push(temp_x, temp_y);
				}
			}
		} else {
			for (int i = 0; i < left_edge.size(); i++) {
				auto curr_left = left_edge.points[i];
				auto curr_right =
						right_edge.points[(right_size * i) / left_size];
				int shift_left_null = 0;
				int shift_right_null = 0;
				TranslateType translate_flag = TranslateType::kNone;

				if (curr_left.first == 0
						&& translate_flag == TranslateType::kNone) {
					translate_flag = TranslateType::kLeftNull;
					shift_left_null = right_edge.points[i].first / 2;
				} else if (curr_right.first == 0
						&& translate_flag == TranslateType::kNone) {
					translate_flag = TranslateType::kRightNull;
					shift_right_null =
							(WorldSize.w - left_edge.points[i].first) / 2;
					//^^^ Start Translation ^^^
					//vvv  Start Averaging  vvv
				} else if (curr_left.first != 0
						&& translate_flag != TranslateType::kNone) {
					translate_flag = TranslateType::kNone;
				} else if (curr_right.first != 0
						&& translate_flag != TranslateType::kNone) {
					translate_flag = TranslateType::kNone;
				}

				//if translate
				if (translate_flag == TranslateType::kLeftNull) {
					path.push(right_edge.points[i].first - shift_left_null,
							right_edge.points[i].second);
				} else if (translate_flag == TranslateType::kRightNull) {
					path.push(left_edge.points[i].first + shift_right_null,
							left_edge.points[i].second);
				} else {
					//if average
					int temp_x = (curr_left.first + curr_right.first) / 2;
					int temp_y = (curr_left.second + curr_right.second) / 2;
					path.push(temp_x, temp_y);
				}

			}
		}
		break;
	}
	}
}

/*
 * @brief: return the shortest side of current roundabout
 * @return: 1 means turning left, 0 means turning right
 * */
int roundabout_shortest(uint32_t a, int pos){
	return (a >> pos) & true;
}

/*
 * @brief: return whether overtake for current roundabout
 * @return: 1 means overtake, 0 means not overtake
 * */
 int roundabout_overtake(uint32_t a, int pos){
 	return (a >> pos) & true;
 }


/**
 * @brief Calculate the servo angle diff
 */
int16_t CalcAngleDiff() {
	int16_t error = 0, sum = 0;
	int avg = 0;
	uint16_t total_sum;
	if (!hadStoppingLine) total_sum = 10;
	else if (roundaboutExitStatus) total_sum = 40;
	else total_sum = 20;
	for (auto&& point : path.points) {
		if (sum > total_sum) //consider first 20 points
			break;
		error += (point.first - carMid.first);
		avg += point.first;
		sum++;
	}
	/*TUNNING carMid*/
//		char temp[100];
//		sprintf(temp, "avg: %.2f", avg/(float)sum);
//		pLcd->SetRegion(Lcd::Rect(0, 16, 128, 15));
//		pWriter->WriteString(temp);

	return error / sum * 20;
}

/*
 * @brief Find if the stopping line exist
 */
bool FindStoppingLine() {
	int refPoint = 0;
	int count = 0;
	for (int x = 0; x < 128; x++) {
		if (getFilteredBit(CameraBuf, x, 380) != refPoint) {
			count++;
			refPoint = !refPoint;
		}
		if (count > 10) {
			return true;
		}
	}
	return false;
}

/*
 * @brief Find if the stopping line exist further
 */
bool FindFurtherStoppingLine() {
	int refPoint = 1;
	int count = 0;
	for (int x = 0; x < 128; x++) {
		if (getFilteredBit(CameraBuf, x, 250) != refPoint) {
			count++;
			refPoint = !refPoint;
		}
		if (count > 11) {
			return true;
		}
	}
	return false;
}

/*
 * @brief Start line overtake
 * Left Car 1; Right Car 2
 * After overtake, Front Car 1; Back Car 2
 */
void StartlineOvertake() {
	/* For car 1; initial position: LEFT; after position: FRONT */

	int cnt = 0;
	while (1) {
		pMotor0->SetPower(500);
		pMotor1->SetPower(500);

		Capture();
		path.points.clear();
		for (int i = 0; i < 10; i++) FindOneLeftEdge();
		for (int i = 0; i < 10; i++) path.push(left_edge.points[i].first + 5, left_edge.points[i].second);

		pServo->SetDegree(util::clamp<uint16_t>(servo_bounds.kCenter - 1.3 * CalcAngleDiff(),
				servo_bounds.kRightBound,
				servo_bounds.kLeftBound));

		//		pEncoder0->Update();
		cnt += curr_enc_val_left;
		if (cnt > 2600) return;
	}
}


}  // namespace

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

void main_car1(bool debug_) {
	InflatePidValues();

	debug = debug_;

	Led::Config ConfigLed;
	ConfigLed.is_active_low = true;
	ConfigLed.id = 0;
	Led led0(ConfigLed);
	ConfigLed.id = 1;
	Led led1(ConfigLed);
	ConfigLed.id = 2;
	Led led2(ConfigLed);
	ConfigLed.id = 3;
	Led led3(ConfigLed);
	pLed3 = &led3;

	led0.SetEnable(false);
	led1.SetEnable(false);
	led2.SetEnable(false);
	led3.SetEnable(true);

	Ov7725::Config cameraConfig;
	cameraConfig.id = 0;
	cameraConfig.w = CameraSize.w;
	cameraConfig.h = CameraSize.h;
	cameraConfig.fps = Ov7725Configurator::Config::Fps::kHigh;
	cameraConfig.contrast = 0x2F;
	cameraConfig.brightness = 0x00;
	std::unique_ptr<Ov7725> camera = util::make_unique<Ov7725>(cameraConfig);
	spCamera = std::move(camera);
	spCamera->Start();

	FutabaS3010::Config ConfigServo;
	ConfigServo.id = 0;
	auto spServo = util::make_unique<FutabaS3010>(ConfigServo);
	pServo = spServo.get();

	DirEncoder::Config ConfigEncoder;
	ConfigEncoder.id = 0;
	auto spEncoder0 = util::make_unique<DirEncoder>(ConfigEncoder);
	pEncoder0 = spEncoder0.get();
	ConfigEncoder.id = 1;
	auto spEncoder1 = util::make_unique<DirEncoder>(ConfigEncoder);
	pEncoder1 = spEncoder1.get();

	DirMotor::Config ConfigMotor;
	ConfigMotor.id = 0;
	auto spMotor0 = util::make_unique<DirMotor>(ConfigMotor);
	pMotor0 = spMotor0.get();
	ConfigMotor.id = 1;
	auto spMotor1 = util::make_unique<DirMotor>(ConfigMotor);
	pMotor1 = spMotor1.get();

	FcYyUsV4 YYdistance(Pin::Name::kPtb0);

	JyMcuBt106::Config ConfigBT;
	ConfigBT.id = 0;
	ConfigBT.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	ConfigBT.rx_isr = &bluetoothListener;
	BTComm bt(ConfigBT);
//	JyMcuBt106 bt(ConfigBT);
//	pBTovertake = &bt;
	pBT = &bt;

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

	//  DebugConsole console(&joystick, &lcd, &writer, 10);

	Timer::TimerInt time_img = 0;
	while(!bt.hasStartReq()&&!debug&&joystick.GetState()==Joystick::State::kIdle);

	/*motor PID setting*/
	Timer::TimerInt pidStart = System::Time();
	IncrementalPidController<float, float> pid_left(0,0,0,0);
	pid_left_p = &pid_left;
	pid_left.SetOutputBound(-500, 500);
	IncrementalPidController<float, float> pid_right(0,0,0,0);
	pid_right_p = &pid_right;
	pid_right.SetOutputBound(-500, 500);
	pid_left.SetKp(2.5);
	pid_right.SetKp(2.5);
	pid_left.SetKi(0.02);
	pid_right.SetKi(0.02);
	pid_left.SetKd(0);
	pid_right.SetKd(0);

	//  DebugConsole console(&joystick, &lcd, &writer, 10);


	pServo->SetDegree(servo_bounds.kCenter);
	while(System::Time()-pidStart<5000);
	//	StartlineOvertake();

	pMotor0->SetClockwise(true);
	pMotor1->SetClockwise(false);

	Timer::TimerInt startTime=System::Time();
	bool met_stop_line=false;
	bool met_stop_line_slow_down = false;
	uint8_t stop_count=0;
	bool brake_flag = true;
	// update the overtake for the first roundabout
	if(TuningVar::overtake_mode){
		overtake = roundabout_overtake(TuningVar::roundabout_overtake_flag, 0);// get ready for the first roundabout
	}
	else{
		overtake = false;
	}

	//	int servoAngle = 0;
	while (true) {
		if(run){
			while (time_img != System::Time()) {
				time_img = System::Time();
				led0.SetEnable(time_img % 500 >= 250);

				if (time_img % 10 == 0) {
					if(TuningVar::show_algo_time){
						time_img = System::Time();
					}
					// disable obstacle after met one obstacle
					if (obstacle_cnt>0){
						TuningVar::obstacle_mode = false;
					}
					// if obstacle_mode == 0 avoid slow speed mode
					if (!TuningVar::obstacle_mode){
						obstacle_cnt = 1;
					}
					if (bt.hasStopCar()) met_stop_line = true;
					need_slow_down = false;
					bool skip_motor_protection=false;
					bt.resendNAKData();


					if (joystick.GetState() == Joystick::State::kSelect) bt.sendStopCar();



					//  Timer::TimerInt new_time = System::Time();
					Capture(); //Capture until two base points are identified
					if (stop_count) stop_count++;
					if (FindStoppingLine()) {
						if(is_front_car && time_img - startTime > 10000){
							stop_count++;
						}else if(time_img - startTime > 10000){
							bt.sendStopCar();
							stop_count++;
						}
						Capture(25);
					}
					if(FindFurtherStoppingLine() && is_front_car){
						met_stop_line_slow_down = true;
						bt.sendSpeed(10);
					}
					if(!is_front_car && bt.getBufferSpeed() == 10 && YYdistance.GetDistance() < 350) bt.sendSpeed(100);
					if (!hadStoppingLine && prevStoppingLine && !FindStoppingLine()) hadStoppingLine = true;
					else prevStoppingLine = FindStoppingLine();
					if (!hadStoppingLine){
						TuningVar::obstacle_mode = false;
					} else TuningVar::obstacle_mode = true;
					if ((stop_count>25 && !is_front_car) || (stop_count>50 && is_front_car)) met_stop_line = true;
					FindEdges();
					if (!hadStoppingLine){
						left_corners.erase(left_corners.begin(), left_corners.end());
						right_corners.erase(right_corners.begin(), right_corners.end());
					}
					Feature feature = featureIdent_Corner();
					if(feature == Feature::kRoundabout && is_front_car) bt.sendFeature(feature);
					GenPath(feature); //Generate path
					/*FOR DEBUGGING*/
					if(false){
						pLcd->SetRegion(Lcd::Rect(0,100,128,15));
//						stop_before_roundexit?pWriter->WriteString("StopB"):pWriter->WriteString("No StopB");
						need_slow_down?pWriter->WriteString("Slow"):pWriter->WriteString("No Slow");
					}
					if(debug){
						pLcd->SetRegion(Lcd::Rect(0,0,128,15));
						if (obsta_overtake_status == 1) pWriter->WriteString("OverLeft");
						if (obsta_overtake_status == 2)	pWriter->WriteString("OverRight");
						pLcd->SetRegion(Lcd::Rect(0,15,128,15));
						stop_obsta_overtake ?pWriter->WriteString("StopB"):pWriter->WriteString("No StopB");
						pLcd->SetRegion(Lcd::Rect(0,75,128,15));
						TuningVar::obstacle_mode?pWriter->WriteString("ObstaM"):pWriter->WriteString("No ObsM");
						pLcd->SetRegion(Lcd::Rect(0,90,128,15));
						is_front_car?pWriter->WriteString("Front"):pWriter->WriteString("Back");
						pLcd->SetRegion(Lcd::Rect(0,105,128,15));
						roundaboutStatus?pWriter->WriteString("RS = 1"):pWriter->WriteString("RS = 0");
						pLcd->SetRegion(Lcd::Rect(0,130,128,15));
						roundaboutExitStatus?pWriter->WriteString("RSE = 1"):pWriter->WriteString("RSE = 0");
						pLcd->SetRegion(Lcd::Rect(0,145,128,15));
						crossingStatus?pWriter->WriteString("Cross = 1"):pWriter->WriteString("Cross = 0");
						char temp_1[100];
						sprintf(temp_1, "Obs_cnt:%d", obstacle_cnt);
						pLcd->SetRegion(Lcd::Rect(0, 30, 128, 15));
						pWriter->WriteString(temp_1);
						sprintf(temp_1, "enc_overt:%d", encoder_total_obstacle_overtake);
						pLcd->SetRegion(Lcd::Rect(0, 45, 128, 15));
						pWriter->WriteString(temp_1);
						sprintf(temp_1, "encoder_obs:%d", encoder_total_obstacle);
						pLcd->SetRegion(Lcd::Rect(0, 60, 128, 15));
						pWriter->WriteString(temp_1);

					}
					if (debug) {
						PrintWorldImage();
						PrintEdge(left_edge, Lcd::kRed); //Print left_edge
						PrintEdge(right_edge, Lcd::kBlue); //Print right_edge
						PrintCorner(left_corners, Lcd::kPurple); //Print left_corner
						PrintCorner(right_corners, Lcd::kPurple); //Print right_corner
						PrintEdge(path, Lcd::kGreen); //Print path
						//						char timestr[100];
						//						pLcd->SetRegion(Lcd::Rect(0,30,128,15));
						//						sprintf(timestr, "Roun_cnt: %d", roundabout_cnt);
						//						pWriter->WriteString(timestr);


					}
					/*END OF DEBUGGING*/

					/*-------------CONTROL SYSTEM-----------------------*/
					int curr_servo_error = CalcAngleDiff();
//					char timestr[100];
//					pLcd->SetRegion(Lcd::Rect(0, 15, 128, 15));
//					sprintf(timestr, "error: %d", curr_servo_error);
//					pWriter->WriteString(timestr);
					/* Motor PID + Servo PID* for different situations*/

					if(roundaboutExitStatus == 1){
						//for stopping the car completely
						if(stop_before_roundexit && overtake){
							// case two: when front car meets exit, back car haven't finished overtake
							// case three: when front car meets exit, back car has finished overtake but interval time is not enough
							if( !pBT->hasFinishedOvertake() || (pBT->hasFinishedOvertake()
									&& (System::Time() - pBT->getOvertakeTime()) <= TuningVar::overtake_interval_time )){
								pServo->SetDegree(util::clamp<uint16_t>(
										servo_bounds.kCenter - (TuningVar::servo_roundabout_kp * curr_servo_error + TuningVar::servo_normal_kd * (curr_servo_error - prev_servo_error)),
										servo_bounds.kRightBound,
										servo_bounds.kLeftBound));
								pid_left.SetSetpoint(0);
								pid_right.SetSetpoint(0);
							}

							//below part only used for restarting the car after stopping
							else if(pBT->hasFinishedOvertake() && ((System::Time() - pBT->getOvertakeTime()) > TuningVar::overtake_interval_time)){
								//only delay when it really stops inside roundabout
//								System::DelayMs(TuningVar::overtake_interval_time);
								stop_before_roundexit = false;
								// roundaboutStatus = 0;
								exit_round_ready = false;
								//					pEncoder0->Update();
								//					pEncoder1->Update();
								encoder_total_exit = 0;
								pBT->resetFinishOvertake();
							}

						}
						//go
						else{
							pServo->SetDegree(util::clamp<uint16_t>(
									servo_bounds.kCenter - (TuningVar::servo_roundabout_kp * curr_servo_error + TuningVar::servo_normal_kd * (curr_servo_error - prev_servo_error)),
									servo_bounds.kRightBound,
									servo_bounds.kLeftBound));
							pid_left.SetSetpoint(TuningVar::targetSpeed_round*differential_left((pServo->GetDegree() - servo_bounds.kCenter)/10));
							pid_right.SetSetpoint(TuningVar::targetSpeed_round* differential_left((-pServo->GetDegree() + servo_bounds.kCenter)/10));
						}
					}

					//roundabout case
					else if(roundaboutStatus == 1){

						//for slowing down the car in advance when need to stop
//						if(stop_before_roundexit && overtake){
//							pServo->SetDegree(util::clamp<uint16_t>(
//									servo_bounds.kCenter - (TuningVar::servo_roundabout_kp * curr_servo_error + TuningVar::servo_normal_kd * (curr_servo_error - prev_servo_error)),
//									servo_bounds.kRightBound,
//									servo_bounds.kLeftBound));
//							pid_left.SetSetpoint(60);
//							pid_right.SetSetpoint(60);
//							//avoid another car's early pass the exit
//							if(pBT->hasFinishedOvertake()){
//								stop_before_roundexit = false;
//								// roundaboutStatus = 0;
//							}
//						}
						if(stop_before_roundexit && overtake && pBT->hasFinishedOvertake()){
							stop_before_roundexit = false;
						}
						/*New stopping method*/
						//slow down the car when the exit is ready
						else if(need_slow_down){
							pServo->SetDegree(util::clamp<uint16_t>(
									servo_bounds.kCenter - (TuningVar::servo_roundabout_kp * curr_servo_error + TuningVar::servo_normal_kd * (curr_servo_error - prev_servo_error)),
									servo_bounds.kRightBound,
									servo_bounds.kLeftBound));
							pid_left.SetSetpoint(TuningVar::targetSpeed_slow*differential_left((pServo->GetDegree() - servo_bounds.kCenter)/10));
							pid_right.SetSetpoint(TuningVar::targetSpeed_slow* differential_left((-pServo->GetDegree() + servo_bounds.kCenter)/10));
						}
						//the speed inside roundabout
						else if(abs(encoder_total_round) > TuningVar::round_encoder_count){
							pServo->SetDegree(util::clamp<uint16_t>(
									//use roundabout kp or normal kp?
									servo_bounds.kCenter - (TuningVar::servo_roundabout_kp * curr_servo_error + TuningVar::servo_normal_kd * (curr_servo_error - prev_servo_error)),
									servo_bounds.kRightBound,
									servo_bounds.kLeftBound));
							pid_left.SetSetpoint(TuningVar::targetSpeed_normal*differential_left((pServo->GetDegree() - servo_bounds.kCenter)/10));
							pid_right.SetSetpoint(TuningVar::targetSpeed_normal* differential_left((-pServo->GetDegree() + servo_bounds.kCenter)/10));
						}
						//the speed during the entrance
						else{
							pServo->SetDegree(util::clamp<uint16_t>(
									servo_bounds.kCenter - (TuningVar::servo_roundabout_kp * curr_servo_error + TuningVar::servo_normal_kd * (curr_servo_error - prev_servo_error)),
									servo_bounds.kRightBound,
									servo_bounds.kLeftBound));
							pid_left.SetSetpoint(TuningVar::targetSpeed_round*differential_left((pServo->GetDegree() - servo_bounds.kCenter)/10));
							pid_right.SetSetpoint(TuningVar::targetSpeed_round* differential_left((-pServo->GetDegree() + servo_bounds.kCenter)/10));
						}
					}

					//sharp turning case
					else if(abs(curr_servo_error) > 140){
						pServo->SetDegree(util::clamp<uint16_t>(
								servo_bounds.kCenter - (TuningVar::servo_sharp_turn_kp * curr_servo_error + TuningVar::servo_normal_kd * (curr_servo_error - prev_servo_error)),
								servo_bounds.kRightBound,
								servo_bounds.kLeftBound));
						pid_left.SetSetpoint(TuningVar::targetSpeed_sharp_turn*differential_left((pServo->GetDegree() - servo_bounds.kCenter)/10));
						pid_right.SetSetpoint(TuningVar::targetSpeed_sharp_turn* differential_left((-pServo->GetDegree() + servo_bounds.kCenter)/10));
					}

					//straight case + TODO:double check further image to decide whether add speed or not
					else if(abs(curr_servo_error) < 50){
//						              pLcd->SetRegion(Lcd::Rect(0, 0, 128, 15));
//						              pWriter->WriteString("Pstraight");
						/*find more 25 edges*/
						while ((left_edge.points.size() < 50) && FindOneLeftEdge()) {}
						while ((right_edge.points.size() < 50) && FindOneRightEdge()) {}

						// case one: have reached worldview boundary - must be NON-straight
						if(left_edge.points.size() < 50 || right_edge.points.size() < 50){
							//still use straight_kp
							pServo->SetDegree(util::clamp<uint16_t>(
									servo_bounds.kCenter - (TuningVar::servo_straight_kp * curr_servo_error + TuningVar::servo_normal_kd * (curr_servo_error - prev_servo_error)),
									servo_bounds.kRightBound,
									servo_bounds.kLeftBound));
							pid_left.SetSetpoint(TuningVar::targetSpeed_slow*differential_left((pServo->GetDegree() - servo_bounds.kCenter)/10));
							pid_right.SetSetpoint(TuningVar::targetSpeed_slow* differential_left((-pServo->GetDegree() + servo_bounds.kCenter)/10));
						}
						// both edges have 50
						else{
							for(int i =34; i<50; i++){
								path.push((left_edge.points[i].first + right_edge.points[i].first)/2,(left_edge.points[i].second + right_edge.points[i].second)/2);
							}
							int further_servo_error = 0;
							int sum = 0;
							for (auto&& point : path.points) {
								sum++;
								if(sum<25) continue; // only consider latter 25 points
								further_servo_error += (point.first - carMid.first);
							}
//							further_servo_error = further_servo_error;
//							pLcd->SetRegion(Lcd::Rect(0, 45, 128, 15));
//							sprintf(timestr, "Ferror: %d", abs(further_servo_error));
//							pWriter->WriteString(timestr);


							// case two: the upper 25 path points produce error bigger than 100 - reduce speed in advance
							if(abs(further_servo_error)>300 || need_slow_down) {
								pServo->SetDegree(util::clamp<uint16_t>(
										servo_bounds.kCenter - (TuningVar::servo_straight_kp * curr_servo_error + TuningVar::servo_normal_kd * (curr_servo_error - prev_servo_error)),
										servo_bounds.kRightBound,
										servo_bounds.kLeftBound));
								pid_left.SetSetpoint(TuningVar::targetSpeed_slow*differential_left((pServo->GetDegree() - servo_bounds.kCenter)/10));
								pid_right.SetSetpoint(TuningVar::targetSpeed_slow* differential_left((-pServo->GetDegree() + servo_bounds.kCenter)/10));
							}
							// case three: real straight - add full power
							else{
//					              pLcd->SetRegion(Lcd::Rect(0, 30, 128, 15));
//					              pWriter->WriteString("Straight");
								pServo->SetDegree(util::clamp<uint16_t>(
										servo_bounds.kCenter - (TuningVar::servo_straight_kp * curr_servo_error + TuningVar::servo_normal_kd * (curr_servo_error - prev_servo_error)),
										servo_bounds.kRightBound,
										servo_bounds.kLeftBound));
								pid_left.SetSetpoint(TuningVar::targetSpeed_straight*differential_left((pServo->GetDegree() - servo_bounds.kCenter)/10));
								pid_right.SetSetpoint(TuningVar::targetSpeed_straight* differential_left((-pServo->GetDegree() + servo_bounds.kCenter)/10));
							}
						}
					}

					//normal turning case
					else{
						pServo->SetDegree(util::clamp<uint16_t>(
								servo_bounds.kCenter - (TuningVar::servo_normal_kp * curr_servo_error + TuningVar::servo_normal_kd * (curr_servo_error - prev_servo_error)),
								servo_bounds.kRightBound+85,
								servo_bounds.kLeftBound-85));
						pid_left.SetSetpoint(TuningVar::targetSpeed_normal*differential_left((pServo->GetDegree() - servo_bounds.kCenter)/10));
						pid_right.SetSetpoint(TuningVar::targetSpeed_normal* differential_left((-pServo->GetDegree() + servo_bounds.kCenter)/10));
					}

					prev_servo_error = curr_servo_error;
					pEncoder0->Update();
					pEncoder1->Update();
					if(!obstacle_cnt){//System::Time() - startTime < 1000){
						pid_left.SetSetpoint(90);
						pid_right.SetSetpoint(90);
					}
					if(met_stop_line_slow_down && is_front_car){
						if(bt.getBufferSpeed() == 100){
							pid_left.SetSetpoint(TuningVar::targetSpeed_straight);
							pid_right.SetSetpoint(TuningVar::targetSpeed_straight);
						}else{
							pid_left.SetSetpoint(30);
							pid_right.SetSetpoint(30);
						}
					}
					if(met_stop_line || (stop_obsta_overtake && is_front_car && obsta_overtake_status != 0)){
						pid_left.SetSetpoint(0);
						pid_right.SetSetpoint(0);
					}
					curr_enc_val_left = pEncoder0->GetCount();
					curr_enc_val_right = -pEncoder1->GetCount();
					SetMotorPower(GetMotorPower(0)+pid_left.Calc(curr_enc_val_left),0);
					SetMotorPower(GetMotorPower(1)+pid_right.Calc(curr_enc_val_right),1);
					//				if((curr_enc_val_left<100 || curr_enc_val_right<100) && (System::Time()-startTime>1000 || skip_motor_protection)){
					//					pMotor0->SetPower(0);
					//					pMotor1->SetPower(0);
					//				}
					if(TuningVar::show_algo_time){
						char buf[10] = {};
						sprintf(buf, "%d", System::Time()-time_img);
						pLcd->SetRegion(Lcd::Rect(5,5,100,15));
						pWriter->WriteString(buf);
					}
				}
			}

		}
	}

}

}  // namespace car1
}  // namespace optimal
}  // namespace algorithm
