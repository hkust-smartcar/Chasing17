/*
 * optimal.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Peter Tse (mcreng), Dipsy Wong, King Huang (XuhuaKing)
 *
 * Optimal Path Algorithm CPP File
 *
 */

#include "algorithm/optimal_car1.h"
#include <vector>
#include <cstring>
#include <sstream>
#include <stdio.h>

#include "libsc/alternate_motor.h"
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

#include "bluetooth.h"
#include "debug_console.h"
#include "util/util.h"
#include "algorithm/worldview/car1.h"

using libsc::AlternateMotor;
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

namespace algorithm {
namespace optimal {
namespace car1 {

namespace {
//BT listener
std::string inputStr;
bool tune = false;
std::vector<double> constVector;
uint8_t now_angle = 0;

Edges left_edge;
Edges right_edge;
Edges path;
Corners left_corners;
Corners right_corners;
std::array<std::pair<uint16_t, uint16_t>, 2> inc_width_pts; //0:left, 1:right
uint16_t start_y; //For crossing, store the last start point coordinate
uint16_t start_x;
uint16_t prev_corner_x; //store the latest corner coordinate appears last time during roundabout
uint16_t prev_corner_y;

/*FOR OVERTAKING*/
bool is_front_car = true;
bool stop_before_roundexit = true;

bool debug = true;
bool has_inc_width_pt = false;
bool is_straight_line = false;
bool exit_round_ready = false; // A flag storing corner status inside roundabout
int roundaboutStatus = 0; // 0: Before 1: Detected 2: Inside (After one corner)
int crossingStatus = 0; // 0: Before 1: Detected/Inside
int roundaboutExitStatus = 0; //0: Before 1: Detected/Inside Exit of Roundabout
uint16_t prev_track_width = 0;
int encoder_total_cross = 0; //for crossroad
int encoder_total_round = 0; // for roundabout
int encoder_total_exit = 0;
int roundabout_cnt = 0; // count the roundabout
Timer::TimerInt feature_start_time;
std::pair<int, int> carMid {61, 0};
int roundabout_nearest_corner_cnt_left = pow(TuningVar.corner_range * 2 + 1, 2); // for finding the nearest corner point for roundabout
int roundabout_nearest_corner_cnt_right = pow(TuningVar.corner_range * 2 + 1, 2);
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
AlternateMotor* pMotor0 = nullptr;
AlternateMotor* pMotor1 = nullptr;

ServoBounds servo_bounds = {1040, 755, 470};

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
void Capture();
Feature featureIdent_Corner();
bool FindStoppingLine();
bool FindEdges();
bool FindOneLeftEdge();
bool FindOneRightEdge();
void GetPath(Feature);
bool getWorldBit(int, int);
void PrintCorner(Corners, uint16_t);
void PrintEdge(Edges, uint16_t);
void PrintImage();
void PrintSuddenChangeTrackWidthLocation(uint16_t);
void PrintWorldImage();
int roundabout_shortest(uint8_t a, int pos);

/*
 * @brief: bluetooth listener for processing tuning
 * */
bool bluetoothListener(const Byte *data, const size_t size) {
	if (data[0] == 'P') {
			//space
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
			TuningVar.angle_div_error = constVector[0];
			now_angle = constVector[1];
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
	if (x <= 0 || x > CameraSize::w - 1 || y <= 0 || y > CameraSize::h - 1)
		return 1; //-1; // Out of bound = black
	//y = CameraSize::h - 1 - y; // set y = 0 at bottom

	return buff[y * CameraSize::w / 8 + x / 8] >> (7 - (x % 8)) & 1; //buff[y*CameraSize::w/8+x/8] & 0x80>>x%8;

	// Median Filter
	int count = 0, total = 0;

	for (int i = max(0, x - 1); i < min(CameraSize::w - 1, x + 1); i++) {
		for (int j = max(0, y - 1); j < min(CameraSize::h - 1, y + 1); j++) {
			total++;
			count += buff[j * CameraSize::w / 8 + i / 8] >> (7 - (i % 8)) & 1;
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
	if (i_x <= 0 || i_x > CameraSize::w - 1 || i_y <= 0
			|| i_y > CameraSize::h - 1)
		return -1;
	return CameraBuf[i_y * CameraSize::w / 8 + i_x / 8] >> (7 - (i_x % 8)) & 1;
}

/**
 * @brief To fetch filtered bit from worldview corrected data, 1 = black; 0 = white
 * @param (x,y): coordinate system in which bottom left corner := (0, 0)
 * @return world bit
 */
bool getWorldBit(int w_x, int w_y) {
	if (w_x <= 0 || w_x > WorldSize::w - 1 || w_y <= 0 || w_y > WorldSize::h - 1)
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
void Capture() {
	left_edge.points.clear();
	right_edge.points.clear();
	bool found_left = false, found_right = false;
	int left_x = -1, left_y = -1;
	int right_x = -1, right_y = -1;
	CameraBuf = spCamera->LockBuffer();
	spCamera->UnlockBuffer();

	//Search horizontally
	for (int i = WorldSize::w / 2; i > 0; i--) {
		if (getWorldBit(i, TuningVar.starting_y) == 1) {
			left_x = i + 1;
			left_y = TuningVar.starting_y;
			found_left = true;
			break;
		}
	}
	if (!found_left) {
		left_x = 1;
		left_y = TuningVar.starting_y;
	}

	//Search horizontally
	for (int i = WorldSize::w / 2; i < WorldSize::w; i++) {
		if (getWorldBit(i, TuningVar.starting_y) == 1) {
			right_x = i - 1;
			right_y = TuningVar.starting_y;
			found_right = true;
			break;
		}
	}
	if (!found_right) {
		right_x = WorldSize::w - 1;
		right_y = TuningVar.starting_y;
	}

	pLed3->Switch();
	left_edge.push(left_x, left_y);
	right_edge.push(right_x, right_y);
}

/**
 * @brief Print raw image to LCD
 */
void PrintImage() {
	pLcd->SetRegion(Lcd::Rect(0, 0, CameraSize::w, CameraSize::h));
	pLcd->FillBits(0x0000, 0xFFFF, CameraBuf, spCamera->GetBufferSize() * 8);
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
					i > max(prev_x - TuningVar.edge_hor_search_max, 1); i--) {
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
				< min(prev_x + TuningVar.edge_hor_search_max,
						WorldSize::w - 1); i++) {
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
	if (left_edge.points.back().second == WorldSize::h - 1)
		return false; //reaches top
	if (left_edge.points.back().first == 1)
		return false; //reaches left
	if (left_edge.points.back().first == WorldSize::w - 1)
		return false; //reaches right


	int CornerCheck = 0;
	int total = 0;
	auto last = left_edge.points.back();
	if (last.first - TuningVar.corner_range <= 0
			|| last.first + TuningVar.corner_range > WorldSize::w - 1
			|| last.second - TuningVar.corner_range <= 0
			|| last.second + TuningVar.corner_range > WorldSize::h - 1)
		return true;
	for (int i = (last.first - TuningVar.corner_range);
			i <= (last.first + TuningVar.corner_range); i++) {
		for (int j = (last.second - TuningVar.corner_range);
				j <= (last.second + TuningVar.corner_range); j++) {
			CornerCheck += getWorldBit(i, j);
			total++;
		}
	}
	//find corners
	if (left_edge.points.back().second
			<= WorldSize::h / TuningVar.corner_height_ratio) {

		//if in this threshold, consider as corner
		if (CornerCheck > total * TuningVar.corner_min / 100
				&& CornerCheck < total * TuningVar.corner_max / 100) {
			if (abs(last.first - left_corners.points.back().first)
					+ abs(last.second - left_corners.points.back().second)
					<= TuningVar.min_corners_dist) { //discard if too close
				return true;
			}
			left_corners.push(last.first, last.second);

		}
	}

	//check if the point is the point closest to corners
	if (CornerCheck < roundabout_nearest_corner_cnt_left) {
		roundabout_nearest_corner_cnt_left = CornerCheck;
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
		if (prev_x == WorldSize::w - 1) { //if align with left bound
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
					< min(prev_x + TuningVar.edge_hor_search_max,
							WorldSize::w - 1); i++) {
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
		for (int i = prev_x; i > max(prev_x - TuningVar.edge_hor_search_max, 1);
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
	if (right_edge.points.back().second == WorldSize::h - 1)
		return false; //reaches top
	if (right_edge.points.back().first == 1)
		return false; //reaches left
	if (right_edge.points.back().first == WorldSize::w - 1)
		return false; //reaches right

	int CornerCheck = 0;
	int total = 0;
	auto last = right_edge.points.back();
	if (last.first - TuningVar.corner_range <= 0
			|| last.first + TuningVar.corner_range > WorldSize::w - 1
			|| last.second - TuningVar.corner_range <= 0
			|| last.second + TuningVar.corner_range > WorldSize::h - 1)
		return true;
	for (int i = (last.first - TuningVar.corner_range);
			i <= (last.first + TuningVar.corner_range); i++) {
		for (int j = (last.second - TuningVar.corner_range);
				j <= (last.second + TuningVar.corner_range); j++) {
			CornerCheck += getWorldBit(i, j);
			total++;
		}
	}
	//find corners
	if (right_edge.points.back().second
			<= WorldSize::h / TuningVar.corner_height_ratio) {

		//if in this threshold, consider as corner
		if (CornerCheck > total * TuningVar.corner_min / 100
				&& CornerCheck < total * TuningVar.corner_max / 100) {
			if (abs(last.first - right_corners.points.back().first)
					+ abs(last.second - right_corners.points.back().second)
					<= TuningVar.min_corners_dist) { //discard if too close
				return true;
			}
			right_corners.push(last.first, last.second);

		}

	}

	//check if the point is the point closest to corners
	if (CornerCheck < roundabout_nearest_corner_cnt_right) {
		roundabout_nearest_corner_cnt_right = CornerCheck;
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
	has_inc_width_pt = false;
	left_corners.points.clear();
	right_corners.points.clear();
	bool flag_break_left = left_edge.points.size() == 0;
	bool flag_break_right = right_edge.points.size() == 0;
	uint16_t staright_line_edge_count = 0; // Track the num. of equal width
	roundabout_nearest_corner_cnt_left = pow(TuningVar.corner_range * 2 + 1, 2);
	roundabout_nearest_corner_cnt_right = pow(TuningVar.corner_range * 2 + 1, 2);
	while (left_edge.points.size() <= 100 && right_edge.points.size() <= 100
			&& (!flag_break_left || !flag_break_right)) {
		if (!flag_break_left)
			flag_break_left = !FindOneLeftEdge();
		if (!flag_break_right)
			flag_break_right = !FindOneRightEdge();

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
				< TuningVar.min_edges_dist) { //two edges meet
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
				left_edge.points.back().first + 1, WorldSize::w - 1)][WorldSize::h
																	  - left_edge.points.back().second][0] == -1) {
			flag_break_left = true;
		}
		if (worldview::car1::transformMatrix[max(
				left_edge.points.back().first - 1, 1)][WorldSize::h
													   - left_edge.points.back().second][0] == -1) {
			flag_break_left = true;
		}
		if (right_edge.points.back().second
				> TuningVar.edge_min_worldview_bound_check) {
			if (worldview::car1::transformMatrix[min(
					right_edge.points.back().first + 1, WorldSize::w - 1)][WorldSize::h
																		   - right_edge.points.back().second][0] == -1) {
				flag_break_right = true;
			}
			if (worldview::car1::transformMatrix[max(
					right_edge.points.back().first - 1, 1)][WorldSize::h
															- right_edge.points.back().second][0] == -1) {
				flag_break_right = true;
			}
		}

		//check sudden change in track width
		if (!has_inc_width_pt) {
			uint16_t dist = (left_edge.points.back().first
					- right_edge.points.back().first)
        						  * (left_edge.points.back().first
        								  - right_edge.points.back().first)
										  + (left_edge.points.back().second
												  - right_edge.points.back().second)
												  * (left_edge.points.back().second
														  - right_edge.points.back().second);
			//      /*FOR DEBUGGING*/
//			      char temp[100];
//			      sprintf(temp, "_dist:%d", dist - prev_track_width);
//			      pLcd->SetRegion(Lcd::Rect(0, 54, 128, 15));
//			      pWriter->WriteString(temp);


			//Straight line judgement
			if (left_edge.points.size() > 1 && right_edge.points.size() > 1 && abs(dist - prev_track_width) < 3) {
				staright_line_edge_count++;
			}
			if (dist >= TuningVar.track_width_threshold
					&& dist - prev_track_width
					>= TuningVar.track_width_change_threshold) {
				inc_width_pts.at(0) = left_edge.points.back();
				inc_width_pts.at(1) = right_edge.points.back();
				has_inc_width_pt = true;
			} else
				prev_track_width = dist;
		}
	}
	//Straight line judgement
	if (staright_line_edge_count >= TuningVar.straight_line_threshold) {
		is_straight_line = true;
	}
	return true;
}

/**
 * @brief Feature Identification by corners
 *
 * Algorithm:
 * 1. Two corners connected together
 * 2. Perpendicular direction, search points @sightDistance away.
 * 3. Black (1) is roundabout, White (0) is crossing
 *
 * @return: Feature: kCrossing, kRound
 * @note: Execute this function after calling FindEdges()
 */
Feature featureIdent_Corner() {
	//1. Straight line
	if (is_straight_line) {
		//    roundaboutStatus = 0;// Avoid failure of detecting roundabout exit
		return Feature::kStraight;
	}

	/*Only assume the first two corners are useful*/
	if (left_corners.points.size() > 0 && right_corners.points.size() > 0) {
		//3. More than two valid corner case
		uint16_t cornerMid_x = (left_corners.points.front().first
				+ right_corners.points.front().first) / 2; //corner midpoint x-cor
		uint16_t cornerMid_y = (left_corners.points.front().second
				+ right_corners.points.front().second) / 2; //corner midpoint y-cor
		if (abs(carMid.second - cornerMid_y) > TuningVar.action_distance) {
			return Feature::kNormal;
		}
		uint16_t test_y = carMid.second + TuningVar.sightDist;
		uint16_t test_x =
				(test_y - cornerMid_y) * (right_corners.points.front().second - left_corners.points.front().second)
				/ (left_corners.points.front().first
						- right_corners.points.front().first) + cornerMid_x;
		/*FOR DEBUGGING*/
		//		if (true) {
		//			pLcd->SetRegion(Lcd::Rect(test_x, WorldSize::h - test_y - 1, 4, 4));
		//			pLcd->FillColor(Lcd::kYellow);
		//		}
		/*END OF DEBUGGING*/
		if (getWorldBit(test_x, test_y) && getWorldBit(test_x + 1, test_y)
				&& getWorldBit(test_x, test_y + 1)
				&& getWorldBit(test_x - 1, test_y)
				&& roundaboutStatus == 0
				&& crossingStatus == 0/*Temporary close*/) {
			//All black
//			pEncoder0->Update();
//			pEncoder1->Update();
			encoder_total_round = 0;
			roundaboutStatus = 1; //Detected
			//			feature_start_time = System::Time(); // Mark the startTime of latest enter time
			roundabout_cnt++;
			return Feature::kRoundabout;
		} else if (!getWorldBit(test_x, test_y)
				&& !getWorldBit(test_x + 1, test_y)
				&& !getWorldBit(test_x, test_y + 1)
				&& !getWorldBit(test_x - 1, test_y)
				&& crossingStatus == 0
				&& roundaboutStatus == 0) // avoid double check for crossing when inside the crossing (encoder_total_cross<2500)
		{
//			pEncoder0->Update();
			encoder_total_cross = 0;
			crossingStatus = 1; //Detected
			//			feature_start_time = System::Time(); // Mark the startTime of latest enter time
			//      roundaboutStatus = 0;// Avoid failure of detecting roundabout exit
			start_y = carMid.second
					+ (TuningVar.cross_cal_start_num
							- encoder_total_cross / TuningVar.cross_cal_ratio);
			start_x = (start_y - (left_corners.points.front().second + right_corners.points.front().second) / 2)
        						  / (left_corners.points.front().first - right_corners.points.front().first)
								  * (right_corners.points.front().second
										  - left_corners.points.front().second)
										  + (left_corners.points.front().first
												  + right_corners.points.front().first) / 2;
			return Feature::kCross;
		}
	}
	/*
	 * @Note:
	 * roundaboutStatus: Becomes 1 once detected, becomes 0 after exit
	 * exit_round_ready: Detects 1 corner after completely entering the roundabout
	 * roundaboutExitStatus: Becomes 1 when exit is ready and one corner disappear, becomes 0 after encoderExit is reached
	 * */
	//4. Only one corner case: Only one corner - Exit/Cross/Entering crossing & roundabout
	else if (left_corners.points.size() > 0 || right_corners.points.size() > 0) {
		/*Double check for crossing to handle only one corner case*/
		if (roundaboutStatus == 0 && crossingStatus
				== 0) { // avoid double check for crossing when inside the crossing (encoder_total_cross<2500)){ //Not inside roundabout, not Exit of Roundabout when encounter one corner case - CONDITION_1
			//	  //Both sides are break due to -1 - CONDITION_2
			//	  bool both_break = false;
			// Only one corner - CONDITION_3
			bool crossing = false;
			//reaches worldview boundaries
			//right edge touch right boundary + left corner
			if ((worldview::car1::transformMatrix[min(
					right_edge.points.back().first + 1, WorldSize::w - 1)][WorldSize::h
																		   - right_edge.points.back().second][0] == -1)
					&& left_corners.points.size() > 0) {
				//Only left corner + close enough
				if (abs(left_corners.points.front().second - carMid.second)
						<= TuningVar.min_dist_meet_crossing) {
					//push the midpoint of right edge into corner
					right_corners.push(
							(right_edge.points.front().first
									+ right_edge.points.back().first) / 2,
									(right_edge.points.front().second
											+ right_edge.points.back().second) / 2);
					crossing = true;
				}
			}
			//left edge touch left boundary)
			if ((worldview::car1::transformMatrix[max(
					left_edge.points.back().first - 1, 1)][WorldSize::h
														   - left_edge.points.back().second][0] == -1)
					&& right_corners.points.size() > 0) {
				//Only right corner
				if (abs(right_corners.points.front().second - carMid.second)
						<= TuningVar.min_dist_meet_crossing) {
					left_corners.push(
							(left_edge.points.front().first
									+ left_edge.points.back().first) / 2,
									(left_edge.points.front().second
											+ left_edge.points.back().second) / 2);
					crossing = true;
				}
			}

			if (crossing) {
				//Record the start midpoint for searching
				start_y = carMid.second
						+ (TuningVar.cross_cal_start_num
								- encoder_total_cross
								/ TuningVar.cross_cal_ratio);
				start_x = (start_y
						- (left_corners.points.front().second
								+ right_corners.points.front().second) / 2)
            						/ (left_corners.points.front().first
            								- right_corners.points.front().first)
											* (right_corners.points.front().second
													- left_corners.points.front().second)
													+ (left_corners.points.front().first
															+ right_corners.points.front().first) / 2;
//				pEncoder0->Update();
				crossingStatus = 1; //Detected
				encoder_total_cross = 0;
				return Feature::kCross;
			}
		}
	}
	/*Exit case handling: ready -> exit*///To avoid entrance double check for corner
	if ((left_corners.points.size() > 0 || right_corners.points.size() > 0)
			&& roundaboutStatus == 1 && roundaboutExitStatus == 0
			&& (abs(encoder_total_round) > TuningVar.round_encoder_count)) {
		//		//keep updating until corner disappear
		//		if (left_corners.points.size() > 0) {
		//			prev_corner_x = left_corners.points.front().first;
		//			prev_corner_y = left_corners.points.front().second;
		//		}
		//		if (right_corners.points.size() > 0) {
		//			prev_corner_x = right_corners.points.front().first;
		//			prev_corner_y = right_corners.points.front().second;
		//		}
		exit_round_ready = true; // Detect one corner
	}
	/*FOR DEBUGGING*/
	if (debug) {
		//		char temp_1[100];
		//		sprintf(temp_1, "Ycor:%d", abs(roundabout_nearest_corner_left.second - carMid.second));
		//		pLcd->SetRegion(Lcd::Rect(0, 0, 128, 15));
		//		pWriter->WriteString(temp_1);
		//		pLcd->SetRegion(Lcd::Rect(0,30,128,15));
		//		exit_round_ready?pWriter->WriteString("ready"):pWriter->WriteString("Not ready");
		//		pLcd->SetRegion(Lcd::Rect(0,45,128,15));
		//		roundaboutStatus?pWriter->WriteString("RS = 1"):pWriter->WriteString("RS = 0");
		//		pLcd->SetRegion(Lcd::Rect(0,56,128,15));
		//		roundaboutExitStatus?pWriter->WriteString("RSE = 1"):pWriter->WriteString("RSE = 0");
	}

	/*FOR DEBUGGING*/
	if (debug) {
		pLcd->SetRegion(Lcd::Rect(0, 30, 128, 15));
		exit_round_ready ? pWriter->WriteString("ready") : pWriter->WriteString("Not ready");
		pLcd->SetRegion(Lcd::Rect(0, 45, 128, 15));
		roundaboutStatus ? pWriter->WriteString("RS = 1") : pWriter->WriteString("RS = 0");

	}
	/*END OF DEBUGGING*/
	//the moment when corner disappears + inside roundabout is Exit /*+ Front is black*/
	//Second time: Exit TODO: change to encoder reader
	bool meet_exit = false;
	if (exit_round_ready && roundaboutStatus == 1
			&& abs(encoder_total_round) > TuningVar.round_encoder_count/*abs(System::Time() - feature_start_time) > TuningVar.feature_inside_time*/) {

		if (is_front_car?!roundabout_shortest(TuningVar.roundabout_shortest_flag, roundabout_cnt - 1):roundabout_shortest(TuningVar.roundabout_shortest_flag, roundabout_cnt - 1)) {
			/*FOR DEBUGGING*/
			if (debug) {
				pLcd->SetRegion(Lcd::Rect(roundabout_nearest_corner_left.first, WorldSize::h - roundabout_nearest_corner_left.second - 1, 4, 4));
				pLcd->FillColor(Lcd::kCyan);
			}
			/*END OF DEBUGGING*/
			// corner disappears && close enough
			meet_exit = abs(roundabout_nearest_corner_left.second - carMid.second) < TuningVar.exit_action_dist;
		} else {
			/*FOR DEBUGGING*/
			if (debug) {
				pLcd->SetRegion(Lcd::Rect(roundabout_nearest_corner_right.first, WorldSize::h - roundabout_nearest_corner_right.second - 1, 4, 4));
				pLcd->FillColor(Lcd::kCyan);
			}
			/*END OF DEBUGGING*/
			// corner disappears && close enough
			meet_exit = abs(roundabout_nearest_corner_right.second - carMid.second) < TuningVar.exit_action_dist;
		}
		//TODO: receive buffer message: only keep moving when receiving exit message
		if(!is_front_car){//back car
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
		else{
			if (meet_exit) {
				//GO
				if(pBT->hasFinishedOvertake()){
					stop_before_roundexit = false;
					// roundaboutStatus = 0;
					exit_round_ready = false;
//					pEncoder0->Update();
//					pEncoder1->Update();
					encoder_total_exit = 0;
					pBT->resetFinishOvertake();
				}
				else{
					stop_before_roundexit = true;
				}
				roundaboutExitStatus = 1;
				return Feature::kRoundaboutExit;
			}
		}
		//	  }
	}
	//5. Nothing special: Return kNormal and wait for next testing
	return Feature::kNormal;
}

/**
 * @brief Print edges
 */
void PrintEdge(Edges path, uint16_t color) {
	for (auto&& entry : path.points) {
		pLcd->SetRegion(
				Lcd::Rect(entry.first, WorldSize::h - entry.second - 1, 2, 2));
		pLcd->FillColor(color);
	}

}

/**
 * @brief Print corners
 */
void PrintCorner(Corners corners, uint16_t color) {
	for (auto&& entry : corners.points) {
		pLcd->SetRegion(
				Lcd::Rect(entry.first, WorldSize::h - entry.second - 1, 4, 4));
		pLcd->FillColor(color);
	}
}

/**
 * Path generation
 * 1. kNormal/kStraight: Weighted average path ("Center line")
 *  - If in a range of y such that there exists no left edge: translate right edge towards the left
 *  - If in a range of y such that there exists no right edge: translate left edge towards the right
 *  - Otherwise, take the average of x as the the path
 * 2. kCross: Path = connect carMid with cornerMid
 * 3. kRoundabout:
 *  - Turn left: Left edge stay same, right_path_1 is carMid points upward, right_path_2 is Find_one_right_edge
 *  - Turn right: Right edge stay same, left_path_1 is carMid points upward, left_path_2 is Find_one_left_edge
 * 4. kRoundaboutExit (Exit of roundabout):
 *  - Turn right: Right edge stay same, left_path_1 is carMid points upward until reaching black
 *  - Turn left: Left edge stay same, right_path_1 is carMid points upward until reaching black
 *
 * TODO (mcreng): Naive psuedo-optimal path ("Curve fitting")
 *
 * Points to take:
 * 1. Current position (width/2, 0)
 * 2. Start/End points of shifted curve due to LEFT_NULL or RIGHT_NULL
 * 3. Under no LEFT_NULL and RIGHT_NULL, the midpt's midpt
 */
void GenPath(Feature feature) {
	int left_size = left_edge.size();
	int right_size = right_edge.size();

	path.points.clear();

	if (!left_size && !right_size) { //simple validity check
		return;
	}
	/*FOR DEBUGGING*/
	if (debug) {
		char temp[100];
		sprintf(temp, "Enc:%d", encoder_total_cross);
		pLcd->SetRegion(Lcd::Rect(0, 16, 128, 15));
		pWriter->WriteString(temp);
	}
	/* CROSSING PASSING PART - Include kCross handling - Overwrite next normal kCross part
	 * @crossingStatus: for storing the status for a period
	 * @kCross: for opening the "switch"
	 * */
	if (crossingStatus == 1
			&& encoder_total_cross
			>= TuningVar.cross_encoder_count/*abs(System::Time() - feature_start_time) > TuningVar.feature_inside_time*/) { //&& encoder pass crossing
		crossingStatus = 0;
	}
	if (crossingStatus == 1
			&& encoder_total_cross < TuningVar.cross_encoder_count) { //Gen new path by searching midpoint
//		pEncoder0->Update÷();
		encoder_total_cross += curr_enc_val_left;
		uint16_t new_right_x;
		uint16_t new_left_x;
		// find new right edge
		for (uint16_t i = start_x; i < WorldSize::w; i++) {
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
							WorldSize::h - start_y - 1, 4, 4));
			pLcd->FillColor(Lcd::kRed);
		}
		/*END OF DEBUGGING*/

		path.push(((new_left_x + new_right_x) / 2 + carMid.first) / 2, start_y); //follow the new midpoint
		//Update start_x and start_y
		start_y = carMid.second
				+ (TuningVar.cross_cal_start_num
						- encoder_total_cross / TuningVar.cross_cal_ratio);
		start_x = (new_left_x + new_right_x) / 2;
		//	path.push(carMid.first,carMid.second);
		return;
	}
	/*END OF CROSSING PASSING PART*/

	/*FOR DEBUGGING*/
	if (debug) {
		char temp[100];
		sprintf(temp, "ExitEnc:%d", abs(encoder_total_exit));
		pLcd->SetRegion(Lcd::Rect(0, 0, 128, 15));
		pWriter->WriteString(temp);
		sprintf(temp, "EntEnc:%d", abs(encoder_total_round));
		pLcd->SetRegion(Lcd::Rect(0, 85, 128, 15));
		pWriter->WriteString(temp);
	}
	/*END OF DEBUGGING*/

	/*ROUNDABOUT ENTRANCE PASSING PART*/
	if (roundaboutStatus == 1) { feature = Feature::kRoundabout; }
	if (roundaboutStatus == 1
			&& abs(encoder_total_round)
	< TuningVar.round_encoder_count/*abs(System::Time() - feature_start_time) < TuningVar.feature_inside_time*/) {
		// TODO(Derppening): Figure out the use of the lines below
//		pEncoder0->Update();
//		pEncoder1->Update();
		encoder_total_round += curr_enc_val_left/*(pEncoder0->GetCount() + pEncoder1->GetCount()) / 2*/;//Because for exit/enter, the car will first left then right which destroy the encoder
		//		feature = Feature::kRoundabout;
	}

	//When exiting the roundabout, keep exit method until completely exit
	if (roundaboutExitStatus == 1
			&& abs(encoder_total_exit) >= TuningVar.roundExit_encoder_count) {
		roundaboutExitStatus = 0;
		roundaboutStatus = 0;
		/*TODO: switch carID, sendBT to another car and set has_exited on the other side to true*/
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
	if (roundaboutExitStatus == 1
			&& abs(encoder_total_exit) < TuningVar.roundExit_encoder_count) {//TODO: Be care of back turning of motor when stop will affect encoder value
		// TODO(Derppening): Figure out the use of the lines below
//		pEncoder0->Update();
//		pEncoder1->Update();
		encoder_total_exit += curr_enc_val_left/*(pEncoder0->GetCount() + pEncoder1->GetCount()) / 2*/;
		feature = Feature::kRoundaboutExit;
	}

	// When entering the roundabout, keep roundabout method until completely enter, but still keep roundaboutStatus until exit
	//	if (roundaboutStatus == 1
	//			&& abs(encoder_total_round)
	//					< TuningVar.round_encoder_count/*abs(System::Time() - feature_start_time) < TuningVar.feature_inside_time*/) {
	//		pEncoder0->Update();
	//		pEncoder1->Update();
	//		encoder_total_round += (pEncoder0->GetCount() + pEncoder1->GetCount())
	//				/ 2;
	//		feature = Feature::kRoundabout;
	//	}
	/*END OF ROUNDABOUT ENTRANCE PASSING PART*/
	/*ROUNDABOUT EXIT PASSING PART*/
	//When exiting the roundabout, keep exit method until completely exit
	//	if (roundaboutExitStatus == 1
	//			&& abs(encoder_total_exit) >= TuningVar.roundExit_encoder_count) {
	//		roundaboutExitStatus = 0;
	//	}
	//	if (roundaboutExitStatus == 1
	//			&& abs(encoder_total_exit) < TuningVar.roundExit_encoder_count) {
	//		pEncoder0->Update();
	//		pEncoder1->Update();
	//		encoder_total_exit += (pEncoder0->GetCount() + pEncoder1->GetCount())
	//				/ 2;
	//		feature = Feature::kRoundaboutExit;
	//	}
	/*END OF ROUNDABOUT EXIT PASSING PART*/
	/*OTHER CASE*/
	switch (feature) {
	case Feature::kRoundabout: {
		// Turning left at the entrance
		if (is_front_car?!roundabout_shortest(TuningVar.roundabout_shortest_flag, roundabout_cnt - 1):roundabout_shortest(TuningVar.roundabout_shortest_flag, roundabout_cnt - 1)) {
			//ensure the size of left is large enough for turning, size of left will never be 0
			while ((left_edge.points.size() < TuningVar.roundroad_min_size) && FindOneLeftEdge()) {}
			//translate right
			for (int i = 0; i < left_edge.points.size(); i++) {
				path.push(left_edge.points[i].first + TuningVar.roundabout_offset, left_edge.points[i].second);
			}

			/*OLD METHOD*/
			//			//ensure the size of left is large enough for turning, size of left will never be 0
			//			int j = 1; //track the y cor
			//			while ((j <= (TuningVar.roundroad_min_size - left_edge.points.size())) && FindOneLeftEdge()) {
			//				j++;
			//			}
			//			int i = 0;
			//			for (; i < left_edge.points.size(); i++) {
			//				// set right edge as carMid+offset before meeting black
			//				if (getWorldBit(carMid.first,left_edge.points.front().second + i) == 0) {
			//					path.push((carMid.first + TuningVar.round_enter_offset+ left_edge.points[i].first) / 2,
			//							left_edge.points[i].second); //left_edge.points[i].second == left_edge.points[i])??
			//				} else
			//					break;
			//			}
			//			//change to find edge part
			//			right_edge.points.clear();
			//			right_edge.push(carMid.first,
			//					left_edge.points.front().second + i - 1);
			//			path.push(
			//					(right_edge.points.back().first
			//							+ TuningVar.round_enter_offset
			//							+ left_edge.points[i].first) / 2,
			//					left_edge.points[i].second);
			//			i++;
			//			for (;(i < left_edge.points.size()) && FindOneRightEdge(); i++) {
			//				path.push((right_edge.points.back().first + left_edge.points[i].first) / 2, left_edge.points[i].second);
			//			}
		}
		// Turning right at the entrance
		else {
			while ( (right_edge.points.size()<TuningVar.roundroad_min_size) && FindOneRightEdge()) {}
			for(int i=0; i<right_edge.points.size(); i++){
				path.push(right_edge.points[i].first - TuningVar.roundabout_offset , right_edge.points[i].second);
			}
		}
		break;
	}
	case Feature::kRoundaboutExit: {
		// Turning left at the entrance - Turning left at the exit
		if (is_front_car?!roundabout_shortest(TuningVar.roundabout_shortest_flag, roundabout_cnt - 1):roundabout_shortest(TuningVar.roundabout_shortest_flag, roundabout_cnt - 1)) {
			//ensure the size of left is large enough for turning, size of left will never be 0
			while ((left_edge.points.size() < TuningVar.roundroad_min_size) && FindOneLeftEdge()) {}
			//translate right
			for(int i=0; i<left_edge.points.size(); i++){
				path.push(left_edge.points[i].first + TuningVar.round_exit_offset, left_edge.points[i].second);
			}
		}
		else {
			while ((right_edge.points.size() < TuningVar.roundroad_min_size) && FindOneRightEdge()) {}
			for(int i=0; i<right_edge.points.size(); i++){
				path.push(right_edge.points[i].first -TuningVar.round_exit_offset, right_edge.points[i].second);
			}
		}

		/*OLD METHOD*/
		//		if (TuningVar.roundabout_turn_left) {
		//			int i = 0;
		//			for (; i < left_edge.points.size(); i++) {
		//				// set right edge as carMid before meeting black
		//				if (getWorldBit(carMid.first,
		//						left_edge.points.front().second + i) == 0) {
		//					path.push(
		//							(carMid.first + TuningVar.round_exit_offset
		//									+ left_edge.points[i].first) / 2,
		//							left_edge.points[i].second); //left_edge.points[i].second == left_edge.points[i])??
		//				} else
		//					break;
		//			}
		//			//change to find edge part
		//			right_edge.points.clear();
		//			right_edge.push(carMid.first,
		//					left_edge.points.front().second + i - 1);
		//			path.push(
		//					(right_edge.points.back().first
		//							+ TuningVar.round_exit_offset
		//							+ left_edge.points[i].first) / 2,
		//					left_edge.points[i].second);
		//			i++;
		//			for (;(i < left_edge.points.size()) && FindOneRightEdge() ; i++) {
		//				path.push((right_edge.points.back().first + left_edge.points[i].first) / 2,
		//						left_edge.points[i].second);
		//			}
		//			// Turning right at the entrance
		//		} else {
		//			int i = 0;
		//			for (; i < right_edge.points.size(); i++) {
		//				// set left edge as carMid before meeting black
		//				if (getWorldBit(carMid.first,
		//						right_edge.points.front().second + i) == 0) {
		//					path.push(
		//							(carMid.first - TuningVar.round_exit_offset
		//									+ right_edge.points[i].first) / 2,
		//							right_edge.points[i].second); //left_edge.points[i].second == left_edge.points[i])??
		//				} else
		//					break;
		//			}
		//			/*change to find edge part*/
		//			// Push the base point first
		//			left_edge.points.clear();
		//			left_edge.push(carMid.first,
		//					right_edge.points.front().second + i - 1);
		//			path.push(
		//					(left_edge.points.back().first - TuningVar.round_exit_offset
		//							+ right_edge.points[i].first) / 2,
		//					right_edge.points[i].second);
		//			i++;
		//			for (;
		//					i < right_edge.points.size() && FindOneLeftEdge() ; i++) {
		//				path.push((left_edge.points.back().first + right_edge.points[i].first) / 2,
		//						right_edge.points[i].second);
		//			}
		//		}
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
							(WorldSize::w - left_edge.points[i].first) / 2;
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
							(WorldSize::w - left_edge.points[i].first) / 2;
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

/**
 * @brief Print sudden change track width location
 */
void PrintSuddenChangeTrackWidthLocation(uint16_t color) {
	if (has_inc_width_pt) {
		pLcd->SetRegion(
				Lcd::Rect(inc_width_pts.at(0).first,
						WorldSize::h - inc_width_pts.at(0).second - 1, 4, 4));
		pLcd->FillColor(color);
		pLcd->SetRegion(
				Lcd::Rect(inc_width_pts.at(1).first,
						WorldSize::h - inc_width_pts.at(1).second - 1, 4, 4));
		pLcd->FillColor(color);
	}
}

/*
 * @brief: return the shortest side of current roundabout
 * @return: 1 means turning left, 0 means turning right
 * */
int roundabout_shortest(uint8_t a, int pos){
	return (a >> (7-pos)) & 1;
}


/**
 * @brief Calculate the servo angle diff
 */
int16_t CalcAngleDiff() {
	int16_t error = 0, sum = 0;
	int16_t roundabout_offset = 0;
	int avg = 0;
	//	if (roundaboutStatus == 1 && abs(encoder_total_round) > TuningVar.round_encoder_count) {
	//		for (auto&& point : path.points) {
	//			if (sum > 10) //consider first 10 points
	//				break;
	//			error += (point.first + roundabout_offset - carMid.first);
	//			sum++;
	//		}
	//	}
	for (auto&& point : path.points) {
		if (sum > (roundaboutExitStatus == 1 ? 40 : 20)) //consider first 20 points
			break;
		error += (point.first - carMid.first);
		avg += point.first;
		sum++;
	}
	/*TUNNING carMid*/
	//	char temp[100];
	//	sprintf(temp, "avg: %.2f", avg/(float)sum);
	//	pLcd->SetRegion(Lcd::Rect(0, 16, 128, 15));
	//	pWriter->WriteString(temp);

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

void main_car1(bool debug_) {

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
	cameraConfig.w = CameraSize::w;
	cameraConfig.h = CameraSize::h;
	cameraConfig.fps = Ov7725Configurator::Config::Fps::kHigh;
	cameraConfig.contrast = 0x3D;
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

	AlternateMotor::Config ConfigMotor;
	ConfigMotor.id = 0;
	auto spMotor0 = util::make_unique<AlternateMotor>(ConfigMotor);
	pMotor0 = spMotor0.get();
	ConfigMotor.id = 1;
	auto spMotor1 = util::make_unique<AlternateMotor>(ConfigMotor);
	pMotor1 = spMotor1.get();

	JyMcuBt106::Config ConfigBT;
	ConfigBT.id = 0;
	ConfigBT.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	BTComm bt(ConfigBT);
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

	//Servo test
	pServo->SetDegree(servo_bounds.kLeftBound);
	System::DelayMs(1000);
	pServo->SetDegree(servo_bounds.kRightBound);
	System::DelayMs(1000);
	pServo->SetDegree(servo_bounds.kCenter);
	System::DelayMs(1000);



	//  while (true) {
	//    if (joystick.GetState() == Joystick::State::kRight) {
	//      TuningVar.roundabout_turn_left = false;
	//      break;
	//    } else if (joystick.GetState() == Joystick::State::kLeft) {
	//      break;
	//    }
	//  }

	//	while(joystick.GetState() != Joystick::State::kIdle){
	//		if(bt.hasStartReq()){
	//			break;
	//		}
	//	}

	//	StartlineOvertake();

	pMotor0->SetClockwise(true);
	pMotor1->SetClockwise(false);
	pMotor0->SetPower(210);
	pMotor1->SetPower(210);

	Timer::TimerInt startTime=System::Time();

	//	int servoAngle = 0;
	while (true) {
		while (time_img != System::Time()) {
			time_img = System::Time();
			led0.SetEnable(time_img % 500 >= 250);

			if (time_img % 10 == 0) {
				//Overtake motor control
				if (roundaboutExitStatus == 1 && stop_before_roundexit) {
					/*Consider braking*/
					pMotor0->SetPower(0);
					pMotor1->SetPower(0);
				}
				else{
					pMotor0->SetPower(210);
					pMotor1->SetPower(210);
				}


				//        Timer::TimerInt new_time = System::Time();
				Capture(); //Capture until two base points are identified
				//        if (FindStoppingLine() && time_img-startTime > 10000) {
				//          pMotor0->SetPower(0);
				//          pMotor1->SetPower(0);
				//          pWriter->WriteString("Stopping Line Detected");
				//        }
				FindEdges();
				Feature feature = featureIdent_Corner();
				GenPath(feature); //Generate path

				/*Tuning offset*/
				//		if (joystick.GetState() == Joystick::State::kUp) {
				//			servoAngle+=10;
				//		} else if (joystick.GetState() == Joystick::State::kDown) {
				//			servoAngle-=10;
				//		}
				//		pLcd->SetRegion(Lcd::Rect(0, 16, 128, 15));
				//		char timestr[100];
				//		sprintf(timestr, "AngleOS: %d", servoAngle);
				//		pServo->SetDegree(servo_bounds.kCenter + servoAngle);
				//		pWriter->WriteString(timestr);
				//		PrintWorldImage();
				//		PrintCorner(left_corners, Lcd::kPurple); //Print left_corner
				//		PrintCorner(right_corners, Lcd::kPurple); //Print right_corner
				//		pLcd->SetRegion(Lcd::Rect(carMid.first, carMid.second, 5, 5));
				//		pLcd->FillColor(Lcd::kRed);
				if (debug) {
					PrintWorldImage();
					PrintEdge(left_edge, Lcd::kRed); //Print left_edge
					PrintEdge(right_edge, Lcd::kBlue); //Print right_edge
					PrintCorner(left_corners, Lcd::kPurple); //Print left_corner
					PrintCorner(right_corners, Lcd::kPurple); //Print right_corner
					PrintEdge(path, Lcd::kGreen); //Print path
					char timestr[100];
					sprintf(timestr, "Roun_cnt: %d", roundabout_cnt);
					pWriter->WriteString(timestr);

					/*ALGO RUNNING TIME*/
					//			pLcd->SetRegion(Lcd::Rect(0, 16, 128, 15));
					//			char timestr[100];
					//			sprintf(timestr, "time: %dms", System::Time() - new_time);
					//			pWriter->WriteString(timestr);

					//
					//			    pLcd->SetRegion(Lcd::Rect(0, 80, 128, 15));
					//			    is_straight_line? pWriter->WriteString("straight"):pWriter->WriteString("Not straight");
					//          switch (feature) {
					//            case Feature::kCross:
					//              pLcd->SetRegion(Lcd::Rect(0, 0, 128, 15));
					//              pWriter->WriteString("Crossing");
					//              break;
					//            case Feature::kRoundabout:
					//              pLcd->SetRegion(Lcd::Rect(0, 60, 128, 15));
					//              pWriter->WriteString("Roundabout");
					//              break;
					//            case Feature::kNormal:
					//              pLcd->SetRegion(Lcd::Rect(0, 30, 128, 15));
					//              pWriter->WriteString("Normal");
					//              break;
					//            case Feature::kRoundaboutExit:
					//              pLcd->SetRegion(Lcd::Rect(0, 30, 128, 15));
					//              pWriter->WriteString("Exit of Roundabout");
					//              break;
					//            case Feature::kStraight:
					//              pLcd->SetRegion(Lcd::Rect(0, 30, 128, 15));
					//              pWriter->WriteString("Straight");
					//              break;
					//          }
				}
				/*END OF DEBUGGING*/

		        /* Servo PID */
				int curr_servo_error = CalcAngleDiff();

				pServo->SetDegree(util::clamp<uint16_t>(
		                servo_bounds.kCenter - (1.3 * curr_servo_error + 0 * (curr_servo_error - prev_servo_error)),
		                servo_bounds.kRightBound,
		                servo_bounds.kLeftBound));
				prev_servo_error = curr_servo_error;

				/* Motor PID */
				pEncoder0->Update();
				pEncoder1->Update();
				curr_enc_val_left = std::abs(pEncoder0->GetCount());
				curr_enc_val_right = std::abs(pEncoder1->GetCount());

			}
		}

	}

}

}  // namespace optimal
}  // namespace algorithm
}
