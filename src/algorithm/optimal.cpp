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

#include "algorithm/optimal.h"

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
#include "libutil/misc.h"

#include "bluetooth.h"
#include "car_manager.h"
#include "util/util.h"

#include "debug_console.h"
#include "algorithm/worldview/car1.h"
#include "algorithm/worldview/car2.h"

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

namespace {
Edges left_edge;
Edges right_edge;
Edges path;
Corners left_corners;
Corners right_corners;
std::array<std::pair<uint16_t, uint16_t>, 2> inc_width_pts; //0:left, 1:right
bool has_inc_width_pt = false;
bool is_straight_line = false;
bool is_start_line = false;
bool stop_the_car_on_start_line = false;
int roundaboutStatus = 0; // 0: Before 1: Detected 2: Inside (After one corner)
int crossingStatus = 0; // 0: Before 1: Detected/Inside
uint16_t prev_track_width = 0;
uint16_t corner_temp_x; //Remember the enter corner_cor after entering the crossing
uint16_t corner_temp_y;
int encoder_total = 0; //for crossroad
Timer::TimerInt feature_start_time;
std::pair<int, int> carMid(WorldSize::w / 2, 0);
const Byte* CameraBuf;
//Byte WorldBuf[128*20];
std::unique_ptr<Ov7725> pCamera = nullptr;
St7735r* pLcd = nullptr;
Led* pLed3 = nullptr;
LcdTypewriter* pWriter = nullptr;
CarManager::ServoBounds* pServoBounds = nullptr;
FutabaS3010* pServo = nullptr;
JyMcuBt106* pBT = nullptr;
DirEncoder* pEncoder0 = nullptr;
DirEncoder* pEncoder1 = nullptr;

CarManager::ServoBounds servo_bounds;
CarManager::Car car;

inline constexpr int max(int a, int b) { return (a > b) ? a : b; }
inline constexpr int min(int a, int b) { return (a < b) ? a : b; }

// for edge finding, in CCW dir
const int8_t dx[9] = {0, -1, -1, -1, 0, 1, 1, 1, 0};
const int8_t dy[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};

// prototype declarations
int16_t CalcAngleDiff();
void Capture();
CarManager::Feature featureIdent_Corner();
CarManager::Feature featureIdent_Width();
int FindDirection(int, int);
bool FindEdges();
bool FindOneLeftEdge();
bool FindOneRightEdge();
void GetPath(CarManager::Feature);
bool getWorldBit(int, int);
void PrintCorner(Corners, uint16_t);
void PrintEdge(Edges, uint16_t);
void PrintImage();
void PrintSuddenChangeTrackWidthLocation(uint16_t);
void PrintWorldImage();

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

/**
 * @brief Consider the left/right of vector at current point pointing to next point
 * @param j direction
 * @param d original left/right-ness
 * @return new left/right-ness
 */
int FindDirection(int j, int d) {
  j %= 8;
  if (j > 0 && j < 4)
    return -1;
  if (j > 4)
    return 1;
  if (j == 4)
    return d;
  return 4;
}

//test distortion world view
//w_x,w_y world coordinate 128*160
//i_x,i_y image coordinate 128*480

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
  switch (car) {
    case CarManager::Car::kCar1: {
      w_y = 160 - w_y;
      int i_x, i_y;
      i_x = worldview::car1::transformMatrix[w_x][w_y][0];
      i_y = worldview::car1::transformMatrix[w_x][w_y][1];
      return getFilteredBit(CameraBuf, i_x, i_y);
    }
      break;
    case CarManager::Car::kCar2: {
      w_y = 160 - w_y;
      int i_x, i_y;
      i_x = worldview::car2::transformMatrix[w_x][w_y][0];
      i_y = worldview::car2::transformMatrix[w_x][w_y][1];
      return getFilteredBit(CameraBuf, i_x, i_y);
    }
      break;
  }

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
void Capture() {
  left_edge.points.clear();
  right_edge.points.clear();
  bool found_left = false, found_right = false;
  int left_x = -1, left_y = -1;
  int right_x = -1, right_y = -1;
//		while(!pCamera->IsAvailable());
  CameraBuf = pCamera->LockBuffer();
  pCamera->UnlockBuffer();

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
  pLcd->FillBits(0x0000, 0xFFFF, CameraBuf, pCamera->GetBufferSize() * 8);
}

bool FindOneLeftEdge() {
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
         i < min(prev_x + TuningVar.edge_hor_search_max, WorldSize::w - 1);
         i++) {
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

//	if (left_edge.points.back().second > TuningVar.edge_min_worldview_bound_check){
  switch (car) { //reaches worldview boundaries
    case CarManager::Car::kCar1:
      if (worldview::car1::transformMatrix[min(
          left_edge.points.back().first + 1, WorldSize::w - 1)][WorldSize::h
          - left_edge.points.back().second][0] == -1) {
//        left_edge.points.pop_back();
        return false;
      }
      if (worldview::car1::transformMatrix[max(
          left_edge.points.back().first - 1, 1)][WorldSize::h
          - left_edge.points.back().second][0] == -1) {
//        left_edge.points.pop_back();
        return false;
      }
      break;
    case CarManager::Car::kCar2:
      if (worldview::car2::transformMatrix[min(
          left_edge.points.back().first + 1, WorldSize::w - 1)][WorldSize::h
          - left_edge.points.back().second][0] == -1) {
//        left_edge.points.pop_back();
        return false;
      }
      if (worldview::car2::transformMatrix[max(
          left_edge.points.back().first - 1, 1)][WorldSize::h
          - left_edge.points.back().second][0] == -1) {
//        left_edge.points.pop_back();
        return false;
      }
      break;
  }
//	}

  //find corners
  if (left_edge.points.back().second
      <= WorldSize::h / TuningVar.corner_height_ratio) {
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

  return true;
}

bool FindOneRightEdge() {
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
      for (int i = prev_x; i < min(prev_x + TuningVar.edge_hor_search_max, WorldSize::w - 1); i++) {
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

  if (right_edge.points.back().second
      > TuningVar.edge_min_worldview_bound_check) {
    switch (car) { //reaches worldview boundaries
      case CarManager::Car::kCar1:
        if (worldview::car1::transformMatrix[min(
            right_edge.points.back().first + 1, WorldSize::w - 1)][WorldSize::h
            - right_edge.points.back().second][0] == -1) {
//          right_edge.points.pop_back();
          return false;
        }
        if (worldview::car1::transformMatrix[max(
            right_edge.points.back().first - 1, 1)][WorldSize::h
            - right_edge.points.back().second][0] == -1) {
//          right_edge.points.pop_back();
          return false;
        }
        break;
      case CarManager::Car::kCar2:
        if (worldview::car2::transformMatrix[min(
            right_edge.points.back().first + 1, WorldSize::w - 1)][WorldSize::h
            - right_edge.points.back().second][0] == -1) {
//          right_edge.points.pop_back();
          return false;
        }
        if (worldview::car2::transformMatrix[max(
            right_edge.points.back().first - 1, 1)][WorldSize::h
            - right_edge.points.back().second][0] == -1) {
//          right_edge.points.pop_back();
          return false;
        }
        break;
    }
  }

  //find corners
  if (right_edge.points.back().second
      <= WorldSize::h / TuningVar.corner_height_ratio) {
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
  is_start_line = false;
  left_corners.points.clear();
  right_corners.points.clear();
  bool flag_break_left = left_edge.points.size() == 0;
  bool flag_break_right = right_edge.points.size() == 0;
  uint16_t staright_line_edge_count = 0; // Track the num. of equal width
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
      for (int i = 0; i < min(10, min(right_edge.size() / 2, left_edge.size() / 2)); i++) { //discard last 10 points
        right_edge.points.pop_back();
        left_edge.points.pop_back();
      }
      flag_break_left = flag_break_right = true;
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
//      char temp[100];
//      sprintf(temp, "_dist:%d", dist - prev_track_width);
//      pLcd->SetRegion(Lcd::Rect(0, 54, 128, 15));
//      pWriter->WriteString(temp);

      //Straight line + Starting line judgement
      if (left_edge.points.size() > 1 && right_edge.points.size() > 1
          && abs(dist - prev_track_width) < 3) {
        staright_line_edge_count++;
        //Starting line judgement
        uint16_t black_count = 0;
        for (uint16_t i = left_edge.points.front().first;
             i < right_edge.points.front().first; i++) {
          if (getWorldBit(i,
                          (left_edge.points.front().second
                              + right_edge.points.front().second) / 2)
              == 1) {
            black_count++;
          }
        }
        //calculate the ratio
        if (black_count
            / abs(
                left_edge.points.front().first
                    - right_edge.points.front().first)
            > TuningVar.black_div_length_ratio_threshold) {
          is_start_line = true;
        }
      }
      if (dist >= TuningVar.track_width_threshold
          && dist - prev_track_width >= TuningVar.track_width_change_threshold) {
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
 * @brief Feature Identification by edges width
 *
 * Algorithm:
 * 1. Width increases suddenly (inc_width_pts serves as "corners")
 * 2. Perpendicular direction, search points @sightDistance away.
 * 3. Black (1) is roundabout, White (0) is crossing
 *
 * @return: Feature: kCrossing, kRound...
 * @note: Execute this function after calling FindEdges() only when width sudden increase is detected
 */
CarManager::Feature featureIdent_Width() {
  std::pair<int, int> carMid(WorldSize::w / 2, 0);
  std::pair<int, int> cornerMid;
  //Width increase case
  if (has_inc_width_pt) {
    //TODO: The conditions ensuring the width increase is reasonable case
    if (true) {
      int cornerMid_x = (inc_width_pts.at(0).first
          + inc_width_pts.at(1).first) / 2; //corner midpoint x-cor
      int cornerMid_y = (inc_width_pts.at(0).second
          + inc_width_pts.at(1).second) / 2; //corner midpoint y-cor
      int edge3th = sqrt(
          pow(cornerMid_x - carMid.first, 2)
              + pow(cornerMid_y - carMid.second, 2)); //Third edge of right triangle
      int test_x = TuningVar.sightDist
          * ((cornerMid_x - carMid.first) / edge3th) + cornerMid_x;
      int test_y = TuningVar.sightDist
          * ((cornerMid_y - carMid.second) / edge3th) + cornerMid_y;
      if (getWorldBit(test_x, test_y) && getWorldBit(test_x + 1, test_y)
          && getWorldBit(test_x, test_y + 1)
          && getWorldBit(test_x - 1, test_y)) {
        //All black
        return CarManager::Feature::kRoundabout;
      } else if (!getWorldBit(test_x, test_y)
          && !getWorldBit(test_x + 1, test_y)
          && !getWorldBit(test_x, test_y + 1)
          && !getWorldBit(test_x - 1, test_y)) {
        return CarManager::Feature::kCross;
      }
    }
      //Special case: Enter crossing with extreme angle - Two corners are on the same side / Only one corner
    else
      return CarManager::Feature::kRoundaboutExit;
  }

  //Return kNormal and wait for next testing
  return CarManager::Feature::kNormal;
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
CarManager::Feature featureIdent_Corner() {
  /*FOR DEBUGGING*/
//	pLcd->SetRegion(
//			Lcd::Rect(carMid.first, WorldSize::h - carMid.second - 1, 2, 2));
//	pLcd->FillColor(Lcd::kRed);
  /*END OF DEBUGGING*/
  //1. Straight line
  if (is_straight_line) {
    roundaboutStatus = 0;// Avoid failure of detecting roundabout exit
    return CarManager::Feature::kStraight;
  }

  //2. Start line
  if (is_start_line) {
    roundaboutStatus = 0;// Avoid failure of detecting roundabout exit
    return CarManager::Feature::kStart;
  }

  /*Only assume the first two corners are useful*/
  if (left_corners.points.size() > 0 && right_corners.points.size() > 0) {
    /*TODO: Double check corner "candidates" - slope<=0
     std::vector<std::pair<uint16_t, uint16_t>>::iterator it;
     //Check left edge
     it = find (left_edge.points.begin(),left_edge.points.end(), right_corners.points.front());
     std::pair<uint16_t, uint16_t> edge_corner_front;
     std::pair<uint16_t, uint16_t> edge_corner_back;
     if (it != left_edge.points.end() && (it+3 < left_edge.points.end()) && (it-3 > left_edge.points.begin())){
     edge_corner_front  = *(it+3);
     edge_corner_back = *(it-3);
     }
     //Check right edge
     **/

    //3. More than two valid corner case
    uint16_t cornerMid_x = (left_corners.points.front().first
        + right_corners.points.front().first) / 2; //corner midpoint x-cor
    uint16_t cornerMid_y = (left_corners.points.front().second
        + right_corners.points.front().second) / 2; //corner midpoint y-cor
    /*FOR DEBUGGING*/
//		pLcd->SetRegion(
//				Lcd::Rect(cornerMid_x, WorldSize::h - cornerMid_y - 1, 2, 2));
//		pLcd->FillColor(Lcd::kRed);
    /*END OF DEBUGGING*/
    if (abs(carMid.second - cornerMid_y) > TuningVar.action_distance) {
      return CarManager::Feature::kNormal;
    }
    uint16_t edge3th = sqrt(
        pow(abs(cornerMid_x - carMid.first), 2)
            + pow(abs(cornerMid_y - carMid.second), 2)); //Third edge of right triangle
    uint16_t test_x = TuningVar.sightDist
        * ((cornerMid_x - carMid.first) / edge3th) + cornerMid_x; //'-': The image is in opposite direction
    uint16_t test_y = TuningVar.sightDist
        * ((cornerMid_y - carMid.second) / edge3th) + cornerMid_y;
    /*FOR DEBUGGING*/
//		pLcd->SetRegion(Lcd::Rect(test_x, WorldSize::h - test_y - 1, 4, 4));
//		pLcd->FillColor(Lcd::kRed);
    /*END OF DEBUGGING*/
    if (getWorldBit(test_x, test_y) && getWorldBit(test_x + 1, test_y)
        && getWorldBit(test_x, test_y + 1)
        && getWorldBit(test_x - 1, test_y)) {
      //All black
      roundaboutStatus = 1; //Detected
      feature_start_time = System::Time(); // Mark the startTime of latest enter time
      return CarManager::Feature::kRoundabout;
    } else if (!getWorldBit(test_x, test_y)
        && !getWorldBit(test_x + 1, test_y)
        && !getWorldBit(test_x, test_y + 1)
        && !getWorldBit(test_x - 1, test_y)
		&& crossingStatus == 0) // avoid double check for crossing when inside the crossing (encoder_total<2500)
    {
      crossingStatus = 1; //Detected
      corner_temp_x = cornerMid_x; //store the entering corners
	  corner_temp_y = cornerMid_y;
      feature_start_time = System::Time(); // Mark the startTime of latest enter time
      pEncoder0->Update();
      encoder_total = 0;
      roundaboutStatus = 0;// Avoid failure of detecting roundabout exit
      return CarManager::Feature::kCross;
    }
  }

    //4. Only one corner case:Only one corner + Front is black
  else if ((left_corners.points.size() > 0 || right_corners.points.size() > 0)
      && getWorldBit(carMid.first, carMid.second + TuningVar.sightDist_exitRound) == 1) {
//		//First time: Inside
//		if(roundaboutStatus == 1){
//			roundaboutStatus = 2;
//		}
//		//Second time: Exit
    if (roundaboutStatus == 1 && abs(System::Time() - feature_start_time) > TuningVar.feature_inside_time) {
      roundaboutStatus = 0;
      return CarManager::Feature::kRoundaboutExit;
    }
  }

  /*Double check for crossing to handle only one corner case*/
  else if(roundaboutStatus == 0 && crossingStatus == 0){ // avoid double check for crossing when inside the crossing (encoder_total<2500)){ //Not inside roundabout, not Exit of Roundabout when encounter one corner case - CONDITION_1
	  //Both sides are break due to -1 - CONDITION_2
	  bool both_break = false;
	  switch (car) { //reaches worldview boundaries
	    case CarManager::Car::kCar1:
	    	//right edge touch right boundary
	      if ((worldview::car1::transformMatrix[min(right_edge.points.back().first + 1, WorldSize::w - 1)][WorldSize::h - right_edge.points.back().second][0] == -1)
	    		  && (worldview::car1::transformMatrix[max(left_edge.points.back().first - 1, 1)][WorldSize::h - left_edge.points.back().second][0] == -1)//left edge touch left boundary
				  ) {
	    	  both_break = true;
	      }
	      break;
	    case CarManager::Car::kCar2:
	      if ((worldview::car2::transformMatrix[min(right_edge.points.back().first + 1, WorldSize::w - 1)][WorldSize::h - right_edge.points.back().second][0] == -1)
	    		  && (worldview::car2::transformMatrix[max(left_edge.points.back().first - 1, 1)][WorldSize::h - left_edge.points.back().second][0] == -1)
				  ) {
	    	  both_break = true;
	      }
	      break;
	  }
	  /*FOR DEBUGGING*/
//	  pLcd->SetRegion(Lcd::Rect(0,16,128,15));
//	  (both_break)?(pWriter->WriteString("both break")):(pWriter->WriteString("Not both break"));
	  if(!both_break){
		  return CarManager::Feature::kNormal;
	  }
	  // Only one corner - CONDITION_3
	  if(left_corners.points.size() > 0 || right_corners.points.size() > 0){
		  pEncoder0->Update();
	      crossingStatus = 1; //Detected
	      encoder_total = 0;
		  return CarManager::Feature::kCross;
	  }
//	  //Only left corner
//	  if(left_corners.points.size() > 0){
//		  //push the midpoint of right edge into corner
//	  }
//	  //Only right corner
//	  if(right_corners.points.size() > 0){
//
//	  }
  }

  //5. Nothing special: Return kNormal and wait for next testing
  return CarManager::Feature::kNormal;
  //TODO: Enter crossing with extreme angle - Two corners are on the same side
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
 * 5. kStart:
 *  - Set the stop_the_car_on_start_line = true;
 *
 * TODO (mcreng): Naive psuedo-optimal path ("Curve fitting")
 *
 * Points to take:
 * 1. Current position (width/2, 0)
 * 2. Start/End points of shifted curve due to LEFT_NULL or RIGHT_NULL
 * 3. Under no LEFT_NULL and RIGHT_NULL, the midpt's midpt
 */
void GenPath(CarManager::Feature feature) {
  int left_size = left_edge.size();
  int right_size = right_edge.size();

  path.points.clear();

  if (!left_size && !right_size) { //simple validity check
    return;
  }
  /*FOR DEBUGGING*/
//  char temp[100];
//  sprintf(temp, "Enc:%d", encoder_total);
//  pLcd->SetRegion(Lcd::Rect(0, 32, 128, 15));
//  pWriter->WriteString(temp);
  if (crossingStatus == 1 && encoder_total >= TuningVar.cross_encoder_count/*abs(System::Time() - feature_start_time) > TuningVar.feature_inside_time*/) {//&& encoder pass crossing
	  crossingStatus = 0;
  }
  if (crossingStatus == 1) { //Gen new path by searching midpoint
	pEncoder0->Update();
	encoder_total += pEncoder0->GetCount();
	uint16_t new_right_x;
	uint16_t new_left_x;
	// find new right edge
	for (uint16_t i = carMid.first; i < WorldSize::w ; i++){
		if(getWorldBit(i, carMid.second+ (TuningVar.cross_cal_start_num - encoder_total/TuningVar.cross_cal_ratio)) == 1){
			new_right_x = i;
			break;
		}
	}
	// find new left edge
	for (uint16_t i = carMid.first; i > 0 ; i--){
		if(getWorldBit(i, carMid.second+ (TuningVar.cross_cal_start_num - encoder_total/TuningVar.cross_cal_ratio)) == 1){
			new_left_x = i;
			break;
		}
	}
    /*FOR DEBUGGING*/
//		pLcd->SetRegion(Lcd::Rect((new_left_x + new_right_x)/2,WorldSize::h - (carMid.second+ (TuningVar.cross_cal_start_num - encoder_total/TuningVar.cross_cal_ratio)) - 1, 4, 4));
//		pLcd->FillColor(Lcd::kRed);
    /*END OF DEBUGGING*/
    path.push((new_left_x + new_right_x)/2, carMid.second+ (TuningVar.cross_cal_start_num - encoder_total/TuningVar.cross_cal_ratio)); //follow the new midpoint
//	  path.push(carMid.first,carMid.second);
    return;
  }

  // When entering the roundabout, keep roundabout method until completely enter TODO: change to encoder value
  if (roundaboutStatus == 1 && abs(System::Time() - feature_start_time) < TuningVar.feature_inside_time) {
    feature = CarManager::Feature::kRoundabout;
  }

  switch (feature) {
    case CarManager::Feature::kNormal:
    case CarManager::Feature::kStraight: {
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
            shift_right_null = (WorldSize::w - left_edge.points[i].first)
                / 2;
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
            shift_right_null = (WorldSize::w - left_edge.points[i].first)
                / 2;
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
    case CarManager::Feature::kCross: {
      uint16_t cornerMid_x = (left_corners.points.front().first
          + right_corners.points.front().first) / 2; //corner midpoint x-cor
      uint16_t cornerMid_y = (left_corners.points.front().second
          + right_corners.points.front().second) / 2; //corner midpoint y-cor
      path.push(cornerMid_x, cornerMid_y);
      path.push(carMid.first, carMid.second);
      break;
    }
    case CarManager::Feature::kRoundabout: {
      // Turning left at the entrance
      if (TuningVar.roundabout_turn_left) {
        int i = 0;
        for (; i < left_edge.points.size(); i++) {
          // set right edge as carMid before meeting black
          if (getWorldBit(carMid.first,
                          left_edge.points.front().second + i) == 0) {
            path.push((carMid.first + left_edge.points[i].first) / 2,
                      left_edge.points[i].second); //left_edge.points[i].second == left_edge.points[i])??
          } else
            break;
        }
        //change to find edge part
        right_edge.points.clear();
        right_edge.push(carMid.first, left_edge.points.front().second + i);
        path.push(
            (right_edge.points.back().first + left_edge.points[i].first)
                / 2, left_edge.points[i].second);
        i++;
        for (; i < left_edge.points.size() && FindOneRightEdge(); i++) {
          path.push((right_edge.points.back().first + left_edge.points[i].first) / 2, left_edge.points[i].second);
        }
        // Turning right at the entrance
      } else {
        int i = 0;
        for (; i < right_edge.points.size(); i++) {
          // set left edge as carMid before meeting black
          if (getWorldBit(carMid.first, right_edge.points.front().second + i) == 0) {
            path.push((carMid.first + right_edge.points[i].first) / 2,
                      right_edge.points[i].second); //left_edge.points[i].second == left_edge.points[i])??
          } else
            break;
        }
        /*change to find edge part*/
        // Push the base point first
        left_edge.points.clear();
        left_edge.push(carMid.first, right_edge.points.front().second + i);
        path.push((left_edge.points.back().first + right_edge.points[i].first) / 2, right_edge.points[i].second);
        i++;
        for (; i < right_edge.points.size() && FindOneLeftEdge(); i++) {
          path.push((left_edge.points.back().first + right_edge.points[i].first) / 2, right_edge.points[i].second);
        }
      }
      break;
    }
    case CarManager::Feature::kRoundaboutExit: {
      // Turning left at the entrance - Turning left at the exit
      if (TuningVar.roundabout_turn_left) {
        int i = 0;
        for (; i < left_edge.points.size(); i++) {
          // set right edge as carMid before meeting black
          if (getWorldBit(carMid.first,
                          left_edge.points.front().second + i) == 0) {
            path.push((carMid.first + left_edge.points[i].first) / 2,
                      left_edge.points[i].second); //left_edge.points[i].second == left_edge.points[i])??
          } else
            break;
        }
        //change to find edge part
        right_edge.points.clear();
        right_edge.push(carMid.first, left_edge.points.front().second + i);
        path.push(
            (right_edge.points.back().first + left_edge.points[i].first)
                / 2, left_edge.points[i].second);
        i++;
        for (; i < left_edge.points.size() && FindOneRightEdge(); i++) {
          path.push((right_edge.points.back().first + left_edge.points[i].first) / 2, left_edge.points[i].second);
        }
        // Turning right at the entrance
      } else {
        int i = 0;
        for (; i < right_edge.points.size(); i++) {
          // set left edge as carMid before meeting black
          if (getWorldBit(carMid.first, right_edge.points.front().second + i) == 0) {
            path.push((carMid.first + right_edge.points[i].first) / 2,
                      right_edge.points[i].second); //left_edge.points[i].second == left_edge.points[i])??
          } else
            break;
        }
        /*change to find edge part*/
        // Push the base point first
        left_edge.points.clear();
        left_edge.push(carMid.first, right_edge.points.front().second + i);
        path.push((left_edge.points.back().first + right_edge.points[i].first) / 2, right_edge.points[i].second);
        i++;
        for (; i < right_edge.points.size() && FindOneLeftEdge(); i++) {
          path.push((left_edge.points.back().first + right_edge.points[i].first) / 2, right_edge.points[i].second);
        }
      }
      break;
    }
    case CarManager::Feature::kStart: {
      stop_the_car_on_start_line = false;
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

/**
 * @brief Calculate the servo angle diff
 */
int16_t CalcAngleDiff() {
  int16_t error = 0, sum = 0;
  for (auto&& point : path.points) {
    if (sum > 20) //consider first 20 points
      break;
    error += (point.first - WorldSize::w / 2);
    sum++;
  }

//	char buff[10];
//	sprintf(buff, "Servo:%d", error);
//	pLcd->SetRegion(Lcd::Rect(0, 0, 100, 15));
//	pWriter->WriteBuffer(buff, 10);

  return error / sum * 20;
}

}  // namespace

void main(CarManager::Car c) {
  CarManager::Config car_config;
  car_config.car = c;

//  car = c;
//  servo_bounds = car == CarManager::Car::kCar1 ? CarManager::kBoundsCar1 : CarManager::kBoundsCar2;

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
  cameraConfig.contrast = 0x2C;
  cameraConfig.brightness = 0x00;
  std::unique_ptr<Ov7725> camera = util::make_unique<Ov7725>(cameraConfig);
  pCamera = std::move(camera);
  pCamera->Start();

  FutabaS3010::Config ConfigServo;
  ConfigServo.id = 0;
  auto pServo = util::make_unique<FutabaS3010>(ConfigServo);
//  FutabaS3010 servo(ConfigServo);
//	std::unique_ptr<FutabaS3010> pServo(new FutabaS3010(ConfigServo));
//  pServo = &servo;

  DirEncoder::Config ConfigEncoder;
  ConfigEncoder.id = 0;
  DirEncoder encoder0(ConfigEncoder);
  pEncoder0 = &encoder0;
//  auto pEncoder0 = util::make_unique<DirEncoder>(ConfigEncoder);
  ConfigEncoder.id = 1;
  DirEncoder encoder1(ConfigEncoder);
  pEncoder1 = &encoder1;
//  auto pEncoder1 = util::make_unique<DirEncoder>(ConfigEncoder);

  AlternateMotor::Config ConfigMotor;
  ConfigMotor.id = 0;
  AlternateMotor motor0(ConfigMotor);
//  auto pMotor0 = util::make_unique<AlternateMotor>(ConfigMotor);
  ConfigMotor.id = 1;
  AlternateMotor motor1(ConfigMotor);
//  auto pMotor1 = util::make_unique<AlternateMotor>(ConfigMotor);
  motor0.SetPower(200);
  motor1.SetPower(200);
  motor1.SetClockwise(false);



//  auto pMpc = util::make_unique<util::MpcDual>(pMotor0.get(), pMotor1.get(), pEncoder0.get(), pEncoder1.get());
//  util::MpcDual mpc(&motor0, &motor1, &encoder0, &encoder1);

  JyMcuBt106::Config ConfigBT;
  ConfigBT.id = 0;
  ConfigBT.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
  JyMcuBt106 bt(ConfigBT);
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

  /*CarManager::Config ConfigMgr;
  ConfigMgr.servo = std::move(pServo);
  ConfigMgr.epc = std::move(pMpc);
  CarManager::Init(std::move(ConfigMgr));*/
//  car_config.servo = std::move(pServo);
//  car_config.epc = std::move(pMpc);
//  CarManager::Init(std::move(car_config));

  Timer::TimerInt time_img = 0;

  servo_bounds = CarManager::GetServoBounds();

  //Servo test
  pServo->SetDegree(servo_bounds.kLeftBound);
//  CarManager::SetTargetAngle(CarManager::GetServoAngles().kLeftAngle);
  System::DelayMs(1000);
  pServo->SetDegree(servo_bounds.kRightBound);
//  CarManager::SetTargetAngle(CarManager::GetServoAngles().kRightAngle);
  System::DelayMs(1000);
  pServo->SetDegree(servo_bounds.kCenter);
//  CarManager::SetTargetAngle(0);
  System::DelayMs(1000);

//  CarManager::SetTargetSpeed(8000);
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

  while (true) {
    while (time_img != System::Time()) {
      time_img = System::Time();
      led0.SetEnable(time_img % 500 >= 250);

      if (time_img % 10 == 0) {
//        Timer::TimerInt new_time = System::Time();
        //pMpc->UpdateEncoder();
        Capture(); //Capture until two base points are identified
        FindEdges();
        CarManager::Feature a = featureIdent_Corner();
        GenPath(a); //Generate path
        /*FOR DEBUGGING*/
//				pLcd->SetRegion(Lcd::Rect(0, 16, 128, 15));
//				char timestr[100];
//				sprintf(timestr, "time: %dms", System::Time() - new_time);
//				pLcd->SetRegion(Lcd::Rect(0, 32, 128, 15));
//				sprintf(timestr, "feature: %dms", feature_start_time);
//				pLcd->SetRegion(Lcd::Rect(0, 64, 128, 15));
//				(crossingStatus == 1)?pWriter->WriteString("Inside"):pWriter->WriteString("Before");
//				PrintWorldImage();
//				PrintEdge(left_edge, Lcd::kRed); //Print left_edge
//				PrintEdge(right_edge, Lcd::kBlue); //Print right_edge
//				PrintCorner(left_corners, Lcd::kPurple); //Print left_corner
//				PrintCorner(right_corners, Lcd::kPurple); //Print right_corner
//
//				switch (a) {
//				case CarManager::Feature::kCross:
//					pLcd->SetRegion(Lcd::Rect(0, 0, 128, 15));
//					pWriter->WriteString("Crossing");
//					break;
//				case CarManager::Feature::kRoundabout:
//					pLcd->SetRegion(Lcd::Rect(0, 0, 128, 15));
//					pWriter->WriteString("Roundabout");
//					break;
//				case CarManager::Feature::kNormal:
//					pLcd->SetRegion(Lcd::Rect(0, 0, 128, 15));
//					pWriter->WriteString("Normal");
//					break;
//				case CarManager::Feature::kRoundaboutExit:
//					pLcd->SetRegion(Lcd::Rect(0, 0, 128, 15));
//					pWriter->WriteString("Exit of Roundabout");
//					break;
//				case CarManager::Feature::kStart:
//					pLcd->SetRegion(Lcd::Rect(0, 0, 128, 15));
//					pWriter->WriteString("Starting line----");
//					break;
//				case CarManager::Feature::kStraight:
//					pLcd->SetRegion(Lcd::Rect(0, 0, 128, 15));
//					pWriter->WriteString("Straight");
//					break;
//				}
//				PrintSuddenChangeTrackWidthLocation(Lcd::kYellow); //Print sudden change track width location
        /*END OF DEBUGGING*/
        // TODO: Modify CalcAngleDiff() to return angle difference, instead of servo value difference
        pServo->SetDegree(libutil::ClampVal(servo_bounds.kRightBound,static_cast<uint16_t>(servo_bounds.kCenter - 1.3*CalcAngleDiff()),

                                      servo_bounds.kLeftBound));
       /*MOTOR PROTECTION*/
//        encoder0.Update();
//        encoder1.Update();
//       if(std::abs(encoder0.GetCount())<10||std::abs(encoder1.GetCount())<10){
//    	   motor0.SetPower(0);
//    	   motor1.SetPower(0);
//       }else{
//    	   motor0.SetPower(300);
//    	   motor1.SetPower(300);
//       }
//		PrintEdge(path, Lcd::kGreen); //Print path
//        CarManager::UpdateParameters();
      }
    }

  }

}

}  // namespace optimal
}  // namespace algorithm
