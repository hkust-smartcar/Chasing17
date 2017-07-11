/*
 * optimal.h
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Peter Tse (mcreng), King Huang (XuhuaKing), Lee Chun Hei (LeeChunHei)
 *
 * Optimal Path Algorithm Header File
 *
 */

#ifndef CHASING17_ALGORITHM_OPTIMAL_CAR1_H_
#define CHASING17_ALGORITHM_OPTIMAL_CAR1_H_

#include <cstdint>
#include <list>
#include <vector>
#include <utility>

#include "libsc/system.h"

#include "car_manager.h"

namespace algorithm {
namespace optimal {
namespace car1 {

void main_car1(bool debug_ = false);

typedef std::list<std::pair<uint16_t, uint16_t>> Corners;
typedef CarManager::Edges Edges;

/**
 * Usage:
 * target = t;
 * left = differential_left(delta_degree) * t;
 * right = differential_left(-delta_degree) * t;
 *
 */
//inline float differential_left_v1(float x) { return 1.00379358 - 0.00870460 * x; }
//inline float differential_right_v1(float x) { return 0.9962064 + 0.00870460 * x; }

inline float differential_left(float x) { return x < 0 ? 1 : (1.00379358 - 0.00870460 * x) / (0.9962064 + 0.00870460 * x); }
inline float differential_right(float x) { return x > 0 ? 1 : (0.9962064 + 0.00870460 * x) / (1.00379358 - 0.00870460 * x); }

/*CAR1*/
namespace TuningVar {
  extern bool show_algo_time;
  extern bool overtake;
  extern bool roundabout_turn_left; //Used for GenPath()
  extern bool single_car_testing;
  extern uint16_t starting_y; //the starting y for edge detection
  extern uint16_t edge_length; //max length for an edge
  extern uint16_t edge_hor_search_max; //max for horizontal search of edge if next edge point cannot be found
  extern uint16_t edge_min_worldview_bound_check; //min for worldview bound check in edge finding
  extern uint16_t corner_range; //the square for detection would be in size corener_range*2+1
  extern float corner_height_ratio; //the max height for detection would be WorldSize.h/corner_height_ratio
  extern uint16_t corner_min, corner_max; //threshold (in %) for corner detection
  extern uint16_t min_corners_dist; // Manhattan dist threshold for consecutive corners
  extern uint16_t min_edges_dist; // Manhattan dist threshold for edges
  extern uint16_t track_width_threshold; //track width threshold for consideration of sudden change (square)
  extern uint16_t track_width_change_threshold; //track width change threshold for consideration of sudden change
  extern uint16_t testDist; // The distance from which the image pixel should be tested and identify feature
  extern uint16_t slowDownDist; // the distance from which the image pixel should be tested and know whether it should slow down in advance
  extern uint16_t sightDist_exitRound; //The distance from which the image pixel is used for exit testing
  extern uint16_t straight_line_threshold; // The threshold num. of equal width for straight line detection
  extern uint16_t action_distance; // The condition in which the car start handling this feature when meeting it
  extern uint16_t stop_distance; // The distance away from starting line - for stopping
  extern libsc::Timer::TimerInt feature_inside_time; // freezing time for feature extraction, the time for entering the entrance
  extern uint16_t cross_cal_start_num;
  extern uint16_t cross_cal_ratio; //Look forward @cross_cal_start_num - encoder_total/@cross_cal_ratio to determine path
  extern uint16_t general_cal_num; //The num of path points considered for servo angle decision except crossing
  extern uint16_t cross_encoder_count; // The hardcoded encoder count that car must reach in crossroad
  extern uint16_t round_enter_offset;
  extern uint16_t servo_offset; //49
  extern uint16_t min_dist_meet_crossing;
  extern uint16_t roundroad_min_size; // When the edge is broken in roundabout, find until this threshold
  extern uint16_t roundroad_exit_radius; // search pixels around to double check exit of roundabout for CAR1
  extern uint16_t exit_action_dist; // double check to avoid corner's sudden disappear inside roundabout
  extern uint16_t roundabout_offset; // half of road width
  extern uint16_t round_exit_offset;
  extern uint16_t round_encoder_count;
  extern uint16_t roundExit_encoder_count;
  extern int32_t roundabout_shortest_flag; //1 means turn left, 0 means turn right. Reading from left to right
  extern int32_t roundabout_overtake_flag;//1 means overtake, 0 means ignore overtake
  extern uint16_t angle_div_error; // translate error into angle
  extern uint16_t nearest_corner_threshold;
  extern uint16_t overtake_interval_time;

  // servo right pid values
  extern float servo_straight_kp_right;
  extern float servo_straight_kd_right;
  extern float servo_normal_kd_right;
  extern float servo_normal_kp_right;
  extern float servo_roundabout_kp_right;
  extern float servo_roundabout_kd_right;
  extern float servo_sharp_turn_kp_right;
  extern float servo_sharp_turn_kd_right;
  extern float servo_trans_kp_slope_right;
  extern float servo_trans_kd_slope_right;

  // servo left pid values
  extern float servo_straight_kp_left;
  extern float servo_straight_kd_left;
  extern float servo_normal_kd_left;
  extern float servo_normal_kp_left;
  extern float servo_roundabout_kp_left;
  extern float servo_roundabout_kd_left;
  extern float servo_sharp_turn_kp_left;
  extern float servo_sharp_turn_kd_left;
  extern float servo_trans_kp_slope_left;
  extern float servo_trans_kd_slope_left;

  // target speed values
  extern uint16_t targetSpeed_straight;
  extern uint16_t targetSpeed_normal;//normal turning
  extern uint16_t targetSpeed_round;
  extern uint16_t targetSpeed_sharp_turn;
  extern uint16_t targetSpeed_slow;//slow down speed during straight
  extern uint16_t targetSpeed_trans;
}  // namespace TuningVar

/**
 * TranslateType enum struct
 *
 * An enum struct implementation designed for GenPath()
 */
enum struct TranslateType {
  kNone = 0, kLeftNull, kRightNull
};

}  // namespace car1
}  // namespace optimal
}  // namespace algorithm
#endif //CHASING17_ALGORITHM_OPTIMAL_CAR1_H_
