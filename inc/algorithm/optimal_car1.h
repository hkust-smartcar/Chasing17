/*
 * optimal.h
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Peter Tse (mcreng), King Huang (XuhuaKing)
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

namespace algorithm {
namespace optimal {
namespace car1 {

void main_car1(bool debug_ = false);

/**
 * Edges struct
 *
 * An type implementation for storage of Edges
 * @member points Vector storing the edges sequentially
 * @member push(int, int) Push a std::pair<int, int> into the vector points
 * @member push(Edges) Push a Edges into the vector points
 * @member size() Return the size of vector points
 * @member insert(int, int, int) Insert a std::pair<int, int> into some position of vector points
 * @member insert(int, Edges) Insert an Edges type into some position of vector points
 * @member grad() Take the gradient of certain Edges
 */
struct Edges {
  inline void push(int x, int y) { points.push_back(std::make_pair(x, y)); }
  inline void push(Edges edge) { points.insert(points.end(), edge.points.begin(), edge.points.end()); }
  inline uint32_t size() { return points.size(); }
  inline void insert(int pos, int x, int y) { points.emplace(points.begin() + pos, std::make_pair(x, y)); }
  inline void insert(int pos, Edges edge) {
    points.insert(points.begin() + pos, edge.points.begin(), edge.points.end());
  }
  Edges grad() {
    Edges temp;
    for (int i = 1; i < this->size(); i++) {
      auto last = this->points[i];
      auto second_last = this->points[i - 1];
      temp.push(last.first - second_last.first, last.second - second_last.second);
    }
    return temp;
  }

  std::vector<std::pair<uint16_t, uint16_t>> points;
};

typedef std::list<std::pair<uint16_t, uint16_t>> Corners;

/**
 * Usage:
 * target = t;
 * left = differential_left(delta_degree) * t;
 * right = differential_left(-delta_degree) * t;
 *
 */
inline float differential_left(float x) { return 1.00716 - 0.00776897*x; }

/*CAR1*/
namespace TuningVar {
  extern bool roundabout_turn_left; //Used for GenPath()
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
  extern uint16_t angle_div_error; // translate error into angle
  extern uint16_t nearest_corner_threshold;
  extern uint16_t overtake_interval_time;

  // servo pid values
  extern float servo_straight_kp;
  extern float servo_straight_kd;
  extern float servo_normal_kd;
  extern float servo_normal_kp;
  extern float servo_roundabout_kp;
  extern float servo_roundabout_kd;
  extern float servo_sharp_turn_kp;
  extern float servo_sharp_turn_kd;
  extern float servo_roundabout_exit_kp;
  extern float servo_roundabout_exit_kd;

  // target speed values
  extern uint16_t targetSpeed_straight;
  extern uint16_t targetSpeed_normal;//normal turning
  extern uint16_t targetSpeed_round;
  extern uint16_t targetSpeed_sharp_turn;
  extern uint16_t targetSpeed_slow;//slow down speed during straight
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
