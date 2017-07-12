/*
 * car_manager.h
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Peter Tse (mcreng), David Mak (Derppening)
 *
 * CarManager struct
 * Arbitrary location for common definitions of structs, some constants, and
 * global-scope variable storage.
 *
 */

#ifndef CHASING17_CARMANAGER_H_
#define CHASING17_CARMANAGER_H_

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

struct CarManager final {
 public:
  /**
   * Enum of possible features
   */
  enum struct Feature : uint8_t {
    kStraight = 0,
    kRoundabout,
    kCross,
	kNormal,
	kRoundaboutExit
  };

  /**
   * Enum of possible sides
   */
  enum struct Side : uint8_t {
    kLeft = 0,
    kMiddle,
    kRight
  };

  /**
   * Enum of possible identities
   */
  enum struct Identity : bool {
    kFront = false,
    kBack = true
  };

  /**
   * Enum of speed sources, i.e. which encoder we get our data from
   */
  enum struct MotorSide {
    kLeft = 0,
    kRight,
    kBoth
  };

  /**
   * Enum of car identity
   */
  enum struct Car : bool {
    kCar1,
    kCar2
  };

  /**
   * Struct of servo boundaries, stored in servo values
   */
  struct ServoBounds {
    uint16_t kLeftBound;
    uint16_t kCenter;
    uint16_t kRightBound;
  };

  struct PidValues {
    float kP;
    float kI;
    float kD;
  };

  struct ImageSize {
    uint16_t w;
    uint16_t h;
  };

  struct PidSet {
    std::string name;

    // servo left pid values
    PidValues ServoStraightLeft;
    PidValues ServoNormalLeft;
    PidValues ServoRoundaboutLeft;
    PidValues ServoSharpTurnLeft;
    PidValues ServoTransitionalSlopeLeft;

    // servo right pid values
    PidValues ServoStraightRight;
    PidValues ServoNormalRight;
    PidValues ServoRoundaboutRight;
    PidValues ServoSharpTurnRight;
    PidValues ServoTransitionalSlopeRight;

    // target speed values
    uint16_t SpeedStraight;
    uint16_t SpeedNormal;//normal turning
    uint16_t SpeedRound;
    uint16_t SpeedSharpTurn;
    uint16_t SpeedSlow;//slow down speed during straight
    uint16_t SpeedTransitionalSlope;
    uint16_t SpeedInside;
  };

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
    void push(int x, int y) { points.push_back(std::make_pair(x, y)); }
    void push(Edges edge) { points.insert(points.end(), edge.points.begin(), edge.points.end()); }
    uint32_t size() { return points.size(); }
    void insert(int pos, int x, int y) { points.emplace(points.begin() + pos, std::make_pair(x, y)); }
    void insert(int pos, Edges edge) {
      points.insert(points.begin() + pos, edge.points.begin(), edge.points.end());
    }
    Edges grad();

    std::vector<std::pair<uint16_t, uint16_t>> points;
  };

  static uint16_t config;

  static uint16_t us_distance_;
  static int32_t left_speed_;
  static int32_t right_speed_;
  static uint16_t servo_deg_;
  static int8_t slope_deg_;
  static Side side_;
  static Feature feature_;
  static Identity identity_;
  static Car car_;

 private:
  // static class: disable ctor
  CarManager() {}
};

#endif  // CHASING17_CARMANAGER_H_
