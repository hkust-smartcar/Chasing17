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

    // servo right pid values
    PidValues ServoStraightRight;
    PidValues ServoNormalRight;
    PidValues ServoRoundaboutRight;
    PidValues ServoSharpTurnRight;

    // target speed values
    uint16_t targetSpeed_straight;
    uint16_t targetSpeed_normal;//normal turning
    uint16_t targetSpeed_round;
    uint16_t targetSpeed_sharp_turn;
    uint16_t targetSpeed_slow;//slow down speed during straight
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
