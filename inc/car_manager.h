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

  /**
   * Struct of servo angles, stored in degrees
   */
  struct ServoAngles {
    uint8_t kLeftAngle;
    uint8_t kRightAngle;
  };

  static constexpr ServoAngles kAnglesCar1 = {36, 38};
  static constexpr ServoAngles kAnglesCar2 = {38, 41};

  static constexpr float kWheelbase = 19.65;
  static constexpr float kAxleLength = 15.2;

  /**
   * Struct of side ratios, i.e. the speed ratio between the side of the car and the middle of the car.
   */
  struct SideRatio {
    float kLeft;
    float kRight;
  };

  static constexpr SideRatio kRatioCar1 = {std::tan(kAnglesCar1.kLeftAngle), std::tan(kAnglesCar1.kRightAngle)};
  static constexpr SideRatio kRatioCar2 = {std::tan(kAnglesCar2.kLeftAngle), std::tan(kAnglesCar2.kRightAngle)};

  struct PidValues {
    float kP;
    float kI;
    float kD;
  };

  struct ImageSize {
    uint16_t w;
    uint16_t h;
  };

  // Getters
  static ServoAngles GetServoAngles();
  static ServoAngles GetServoAngles(Car);
  static SideRatio GetSideRatio();
  static SideRatio GetSideRatio(Car);

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
