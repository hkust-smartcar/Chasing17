/*
 * car_manager.h
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Peter Tse (mcreng), David Mak (Derppening)
 *
 * CarManager class
 * Stores an important selection of parameters of the current car, including
 * speed, servo degree, slope, side, and identity. This class also provides an
 * interface for other classes to access the motor controller, servo and
 * gyroscope.
 *
 * Usage:
 * Initialize a config, then construct the objects in place or use std::move to
 * move the smart pointer. Call Init(), and the rest will be handled by the
 * class.
 *
 * Getters are designed to be used universally, but Setters aren't. In particular:
 * - SetFeature is exclusively for the camera algorithm.
 * - SetIdentity/SwitchIdentity is exclusively for the overtaking algorithm.
 * - Other Setters are designed to set target values.
 *
 * Prerequisites:
 * - util::MpcDual OR util::Mpc
 * - util::ServoController OR libsc::FutubaS3010
 * - FcYyUsV4
 * - Mpu9250
 *
 */

#ifndef CHASING17_CARMANAGER_H_
#define CHASING17_CARMANAGER_H_

#include <cmath>

#include "libsc/dir_encoder.h"
#include "libsc/futaba_s3010.h"

#include "fc_yy_us_v4.h"

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

  static constexpr ServoBounds kBoundsCar1 = {1040, 755, 470};
  static constexpr ServoBounds kBoundsCar2 = {1145, 845, 545};

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
    enum Side {
      kLeft = 0,
      kRight
    };

    float kP[2];
    float kI[2];
    float kD[2];
  };

  static constexpr const PidValues kMotorPidCar1 = {{7, 7}, {0.07323, 0.03221}, {0, 0}};
  static constexpr const PidValues kMotorPidCar2 = {{0.5, 0.0}, {0.0, 0.0}, {0, 0.0}};
  static constexpr const PidValues kServoPidCar1 = {{1.0, 0}, {0, 0}, {0, 0}};
  static constexpr const PidValues kServoPidCar2 = {{1.0, 0}, {0, 0}, {0, 0}};

  struct ImageSize {
    uint16_t w;
    uint16_t h;
  };

  // Getters
  static ServoAngles GetServoAngles();
  static ServoBounds GetServoBounds();
  static SideRatio GetSideRatio();
  static PidValues GetMotorPidValues();

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
