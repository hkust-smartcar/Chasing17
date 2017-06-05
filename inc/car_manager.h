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
 * - libsc::FutubaS3010
 * - libsc::Mpu6050
 *
 */

#ifndef CHASING17_CARMANAGER_H_
#define CHASING17_CARMANAGER_H_

#include <memory>

#include "libbase/misc_types.h"
#include "libsc/dir_encoder.h"
#include "libsc/mpu6050.h"
#include "libsc/futaba_s3010.h"

#include "util/mpc.h"
#include "util/mpc_dual.h"

class CarManager final {
 public:
  /**
   * Enum of possible features
   */
  enum struct Feature : uint8_t {
    kStraight = 0,
    kRoundabout,
    kCross
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
   * Enum of car identity, i.e. whether it is old or new
   */
  enum struct Car : bool {
    kCar1,
    kCar2
  };

  /**
   * Configuration struct. Used for initialization of CarInfo.
   *
   * Usage:
   * Construct objects in place or use std::move to move smart pointer.
   */
  struct Config {
    std::unique_ptr<util::Mpc> epc_left = nullptr;
    std::unique_ptr<util::Mpc> epc_right = nullptr;
    std::unique_ptr<util::MpcDual> epc = nullptr;
    std::unique_ptr<libsc::Mpu6050> accel = nullptr;
    std::unique_ptr<libsc::FutabaS3010> servo = nullptr;
    Identity identity;
    Car car;
  };

  struct ServoBounds {
    uint16_t kLeftBound;
    uint16_t kCenter;
    uint16_t kRightBound;
  };

  static constexpr ServoBounds kBoundsCar1 = {1040, 755, 470};
  static constexpr ServoBounds kBoundsCar2 = {1145, 845, 545};

  struct ServoAngles {
    uint8_t kLeftAngle;
    uint8_t kRightAngle;
  };

  static constexpr ServoAngles kAnglesCar1 = {36, 38};
  static constexpr ServoAngles kAnglesCar2 = {38, 41};

  /**
   * Update all parameters of the car (speed, slope, servo angle)
   */
  static void UpdateParameters();

  /**
   * Initializes all smart pointers.
   *
   * @note Use std::move(Config), or else there will be a compilation error.
   *
   * @param config
   */
  static void Init(Config config);

  // Getters
  static uint8_t GetLeftSpeed() { return left_speed_; }
  static uint8_t GetRightSpeed() { return right_speed_; }
  static uint16_t GetServoDeg() { return servo_deg_; }
  static int8_t GetSlope() { return slope_deg_; }
  static Side GetSide() { return side_; }
  static Feature GetFeature() { return feature_; }
  static Identity GetIdentity() { return identity_; }
  static Car GetCar() { return car_; }
  static ServoBounds GetServoBounds() {
    return car_ == CarManager::Car::kCar1 ? CarManager::kBoundsCar1 : CarManager::kBoundsCar2;
  }
  static ServoAngles GetServoAngles() {
    return car_ == CarManager::Car::kCar1 ? CarManager::kAnglesCar1 : CarManager::kAnglesCar2;
  }

  // Setters
  static void SetFeature(const Feature f) { feature_ = f; }
  static void SetIdentity(const Identity i) { identity_ = i; }
  static void SetOverrideProtection(const bool override_protection, const MotorSide side);
  static void SetSide(const Side s) { side_ = s; }
  static void SetTargetAngle(uint16_t angle);
  static void SetTargetSpeed(MotorSide src, int16_t speed);
  static void SwitchIdentity();

 private:
  // Static class. Disable constructor
  CarManager() {}

  // Updaters
  static void UpdateSpeed();
  static void UpdateSlope();
  static void UpdateServoAngle();

  static uint8_t left_speed_;
  static uint8_t right_speed_;
  static uint16_t servo_deg_;
  static int8_t slope_deg_;
  static Side side_;
  static Feature feature_;
  static Identity identity_;
  static Car car_;

  static std::unique_ptr<util::Mpc> epc_left_;
  static std::unique_ptr<util::Mpc> epc_right_;
  static std::unique_ptr<util::MpcDual> epc_;
  static std::unique_ptr<libsc::Mpu6050> accel_;
  static std::unique_ptr<libsc::FutabaS3010> servo_;
};

#endif  // CHASING17_CARMANAGER_H_
