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
#include <memory>

#include "libbase/misc_types.h"
#include "libsc/dir_encoder.h"
#include "libsc/futaba_s3010.h"

#include "fc_yy_us_v4.h"
#include "util/mpc.h"
#include "util/mpc_dual.h"
#include "util/servo_controller.h"

// forward declaration for util::Mpc, util::MpcDual, util::ServoController
namespace util {
class Mpc;
class MpcDual;
class ServoController;
}

class CarManager final {
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
   * Configuration struct. Used for initialization of CarInfo.
   *
   * Usage:
   * Construct objects in place or use std::move to move smart pointer.
   */
  struct Config {
    std::unique_ptr<util::Mpc> epc_left = nullptr;
    std::unique_ptr<util::Mpc> epc_right = nullptr;
    std::unique_ptr<util::MpcDual> epc = nullptr;
    std::unique_ptr<util::ServoController> servo_controller = nullptr;
    std::unique_ptr<libsc::FutabaS3010> servo = nullptr;
    std::unique_ptr<FcYyUsV4> usir = nullptr;
//    std::unique_ptr<Mpu9250> mpu = nullptr;
    Identity identity;
    Car car;
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

  static constexpr const PidValues kMotorPidCar1 = {{0, 0}, {0, 0}, {0, 0}};
  static constexpr const PidValues kMotorPidCar2 = {{0.005, 0.0}, {0.0025, 0.0}, {0, 0.0}};
  static constexpr const PidValues kServoPidCar1 = {{0, 0}, {0, 0}, {0, 0}};
  static constexpr const PidValues kServoPidCar2 = {{0, 0}, {0, 0}, {0, 0}};

  static PidValues GetMotorPidValues();

  /**
   * Update all parameters of the car (speed, slope, servo angle, distance)
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
  static uint16_t GetDistance() { return us_distance_; }
  /**
   * @return Current speed of left wheel in encoder values
   */
  static int32_t GetLeftSpeed() { return left_speed_; }
  /**
   * @return Current speed of right wheel in encoder values
   */
  static int32_t GetRightSpeed() { return right_speed_; }
  /**
   * @return Current servo angle in servo values
   */
  static uint16_t GetServoDeg() { return servo_deg_; }
  static int8_t GetSlope() { return slope_deg_; }
  static Side GetSide() { return side_; }
  static Feature GetFeature() { return feature_; }
  static Identity GetIdentity() { return identity_; }
  static Car GetCar() { return car_; }
  static ServoAngles GetServoAngles();
  static ServoBounds GetServoBounds();
  static SideRatio GetSideRatio();

  // Setters
  static void SetFeature(const Feature f) { feature_ = f; }
  static void SetIdentity(const Identity i) { identity_ = i; }
  static void SetOverrideProtection(const bool override_protection, const MotorSide side = MotorSide::kBoth);
  static void SetSide(const Side s) { side_ = s; }
  /**
   * @param angle Target angle in degrees
   */
  static void SetTargetAngle(const int16_t angle);
  /**
   * @param speed Target speed in encoder values
   * @param side Where the target speed should be committed to
   */
  static void SetTargetSpeed(const int16_t speed, MotorSide side = MotorSide::kBoth);
  static void SwitchIdentity();

 private:
  // Static class. Disable constructor
  CarManager() {}

  // Updaters
  static void UpdateDistance();
  static void UpdateSpeed();
  static void UpdateSlope();
  static void UpdateServoAngle();

  static uint16_t us_distance_;
  static int32_t left_speed_;
  static int32_t right_speed_;
  static uint16_t servo_deg_;
  static int8_t slope_deg_;
  static Side side_;
  static Feature feature_;
  static Identity identity_;
  static Car car_;

  static std::unique_ptr<util::Mpc> epc_left_;
  static std::unique_ptr<util::Mpc> epc_right_;
  static std::unique_ptr<util::MpcDual> epc_;
  static std::unique_ptr<util::ServoController> servo_controller_;
  static std::unique_ptr<libsc::FutabaS3010> servo_;
//  static std::unique_ptr<Mpu9250> mpu_;
  static std::unique_ptr<FcYyUsV4> usir_;
};

#endif  // CHASING17_CARMANAGER_H_
