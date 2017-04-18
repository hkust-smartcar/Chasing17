/* overtake.h
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening), Peter Tse (mcreng)
 *
 * Overtake class
 *
 * Implements the decision-making flow and execution flow of the overtaking
 * action of the cars.
 *
 * Prerequisites:
 * - CarInfo
 * - BTComm
 * - libsc::InfraRedSensor
 * - libsc::Us100
 *
 */

#ifndef CHASING17_ALGORITHM_OVERTAKE_H_
#define CHASING17_ALGORITHM_OVERTAKE_H_

#include <cstdint>
#include <memory>

#include "libsc/infra_red_sensor.h"
#include "libsc/system.h"
#include "libsc/us_100.h"

#include "bluetooth.h"
#include "car_manager.h"
#include "util/mpc.h"

class Overtake final {
 public:
  struct Config {
    std::unique_ptr<BTComm> bluetooth = nullptr;
    std::unique_ptr<libsc::InfraRedSensor> ir_front = nullptr;
    std::unique_ptr<libsc::InfraRedSensor> ir_rear = nullptr;
    std::unique_ptr<libsc::Us100> us = nullptr;
  };

  /**
   * Executes the overtaking procedure
   *
   * @return true if function should continue running
   */
  // TODO(Derppening): decide if we need to return anything
  static bool ExecuteOvertake();

  static void Init(Config config);

  /**
   * Updates all parameters of the car.
   */
  static void UpdateParameters();

  static uint16_t GetDist() { return dist_; }
  static bool GetFrontIr() { return ir_front_->IsDetected(); }

 private:
  // Static class. Disable constructor
  Overtake() {}

  /**
   * Gets the distance between the two cars
   */
  static void UpdateDist();

  /**
   * Cleanup post-overtake.
   */
  static void Cleanup(bool);

  /**
   * Decides whether to execute overtaking
   *
   * @note Only called if algorithm is running on the back car
   * @return true if algorithm decides to overtake
   */
  // TODO(Derppening): decide if we need to return anything
  static bool DecideOvertake();

  /**
   * Constant for too much distance
   */
  static constexpr uint8_t kDistanceInf = 0xFF;

  /**
   * Constant for timeout
   */
  static constexpr uint16_t kOvertakeTimeout = 5000;

  /**
   * Whether we are overtaking right now
   */
  static bool is_overtaking_;

  static uint16_t dist_;
  static libsc::Timer::TimerInt time_begin_;

  static std::unique_ptr<BTComm> bluetooth_;
  static std::unique_ptr<libsc::InfraRedSensor> ir_front_;
  static std::unique_ptr<libsc::InfraRedSensor> ir_rear_;
  static std::unique_ptr<libsc::Us100> us_;
};

#endif  // CHASING17_ALGORITHM_OVERTAKE_H_
