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
 * - FcYyUsV4
 *
 */

#ifndef CHASING17_ALGORITHM_OVERTAKE_H_
#define CHASING17_ALGORITHM_OVERTAKE_H_

#include <cstdint>
#include <memory>

#include "libsc/system.h"

#include "bluetooth.h"
#include "car_manager.h"
#include "fc_yy_us_v4.h"
#include "util/mpc.h"

class Overtake final {
 public:
  struct Config {
    std::unique_ptr<BTComm> bluetooth = nullptr;
    std::unique_ptr<FcYyUsV4> usir = nullptr;
  };

  /**
   * Executes the overtaking procedure
   *
   * @return true if function should continue running
   */
  static bool ExecuteOvertake();

  static void Init(Config config);

  /**
   * Updates all parameters of the car.
   */
  static void UpdateParameters();

  static uint16_t GetDist() { return dist_; }
//  static bool GetFrontIr() { return ir_front_->IsDetected(); }
  static bool GetFrontIr() {
    // TODO(Derppening): Replace with call to IR (if there is one)
    return false;
  }

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
  static std::unique_ptr<FcYyUsV4> usir_;
};

#endif  // CHASING17_ALGORITHM_OVERTAKE_H_
