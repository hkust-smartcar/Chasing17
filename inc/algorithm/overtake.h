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
  };

  /**
   * Executes the overtaking procedure
   *
   * @return true if function should continue running
   */
  static bool ExecuteOvertake();

  static void Init(Config config);

//  static bool GetFrontIr() { return ir_front_->IsDetected(); }
  static bool GetFrontIr() {
    // TODO(Derppening): Replace with call to IR (if there is one)
    return false;
  }

 private:
  // Static class. Disable constructor
  Overtake() {}

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
  static bool DecideOvertake();

  /**
   * Constant for timeout
   */
  static constexpr uint16_t kOvertakeTimeout = 5000;

  /**
   * Whether we are overtaking right now
   */
  static bool is_overtaking_;

  static libsc::Timer::TimerInt time_begin_;

  static std::unique_ptr<BTComm> bluetooth_;
};

#endif  // CHASING17_ALGORITHM_OVERTAKE_H_
