/*
 * servo_controller.h
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening), Peter Tse (mcreng)
 *
 * Servo Controller class
 *
 * Implements a PID Controller system using servo values to make adjustments
 * to the servo values. Allows getting and target-setting of servo angles using
 * degrees (instead of servo values).
 *
 * Prerequisites:
 * - CarManager
 *
 */

#ifndef CHASING17_UTIL_SERVO_CONTROLLER_H_
#define CHASING17_UTIL_SERVO_CONTROLLER_H_

#include <memory>
#include <vector>

#include "libsc/futaba_s3010.h"
#include "libsc/lcd_console.h"
#include "libsc/system.h"

#include "car_manager.h"

namespace util {
class ServoController {
 public:
  /**
   * Default constructor
   *
   * @param s Pointer to a FutabaS3010 object
   */
  explicit ServoController(libsc::FutabaS3010* s);

  ~ServoController();

  // Setters
  /**
   * Sets the target angle.
   *
   * @param change Target angle in degrees.
   * @param commit_now Whether to commit the target angle immediately. This will
   * also update the servo angles.
   */
  void SetTargetAngle(int16_t change, bool commit_now = true);
  /**
   * Sets @c commit_target_flag_
   * @param flag If true, next call to @c DoCorrection() will use the target
   * angle for correction. Otherwise, the current angle will be used.
   */
  void SetCommitFlag(bool flag) { commit_target_flag_ = flag; }

  // Getters
  /**
   * @return The current servo angle, in servo values.
   */
  uint16_t GetRawAngle() { return servo_->GetDegree(); }
  /**
   * @return Time elapsed since the previous call to @c DoCorrection()
   */
  libsc::Timer::TimerInt GetTimeElapsed() const { return libsc::System::Time() - time_servo_start_; }

  /**
   * Does servo PD correction, and updates the servo values. Also commits the
   * user-given target angle is @c commit_target_flag_ is set to true.
   */
  void DoCorrection();

 protected:
  ServoController() {};

  /**
   * Whether to commit the user-defined target angle on next call to
   * @c DoCorrection()
   */
  bool commit_target_flag_ = false;

 private:
  void CommitTargetAngle();

  /**
  * Constants for encoder to motor value conversions
  */
  static constexpr float kP = 1;
  static constexpr float kD = 0.001;

  /**
   * Servo target angle for the previous call to @c DoCorrection()
   */
  int16_t last_servo_target_ = 0;
  /**
   * Servo target angle for the current call to @c DoCorrection()
   */
  int16_t curr_servo_target_ = 0;
  /**
   * Temporary value for to-be-committed target angle
   */
  int16_t servo_target_ = 0;

  /**
   * Error of the previous execution
   */
  float last_servo_error_ = 0.0;
  /**
   * The average of the most recent servo values in angle per second
   */
  float average_servo_val_ = 0;
  /**
   * Vector of latest five encoder values
   */
  std::vector<int16_t> last_servo_vals_ = std::vector<int16_t>();

  /**
   * When the current servo cycle started.
   */
  libsc::Timer::TimerInt time_servo_start_ = 0;
  /**
   * How long the previous servo cycle lasted.
   */
  libsc::Timer::TimerInt last_servo_duration_ = 0;

  std::shared_ptr<libsc::FutabaS3010> servo_ = nullptr;

  friend class ServoControllerDebug;
};

class ServoControllerDebug final {
 public:
  /**
   * Default constructor
   *
   * @param servo_ctrl Pointer to a @c ServoController object.
   */
  explicit ServoControllerDebug(ServoController* servo_ctrl);

  /**
   * Outputs the raw servo value, target value (in degrees), previous value (in degrees),
   * and previous error (in degrees per second).
   *
   * @param console Pointer to a console object.
   */
  void OutputServoValues(libsc::LcdConsole* console) const;
  /**
   * Outputs all 5 error values, as well as an average of them.
   *
   * @param console Pointer to a console object.
   */
  void OutputErrorValues(libsc::LcdConsole* console) const;

  /**
   * @return The duration of the previous servo cycle.
   */
  libsc::Timer::TimerInt GetLastRunDuration() const { return servo_controller_->last_servo_duration_; }

 protected:
  ServoControllerDebug() {};

 private:
  std::unique_ptr<ServoController> servo_controller_;
};
}

#endif  // CHASING17_UTIL_SERVO_CONTROLLER_H_
