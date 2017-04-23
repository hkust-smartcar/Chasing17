/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening), Peter Tse (mcreng)
 *
 * Motor Power Controller class
 *
 * Implements a PID Controller system using encoders to make speed
 * adjustments to the motor power. Allows getting and target-setting of
 * encoder values.
 *
 * Prerequisites:
 * - libsc::AlternateMotor
 * - libsc::DirEncoder
 *
 */

#ifndef CHASING17_UTIL_MPC_H_
#define CHASING17_UTIL_MPC_H_

#include <memory>
#include <string>
#include <deque>

#include "libsc/alternate_motor.h"
#include "libsc/dir_encoder.h"
#include "libsc/lcd_console.h"
#include "libsc/system.h"

#include "util/util.h"

namespace util {
class Mpc {
 public:
  /**
   * Constructor accepting an already-created encoder object.
   *
   * @param e Pointer to an encoder object
   * @param m Pointer to an AlternateMotor object
   */
  explicit Mpc(libsc::DirEncoder* e, libsc::AlternateMotor* m)
      : motor_(m), encoder_(e) {
    motor_->SetPower(0);
    UpdateEncoder();
  }

  ~Mpc() {
    encoder_.reset();
    motor_.reset();
  }

  // Setters
  /**
   * Sets @c commit_target_flag.
   *
   * @param b If true, next call to @c DoCorrection() will use the target speed
   * for correction. Otherwise the current speed will be used.
   */
  void SetCommitFlag(bool flag) { commit_target_flag_ = flag; }
  /**
   * Sets the target speed.
   *
   * @param speed Speed in units per second; value will be directly compared
   * with encoder values.
   * @param commit_now Whether to commit the target speed immediately. This will
   * also reset the encoder values.
   */
  void SetTargetSpeed(const int16_t speed, bool commit_now = true);
  /**
   * Adds to the target speed.
   *
   * @param d_speed Speed difference in units per second; value will be directly
   * compared with encoder values.
   * @param commit_now Whether to commit the target speed immediately. This will
   * also reset the encoder values.
   */
  void AddToTargetSpeed(const int16_t d_speed, bool commit_now = true);

  // Getters
  /**
   * @return The time elapsed between now and last time the encoder values
   * were reset.
   */
  inline libsc::Timer::TimerInt GetTimeElapsed() const { return libsc::System::Time() - time_encoder_start_; }
  /**
   * @return Current target speed.
   */
  inline int16_t GetTargetSpeed() const { return target_speed_; }
  /**
   * @return Current speed in encoder value per second
   */
  inline int32_t GetCurrentSpeed() const { return average_encoder_val_; }

  /**
   * Does motor power correction using encoder, and resets the encoder count.
   * Also commits the user-given target speed if @c commit_target_flag is true.
   */
  void DoCorrection();

 protected:
  Mpc() {};

  /**
   * Whether to commit the user-defined target speed on next call to
   * @c DoCorrection()
   */
  bool commit_target_flag_ = false;

 private:
  /**
   * Constants for encoder to motor value conversions
   */
  static float kP, kI, kD;
  enum struct MotorConstants {
    /**
     * Lower bound of motor power which should not be used for extended periods
     * of time. [0,1000]
     */
        kLowerBound = 75,
    /**
     * Upper bound of motor power which should not be used for extended periods
     * of time. [0,1000]
     */
        kUpperBound = 500,
    /**
     * Lower bound of motor power which should never be exceeded.
     * [0,kMotorLowerBound]
     */
        kLowerHardLimit = 75,
    /**
     * Upper bound of motor power which should never be exceeded.
     * [kMotorUpperBound,1000]
     */
        kUpperHardLimit = 500,
  };

  /**
   * Commits the target speed.
   */
  void CommitTargetSpeed();
  /**
   * Updates the encoder value and resets the encoder
   */
  void UpdateEncoder();

  /**
   * Compares if two variables have the same sign.
   *
   * @tparam T Any numeric type
   * @param val1 First value
   * @param val2 Second value
   * @return True if both variables have the same sign
   */
  template<typename T>
  bool HasSameSign(T val1, T val2) const { return (val1 >= 0 && val2 >= 0) || (val1 < 0 && val2 < 0); }

  // Speed-related variables
  /**
   * Current reference target speed
   */
  int16_t curr_speed_ = 0;
  /**
   * User-defined target speed
   */
  int16_t target_speed_ = 0;
  /**
   * Last encoder value
   */
  int32_t last_encoder_val_ = 0;
  /**
   * Queue of latest ten encoder values
   */
  std::deque<int32_t> last_ten_encoder_val_;
  /**
   * The average of newest ten values of the encoder in units per second
   */
  int32_t average_encoder_val_ = 0;
  /**
   * Cumalative error
   */
  float cum_error_ = 0;
  /**
   * Error of previous execution
   */
  int32_t prev_error_ = 0;

  // Timekeepers
  /**
   * When the current encoder cycle started.
   */
  libsc::Timer::TimerInt time_encoder_start_ = 0;
  /**
   * How long the last encoder cycle lasted.
   */
  libsc::Timer::TimerInt last_encoder_duration_ = 0;

 private:
  std::shared_ptr<libsc::AlternateMotor> motor_;
  std::shared_ptr<libsc::DirEncoder> encoder_;

  friend class MpcDebug;
  friend class MpcDualDebug;
};

/**
 * Debug class for @c Mpc. Provides access to private variables for debugging purposes.
 */
class MpcDebug {
 public:
  /**
   * Constructor which accepts an already-created @c Mpc object.
   *
   * @param mpc Pointer to the @c mpc object.
   */
  explicit MpcDebug(Mpc* mpc) : mpc_(mpc) {};
  /**
   * Outputs the encoder value (in units per second) and power of the managed motor.
   *
   * @param console Pointer to a console object
   */
  void OutputEncoderMotorValues(libsc::LcdConsole* console) const;
  /**
   * Outputs the previous encoder duration and value.
   *
   * @param console Pointer to a console object
   */
  void OutputLastEncoderValues(libsc::LcdConsole* console) const;

  // Setters
  /**
   * Manually sets the power of the motor. [0,1000]
   *
   * @note Motor power will be overriden when next @c DoCorrection() is called
   *
   * @param pwr Power of the motor
   * @param is_clockwise True if the motor should be spinning clockwise
   */
  void SetMotorPower(uint16_t power, bool is_clockwise);

  // Getters
  /**
   * @return The period of the last encoder execution.
   */
  inline libsc::Timer::TimerInt GetLastRunDuration() const { return mpc_->last_encoder_duration_; }
  /**
   * @return The encoder value in units per second
   */
  inline int32_t GetEncoderVal() const { return mpc_->average_encoder_val_; }

 protected:
  MpcDebug() {};

 private:
  std::unique_ptr<Mpc> mpc_;
};
}  // namespace util

#endif  // CHASING17_UTIL_MPC_H_
