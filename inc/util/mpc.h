/*
 * mpc.h
 *
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
#include <queue>
#include <string>

#include "libsc/alternate_motor.h"
#include "libsc/dir_encoder.h"
#include "libsc/lcd_console.h"
#include "libsc/system.h"

#include "util/util.h"

namespace util {
class Mpc {
 public:
  /**
   * Default constructor
   *
   * @param e Pointer to an encoder object
   * @param m Pointer to an AlternateMotor object
   * @param isClockwise Boolean stating whether the motor is rotating in clockwise direction
   */
  explicit Mpc(libsc::DirEncoder* e, libsc::AlternateMotor* m, bool isClockwise);

  ~Mpc();

  // Setters
  /**
   * Sets @c commit_target_flag.
   *
   * @param flag If true, next call to @c DoCorrection() will use the target speed
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

  /**
   * Set the override flag
   *
   * @param force_override Whether to override motor protection
   */
  void SetForceOverride(bool force_override);

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
   * Also commits the user-given target speed if @c commit_target_flag_ is true.
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
  static constexpr float kP = 0.00198;
  static constexpr float kI = 0.002;
  static constexpr float kD = 0.00035;

  struct MotorConstants {
    /**
     * Lower bound of motor power which should not be used for extended periods
     * of time. [0,1000]
     */
    static constexpr uint16_t kLowerBound = 75;
    /**
     * Upper bound of motor power which should not be used for extended periods
     * of time. [0,1000]
     */
    static constexpr uint16_t kUpperBound = 500;
    /**
     * Lower bound of motor power which should never be exceeded.
     * [0,kMotorLowerBound]
     */
    static constexpr uint16_t kLowerHardLimit = 75;
    /**
     * Upper bound of motor power which should never be exceeded.
     * [kMotorUpperBound,1000]
     */
    static constexpr uint16_t kUpperHardLimit = 500;
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
  bool HasSameSign(T val1, T val2) const { return (val1 > 0 && val2 > 0) || (val1 < 0 && val2 < 0); }

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
   * Vector of latest ten encoder values
   */
  std::vector<int32_t> last_ten_encoder_val_ = std::vector<int32_t>();
  /**
   * The average of newest ten values of the encoder in units per second
   */
  float average_encoder_val_ = 0.0;
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

  /**
   * Motor protection override count, in no. of cycles.
   */
  uint8_t force_start_count_ = 0;
  /**
   * Motor direction reversal count, in no. of cycles.
   */
  uint8_t force_reverse_count_ = 0;

  /**
   * Number of cycles to wait when overriding motor protection
   */
  static constexpr uint8_t kOverrideWaitCycles = 10;
  /**
   * Number of cycles to wait when reversing motor direction
   */
  static constexpr uint8_t kReverseWaitCycles = 10;
  /**
   * The minimum encoder value before motor protection kicks in
   */
  static constexpr uint16_t kProtectionMinCount = 250;

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
   * Default constructor.
   *
   * @param mpc Pointer to the @c mpc object.
   */
  explicit MpcDebug(Mpc* mpc);
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
