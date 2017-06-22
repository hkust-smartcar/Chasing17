/*
 * mpc_dual.h
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 *
 * Dual Motor Power Controller class
 * Implements a dual-motor Proportional Controller system using two Mpc classes
 * to make speed adjustments to motor power. Also adds software differential
 * feature.
 *
 * Prerequisites:
 * - libsc::AlternateMotor
 * - libsc::DirEncoder
 * - libsc::FutubaS3010
 * - CarManager
 *
 */

#ifndef CHASING17_UTIL_MPC_DUAL_H_
#define CHASING17_UTIL_MPC_DUAL_H_

#include <memory>
#include <string>

#include "libsc/alternate_motor.h"
#include "libsc/dir_encoder.h"
#include "libsc/lcd_console.h"
#include "libsc/system.h"

#include "car_manager.h"
#include "util/mpc.h"
#include "util/util.h"

namespace util {
class MpcDual final : protected Mpc {
 public:
  enum struct MotorSide {
    kLeft = 0,
    kRight,
    kBoth
  };

  /**
   * Constructor accepting already-created encoder and motor objects.
   *
   * @note Swapping the left/right motors/encoders will have undesirable
   * effects which could hinder the car performance!
   *
   * @param motor_left Pointer to left AlternateMotor
   * @param motor_right Poitner to right AlternateMotor
   * @param encoder_left Pointer to left DirEncoder
   * @param encoder_right Pointer to right DirEncoder
   */
  MpcDual(libsc::AlternateMotor* motor_left,
                   libsc::AlternateMotor* motor_right,
                   libsc::DirEncoder* encoder_left,
                   libsc::DirEncoder* encoder_right);

  ~MpcDual();

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
  void AddToTargetSpeed(const int16_t speed, bool commit_now = true);

  /**
   * Set the override flag
   *
   * @param force_override Whether to override motor protection
   * @param side Which side to commit to
   */
  void SetForceOverride(bool force_override, MotorSide side = MotorSide::kBoth);

  // Getters
  /**
   * @param side Which encoder to retrieve the time from.
   *
   * @return The time elapsed between now and last time the encoder values
   * were reset.
   */
  libsc::Timer::TimerInt GetTimeElapsed(MotorSide side) const;
  /**
   * @param side Which encoder to retrieve the speed from.
   *
   * @return Current speed in encoder value per second
   */
  int32_t GetCurrentSpeed(MotorSide side) const;

  /**
   * Does motor power correction using encoder, and resets the encoder count.
   * Also commits the user-given target speed if @c commit_target_flag is true.
   *
   * @note The final target speed may not be equal to the given target speed,
   * due to software differential calculations. This is normal.
   *
   * @params
   */
  void DoCorrection();

  float motor_speed_diff_;

 private:
  std::unique_ptr<Mpc> mpc_left_;
  std::unique_ptr<Mpc> mpc_right_;

  friend class MpcDualDebug;
};

class MpcDualDebug final : protected MpcDebug {
 public:
  explicit MpcDualDebug(MpcDual* mpc_dual);

  void OutputEncoderMotorValues(libsc::LcdConsole* console, MpcDual::MotorSide side) const;
  void OutputLastEncoderValues(libsc::LcdConsole* console, MpcDual::MotorSide side) const;
  void OutputPidValues(libsc::LcdConsole* console, MpcDual::MotorSide side = MpcDual::MotorSide::kBoth) const;

  // Setters
  void SetMotorPower(uint16_t power, MpcDual::MotorSide side, bool is_clockwise);

  // Getters
  libsc::Timer::TimerInt GetLastRunDuration(MpcDual::MotorSide side) const;

  int32_t GetEncoderVal(MpcDual::MotorSide side) const;

 private:
  std::unique_ptr<MpcDual> mpc_dual_;
};
}  // namespace util

#endif //CHASING17_UTIL_MPC_DUAL_H_
