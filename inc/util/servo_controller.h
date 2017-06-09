//
// Created by david on 8/6/2017.
//

#ifndef CHASING17_UTIL_SERVO_CONTROLLER_H_
#define CHASING17_UTIL_SERVO_CONTROLLER_H_

#include <memory>

#include "libsc/futaba_s3010.h"
#include "libsc/system.h"

#include "car_manager.h"
#include "mpu9250.h"

namespace util {
class ServoController {
 public:
  ServoController(Mpu9250* c, libsc::FutabaS3010* s);

  ~ServoController();

  void SetTargetAngle(int16_t change, bool commit_now = true);

  libsc::Timer::TimerInt GetTimeElapsed() const { return libsc::System::Time() - time_encoder_start_; }

  void DoCorrection();

 protected:
  bool commit_target_flag_ = false;

 private:
  int16_t CalcAngleDiff(int16_t a, int16_t b) {
    int k = (a - b + 360) % 360;
    return k > 180 ? k - 360 : k;
  }

  void CommitTargetAngle();

  void UpdateCompass();

  /**
  * Constants for encoder to motor value conversions
  */
  static constexpr float kP = 0.00198;
  static constexpr float kI = 0;
  static constexpr float kD = 0.00035;

  uint16_t last_compass_angle_ = 0;
  uint16_t target_compass_angle_ = 0;
  int16_t servo_change_target_ = 0;
  uint16_t average_compass_val_ = 0;

  int16_t prev_error_ = 0;

  std::vector<uint16_t> last_ten_compass_val_ = std::vector<uint16_t>(11);

  libsc::Timer::TimerInt time_encoder_start_ = 0;
  libsc::Timer::TimerInt last_compass_duration_ = 0;

  std::shared_ptr<Mpu9250> compass_ = nullptr;
  std::shared_ptr<libsc::FutabaS3010> servo_ = nullptr;
};
}

#endif  // CHASING17_UTIL_SERVO_CONTROLLER_H_
