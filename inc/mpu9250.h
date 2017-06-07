/*
 * mpu9250.h
 *
 * Author: Harrison Ng, Ming Tsang, David Mak
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#ifndef CHASING17_MPU9250_H_
#define CHASING17_MPU9250_H_

#include <cstdint>
#include <array>

#include "libbase/misc_types.h"
#include "libbase/k60/i2c_master.h"
#include "libbase/k60/pin.h"
#include "libbase/k60/soft_i2c_master.h"

#define USE_SOFT_MPU9250 true

class Mpu9250 {
  static constexpr bool kUseSoftI2c = USE_SOFT_MPU9250;

 public:
#if USE_SOFT_MPU9250
  using I2cMaster = libbase::k60::SoftI2cMaster;
#else
  using I2cMaster = libbase::k60::I2cMaster;
#endif  // USE_SOFT_MPU9250

  static constexpr libbase::k60::Pin::Name kScl = libbase::k60::Pin::Name::kDisable;
  static constexpr libbase::k60::Pin::Name kSda = libbase::k60::Pin::Name::kDisable;

  struct Config {
    enum struct Range {
      kSmall = 0,
      kMid,
      kLarge,
      kExtreme,
    };

    // kSmall -> kExtreme = ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
    Range gyro_range;
    // kSmall -> kExtreme = ±2g, ±4g, ±8g, ±16g
    Range accel_range;

    /// Calibrate the gyroscope while initializing
    bool cal_drift = false;

    I2cMaster* i2c_master_ptr = nullptr;
  };

  explicit Mpu9250(const Config& config);

  bool Update(bool clamp_ = true);
  bool UpdateF(bool clamp_ = true);

  const std::array<int32_t, 3>& GetAccel() const {
    return m_accel;
  }
  const std::array<float, 3>& GetAccelF() const {
    return m_accel_f;
  }

  const std::array<int32_t, 3>& GetOmega() const {
    return m_omega;
  }
  const std::array<float, 3>& GetOmegaF() const {
    return m_omega_f;
  }

  float GetCelsius() const {
    return m_temp;
  }

  bool IsCalibrated() const {
    return m_is_calibrated;
  }

  const std::array<int32_t, 3>& GetOffset() const {
    return m_omega_offset;
  }
  const std::array<float, 3>& GetOffsetF() const {
    return m_omega_offset_f;
  }

  bool Verify();

  I2cMaster* GetI2cMaster() {
    return m_i2c;
  }

  const uint16_t GetGyroScaleFactor(void) const {
    return m_gyro_scale_factor;
  }

  const uint16_t GetAccelScaleFactor(void) const {
    return m_accel_scale_factor;
  }

 private:

  void Calibrate();
  void CalibrateF();

  uint16_t GetGyroScaleFactor();
  uint16_t GetAccelScaleFactor();

  I2cMaster::Config GetI2cConfig();

  I2cMaster* m_i2c;
  std::array<int32_t, 3> m_accel;
  std::array<int32_t, 3> m_omega;
  std::array<float, 3> m_accel_f;
  std::array<float, 3> m_omega_f;
  std::array<int32_t, 3> m_omega_offset;
  std::array<float, 3> m_omega_offset_f;
  float m_temp = 0.0;
  bool m_is_calibrated = false;
  uint16_t m_gyro_scale_factor;
  uint16_t m_accel_scale_factor;

  Config::Range m_gyro_range;
  Config::Range m_accel_range;
};

#endif  // CHASING17_MPU9250_H_
