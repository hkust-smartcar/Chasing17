/*
 * fc_yy_us_v4.h
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Leslie Lee (LeeChunHei), Peter Tse (mcreng)
 *
 * FcYyUsV4 class
 * Interface for Freecar YingYang Ultrasonic Sensor v4.
 *
 * Usage:
 * Pass the Pin::Name of the sensor to the constructor. No additional setup
 * will be required.
 *
 */

#ifndef CHASING17_FCYYUSV4_H_
#define CHASING17_FCYYUSV4_H_

#include <limits>
#include <vector>

#include "libbase/k60/gpio.h"

using libbase::k60::Gpi;
using libbase::k60::Pin;

class FcYyUsV4 {
 public:
  static constexpr unsigned int kMinDistance = 0;
  static constexpr unsigned int kMaxDistance = 0;//std::numeric_limits<unsigned int>::max();

  /**
   * @param pin Name of Pin connected to the sensor
   */
  FcYyUsV4(Pin::Name pin); //corresponds to I2C0_SCL: kPtb0

  /**
   * @return The distance measured by the sensor (mm)
   */
  unsigned int GetDistance() const { return distance_; }
  unsigned int GetAvgDistance() const { return average_distance_; }

 private:
  static void listener(Gpi* gpi);

  Gpi m_pin_;
  Gpi::Config gpi_config_;

  static uint32_t impulse_start_time_;
  static unsigned int distance_;

  static std::vector<unsigned int> last_ten_distance_;
  static unsigned int average_distance_;
};

#endif  // CHASING17_FCYYUSV4_H_
