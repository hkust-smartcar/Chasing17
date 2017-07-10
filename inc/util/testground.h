/*
 * testground.h
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Testground for whatever you want.
 *
 */

#ifndef CHASING17_UTIL_TESTGROUND_MAIN_H_
#define CHASING17_UTIL_TESTGROUND_MAIN_H_

#include <cstdint>

namespace util {
namespace testground {

extern uint16_t config;

struct TestVals {
  int32_t i;
  float f;
};

extern TestVals t1;
extern TestVals t2;
extern TestVals vals;

void main();

}  // namespace testground
}  // namespace util

#endif  // CHASING17_UTIL_TESTGROUND_MAIN_H_
