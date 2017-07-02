/*
 * util.tcc
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 *
 * Template implementations for util.h
 *
 */

#include "util/util.h"

#include <cstring>

namespace util {
template<std::size_t size>
void CopyByteArray(const Byte* const src, std::array<Byte, size>& dest) {
  std::memcpy(dest.data(), src, size);
}
}  // namespace util
