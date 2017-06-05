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

template<std::size_t size>
bool GetBitValue(const std::array<Byte, size>& a, const std::size_t pos) {
  if (pos >= size * 8) {
    return false;
  }
  return (a.at(pos / 8) >> (7 - pos % 8)) & 1;
}

template<std::size_t size>
bool GetBitValue(const std::array<Byte, size>& a, const std::size_t x_size, const std::size_t x, const std::size_t y) {
  if (x >= x_size) {
    return false;
  }
  std::size_t pos = y * x_size + x;
  return (a.at(pos / 8) >> (7 - pos % 8)) & 1;
}
}  // namespace util
